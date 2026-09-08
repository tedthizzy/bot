# The box

One 27B vision-language model behind an OpenAI-compatible endpoint. That is the
whole contract. The box is in no safety path, holds no robot state, and is never
assumed present — a dead box degrades the rover to the router's local intents
(`stop`, `forward`, `back`, `left`, `right`, `say`), which need no network at
all.

Everything here is a convenience. If you already serve a VLM at
`/v1/chat/completions` that accepts an image and honours
`response_format: json_schema`, point `ROVER__BOX__URL` at it and skip to
[the one curl](#the-one-curl).

## Bring-up (10 minutes, in parallel with the firmware flash)

```bash
cp box/.env.example box/.env          # fill in HF_TOKEN if the weights are gated
docker compose --env-file box/.env -f docker/compose.box.yml up -d vllm
docker compose -f docker/compose.box.yml logs -f vllm     # wait for "Application startup complete"
```

First start downloads ~20 GB and takes several minutes; `HF_CACHE` keeps it.

Three things to check **before** build day, because each one fails at start-up
with no diagnosis path:

1. `docker manifest inspect vllm/vllm-openai:v0.28.0` — pin the exact 0.28.x
   patch, then pin its digest.
2. `vllm serve --help | grep default-chat-template-kwargs` — the one flag in
   ARCHITECTURE 5.7 that no source confirms exists. It is deliberately **not**
   in the compose file; `enable_thinking: false` is sent client-side on every
   request instead (A16).
3. `NCCL_P2P_DISABLE=1` is already in the service environment. GeForce drivers
   ship without peer-to-peer, and without it TP=2 hangs at init. If it still
   hangs, uncomment `--disable-custom-all-reduce` beside it.

Then, from the Pi or the Mac:

```bash
python -m rover_brain.box_probe --url "$ROVER__BOX__URL"
```

Five calls, ~30 seconds. It writes `box_caps.json`: `supports_json_schema`,
`supports_oneof`, `supports_images`, `image_tokens_observed`, `ttft_cold_ms`,
`ttft_warm_ms`, `decode_tok_s`, and whether TP=2 came up. **If
`supports_oneof` is false, set `[box] structured_output_mode = "json_object"`**
and the compat profile takes over — a flat schema with discrimination in
pydantic. That is the whole of open item 8.

## The one curl

This proves the three things the rover actually needs — the endpoint takes an
image, it honours a strict JSON schema, and it returns one `SkillCall` — through
the real endpoint, with no rover code in the way. `python3` only builds the
request body; the request itself is the single `curl`.

```bash
export ROVER__BOX__URL="http://localhost:8000/v1"
IMG=tests/fixtures/frames/f_kitchen.jpg  # any JPEG will do

python3 - "$IMG" box/schema/skillcall.json <<'PY' | \
curl -sS "$ROVER__BOX__URL/chat/completions" \
     -H 'Content-Type: application/json' \
     -H "Authorization: Bearer ${ROVER_BOX_API_KEY:-none}" \
     --data-binary @- | python3 -m json.tool
import base64, json, pathlib, sys
img, schema = sys.argv[1], sys.argv[2]
b64 = base64.b64encode(pathlib.Path(img).read_bytes()).decode()
print(json.dumps({
    "model": "rover-vlm",
    "messages": [
        {"role": "system", "content": pathlib.Path("box/prompts/system.md").read_text()},
        {"role": "user", "content": [
            {"type": "image_url",
             "image_url": {"url": f"data:image/jpeg;base64,{b64}"}},
            {"type": "text", "text": json.dumps({
                "heading_deg": 0, "battery_pct": 80,
                "obstacle_ahead": False, "front_range_cm": 120,
                "bumper": False, "moving": False,
                "power_cap_pct": 20, "last_result": "done", "last_scene": "",
                "recently_seen": [],
                "allowed_skills": ["drive_for", "turn_to", "stop", "say",
                                   "describe_scene", "find", "set_face"],
                "motion_budget_left": {"seconds": 12}})},
            {"type": "text", "text": "USER: what do you see?"}]}],
    "response_format": {"type": "json_schema", "json_schema": {
        "name": "skill_call", "strict": True,
        "schema": json.loads(pathlib.Path(schema).read_text())}},
    "extra_body": {"chat_template_kwargs": {"enable_thinking": False}},
    "max_tokens": 160, "temperature": 0.0, "top_p": 1.0, "stream": False}))
PY
```

A pass looks like `choices[0].message.content` parsing as one object with
`speech`, `skill` and `args`, and `usage.prompt_tokens` about 300 higher than
the same call without the image — that 300 is A18's 640×480 → 20×15 grid, and
seeing it is how you confirm the image was really processed rather than
silently dropped.

Run the identical command a second time and `usage.prompt_tokens_details.
cached_tokens` must be greater than zero. That is prefix caching working, which
is what makes a validator retry cheap (A17): the retry appends to the same
prefix and never drops the image.

`box/schema/*.json` is generated — `make schemas` writes it from
`rover_contracts`, and CI fails if it drifts. Do not hand-edit it.

## Measuring the latency numbers

ARCHITECTURE 9 budgets the box at **0.15 s TTFT with no image, 0.30–0.70 s with
one**, and **0.20–0.22 s to decode the first sentence** (~20 tokens at the
90–100 tok/s a stock driver gives). Both are column rows in a sum whose gate is
p50 ≤ 2.1 s / p95 ≤ 2.8 s end-of-speech to first audio *with* an image.

Measure them, in this order:

1. **Time to first token.** `box_probe` reports `ttft_cold_ms` and
   `ttft_warm_ms` from five streamed calls. By hand, the equivalent is the same
   curl with `"stream": true` and `-N`, timing the first `data:` line:

   ```bash
   ... | curl -sSN --data-binary @- "$ROVER__BOX__URL/chat/completions" ... \
        -w '\ntotal %{time_total}s  connect %{time_connect}s\n' | \
        awk 'NR==1 {print systime()} /^data:/ && !seen {seen=1; print "first token"}'
   ```

   Take the median of five and report it against the 0.30–0.70 s row. A cold
   first call is not the number; the second and later ones are.

2. **Decode rate.** `usage.completion_tokens` divided by the streamed wall time
   after the first token, on a reply long enough to matter (set
   `max_tokens: 160` and ask for a description). Expect **90–100 tok/s stock**.
   108–114 tok/s appears only on a patched driver with FA2 and fp16 KV on a
   nightly, which this design does not run — if you measure that, say which
   driver you are on before believing the number.

3. **The whole turn.** `make gate-g1 BOX=real` replays
   `tests/gates/g1/utterances.jsonl` — 50 utterances × 3 world states × 3
   frames — and logs p50/p95 TTFT and decode alongside schema validity, skill
   accuracy and the adversarial block's attack-success rate with a confidence
   interval. It writes its result to `logs/gates/`; see `docs/gates.md`. That
   run, not a hand-timed curl, is what settles A15's model pick at int4 with
   thinking disabled.

Record the numbers in `box_caps.json` and in the gate result. A latency claim
without one of those behind it is an assertion, and this project does not count
assertions.
