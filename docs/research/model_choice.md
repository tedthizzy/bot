# Model choice: the ~27B-class open VLM for the rover's command parser + scene describer (state as of 2026-09-07)

## Summary

The brief's model assumptions are from the Gemma 3 era (spring 2025) and two full generations have passed. In 2026 the 27B-class open VLM field that fits 2× RTX 3090 (48 GB) at int4 has collapsed to two serious families, both Apache-2.0, both natively multimodal, both with native tool calling and a vLLM tool parser:

- **Qwen3.8-27B** (dense, released 2026-08-14; predecessors Qwen3.6-27B 2026-04-22 and Qwen3.5-27B 2026-02-24). Best-in-class spatial numbers for a rover: on Roboflow Vision Evals (2026-09-05) it scores 74.7% overall vs Gemma 4 31B's 67.0%, with object detection 65.7% vs 48.2% and counting 64.9% vs 51.4% [MEASURED, hosted API]. Qwen3.6-27B publishes ERQA 62.5, RefSpatialBench 70.0, CountBench 97.8 [VENDOR].
- **Gemma 4 31B** (dense, released 2026-04-02, Apache 2.0, official QAT W4A16 checkpoint). Native function calling (Gemma 3 27B scored 6.6% on τ2-bench, Gemma 4 31B 86.4% [VENDOR]), configurable image token budget (70–1120), best OCR (90.8%) and perfect 30/30 multi-step JSON tool sequences in a 24 GB GPU test [MEASURED]. Weaker detection and counting.

**Recommendation: Qwen3.8-27B primary (Qwen3.6-27B if your vLLM release does not yet load 3.8), Gemma 4 31B QAT W4A16 fallback.** Serve on vLLM ≥0.19 with tensor-parallel 2, thinking disabled per request, and constrain the output with `response_format: json_schema` (xgrammar) where `skill` is an enum. That makes a nonexistent skill unrepresentable at the grammar level. Keep the pydantic bounds check and single retry on the Pi; the grammar cannot reliably enforce numeric ranges.

The brief's "assume Gemma-class, don't rely on native tool-call parsing" no longer holds as a statement about model capability. It still holds as a design choice, for a different reason: for a single fixed skill list, schema-constrained JSON is stricter and simpler than tool-call parsing, and Gemma 4's native tool serialization is a custom non-JSON format you would otherwise have to parse yourself.

## State of the art (2026)

### What fits 2× 3090 at int4, and what does not

| Model | Released | Params | License | Fits 48 GB int4? | vLLM |
|---|---|---|---|---|---|
| Qwen3.8-27B | 2026-08-14 | 27B dense, native vision | Apache 2.0 | Yes; Q4_K_M 15.93 GiB [MEASURED] | `Qwen3.5ForConditionalGeneration` family; version floor for 3.8 not stated on card |
| Qwen3.6-27B / 35B-A3B | 2026-04-22 / 04-16 | 27B dense / 35B MoE 3B active | Apache 2.0 | Yes; Q4_K_M 15.66 GiB [MEASURED] | vLLM ≥0.19.0 [VENDOR] |
| Qwen3.5-27B | 2026-02-24 | 27B dense | Apache 2.0 | Yes | main branch at release, in 0.19+ |
| Gemma 4 31B-it | 2026-04-02 | 30.7B dense | Apache 2.0 | Yes; QAT W4A16 19.8 GB vs 59.0 GB bf16 [VENDOR] | vLLM v0.19.0 (transformers ≥5.5.0), `vllm/vllm-openai:gemma4` image |
| Gemma 4 26B-A4B-it | 2026-04-02 | 25.2B MoE, 3.8B active | Apache 2.0 | Yes, on one GPU; AWQ int4 ~15 GB [community] | same; Google says use `--quantization int8_per_channel_weight_only`, no W4A16 QAT for the MoE |
| Qwen3-VL-32B | 2025-10-21 | 33B dense | Apache 2.0 | Yes | vLLM ≥0.11.0 |
| Mistral Small 3.2 24B (2506) | 2025-06 | 24B dense | Apache 2.0 | Yes (~55 GB bf16, ~15 GB int4) | vLLM ≥0.9.1, `--tokenizer_mode mistral --tool-call-parser mistral` |
| InternVL3.5-38B / 30B-A3B / 20B-A4B | 2025-08 | 38B / MoE | Apache 2.0 | Yes | vLLM recipe exists; tokenizer special-token issues reported |
| Molmo2-8B | 2025-12-11 | 9B (Qwen3-8B + SigLIP2) | Apache 2.0 | Yes, trivially | `Molmo2ForConditionalGeneration` |
| GLM-4.6V-Flash | 2025-12 | 9B | MIT | Yes | vLLM ≥0.12.0 |
| GLM-4.6V | 2025-12 | 106B-A12B | MIT | **No** (~53 GB weights at int4) [INFERRED] | — |
| Mistral Small 4 | 2026-03-16 | 119B MoE | Apache 2.0 | **No** at int4 (~60 GB) [INFERRED] | — |
| Llama 4 Scout | 2025-04 | 109B-A17B | Llama license | **No**; Q4 ~55 GB [community] | `llama4_pythonic` parser |
| Kimi-VL-A3B | 2025-04 | 16B MoE, 2.8B active | MIT | Yes | no LoRA/PP flags; dated |
| MiniCPM-V 4.5 / 4.6 | 2025-08-26 / 2026-05-11 | 8B / 1.3B | Apache 2.0 | Yes | phone-class, not the box's job |
| Pixtral 12B / Large | 2024-09 / 2024-11 | 12B / 124B | Apache 2.0 / research | 12B yes | folded into Mistral Small 4 |

Three things changed since the brief was written:

1. **Licenses converged on Apache 2.0.** Gemma 4 is the first Gemma under Apache 2.0 (Gemma 3 shipped under the Gemma Terms of Use with a prohibited-use policy). Qwen has been Apache 2.0 throughout. The "Gemma terms vs Apache-2.0" question is moot for the two finalists.
2. **Native tool calling became table stakes.** vLLM ships parsers for `gemma4`, `qwen3_coder`/`qwen3_xml`, `mistral`, `glm45`/`glm47`, `llama4_pythonic`, `hermes`, and others. Gemma 4's format is a custom serialization (`<|tool_call>call:func_name{key:<|"|>value<|"|>,num:42}<tool_call|>`), not JSON, parsed by `--tool-call-parser gemma4`. Qwen3.5+ uses the Qwen3-Coder XML-ish format parsed by `qwen3_coder`.
3. **Thinking modes are on by default.** Qwen3.5/3.6/3.8 generate `<think>…</think>` unless you pass `chat_template_kwargs: {"enable_thinking": false}` (or serve with `--default-chat-template-kwargs '{"enable_thinking": false}'`); Qwen3.8 adds a `reasoning_effort` knob. Gemma 4 has `enable_thinking` in `apply_chat_template` and `--reasoning-parser gemma4`. A robot command parser that forgets this gets multi-second reasoning traces before its JSON.

### Vision and spatial abilities that matter for a rover

- **Grounding format.** Qwen3-VL changed from Qwen2.5-VL's absolute pixel coordinates to relative 0–1000, emitting `{"bbox_2d": [x1,y1,x2,y2], "label": ...}` and `{"point_2d": [x,y]}`; the cookbook rescales with `x/1000*width` [VENDOR]. Qwen3.5+ inherits the Qwen3-VL stack and reports RefCOCO 90.9–92.5; I did not find an explicit coordinate-format statement for 3.5/3.6/3.8, so treat 0–1000 relative as [INFERRED] and check on a calibration frame. Gemma 4 emits `{"box_2d": [y1, x1, y2, x2], "label": ...}` in a 1000×1000 normalized space without special prompting [VENDOR]. Note the axis order differs between the two families; the Pi validator must know which model is behind the endpoint.
- **Detection and counting.** Roboflow Vision Evals (hosted APIs, updated 2026-09-05): Qwen3.8-27B object detection 65.7%, counting 64.9%, OCR 92.2%; Qwen3.5-27B 50.5% / 67.6% / 84.7%; Gemma 4 31B 48.2% / 51.4% / 90.8% [MEASURED]. The Qwen3.6-27B card lists CountBench 97.8, RefSpatialBench 70.0 (left/right/near/far relations), and ERQA 62.5 (the embodied reasoning QA set from the Gemini Robotics work) [VENDOR]. Gemma 4 publishes MMMU Pro 76.9% but no grounding or spatial rows.
- **Pointing.** Molmo2-8B (Dec 2025) remains the specialist: points as `(x, y)` scaled by 1000, regex-extracted from text, image and video, Apache 2.0. It is not a command parser, but it fits on the second GPU beside a 27B if `find(object)` needs a precise point.
- **Reading signs / OCR.** Gemma 4 31B (90.8%) and Qwen3.8-27B (92.2%) are within noise of each other on Roboflow's OCR task; either reads room signs at 640×480.
- **Depth/distance.** No open 27B-class VLM gives metric depth; both families answer relative near/far. Use the VL53L1X ToF sensors for distance in the safety chain, not the model. No benchmark evidence was found for metric estimates from either model.
- **Spatial relations in general** remain weak across all VLMs (2026 papers "The Spatial Blindspot of Vision-Language Models", OmniSpatial, SpatiaLab). Left/right on a single forward-facing frame is fine; multi-view or egocentric-to-allocentric reasoning is not.

### Instruction following and schema adherence under a ~1k-token system prompt

- A 24 GB GPU test (RTX 4090, llama.cpp b10453, Q4_K_M, 2026-08-17) ran 90 single-tool calls and 30 multi-step workflows: Qwen3.8-27B 90/90 and 28/30, Gemma 4 31B 90/90 and 30/30, Qwen3.6-27B 79/90 and 27/30 [MEASURED]. Both finalists produce well-formed calls when the tool spec is in the prompt.
- BFCL-v4 (self-reported, 18 models, updated 2026-09-07): Qwen3.5-27B 0.685, rank 5. Gemma 4 and Qwen3.6/3.8 are not on the board (not submitted, not failed). A community run of Gemma 4 26B-A4B at UD-IQ4_XS on llama.cpp scored 89.13% non-live, 63.80% live, 45.12% multi-turn [MEASURED, RTX 5070 Ti]. Gemma 3 12B sat at rank 78 with a prompted workaround.
- Mistral Small 3.2 reports IFEval 84.78% and cut infinite generations from 2.11% to 1.29% [VENDOR]; it is the safe 2025 fallback if both 2026 families misbehave.
- **Hallucinated skills and refusals:** I found no 2026 measurement of nonexistent-tool hallucination or refusal rates for Gemma 4 or Qwen3.x under a robot-style system prompt. BFCL's "irrelevance" category is the closest proxy and the scores above do not break it out. Do not rely on the model here; make the skill an enum in the grammar.

### Serving details that changed

- vLLM's guided decoding API was renamed: `guided_json`/`guided_regex`/`guided_choice`/`guided_grammar` were removed in v0.12.0; use `structured_outputs: {json|regex|choice|grammar|structural_tag}` or OpenAI-style `response_format: {"type": "json_schema", ...}`. Backends: xgrammar, guidance, outlines, lm-format-enforcer; default `auto`; CLI `--structured-outputs-config.backend`. `tool_choice="required"` and named tools also go through structured outputs, and `strict: true` on a tool enforces its schema.
- Prefix caching is on by default in vLLM V1 and hashes image content into block hashes; v0.19.0 fixed "multimodal prefix cache key collisions". Put the system prompt and skill list first and the frame last so the cached prefix is stable.
- Ampere (3090) quantization support in vLLM: AWQ, GPTQ, Marlin (GPTQ/AWQ/FP8/FP4 weight-only), INT8 W8A8, bitsandbytes, GGUF. Not supported: llm-compressor FP8 W8A8 activations, and NVFP4 checkpoints (Blackwell). Google's `gemma-4-31B-it-qat-w4a16-ct` is compressed-tensors W4A16 and loads through the Marlin path with no `--quantization` flag.
- Gemma 4's image token budget is `--mm-processor-kwargs '{"max_soft_tokens": N}'`, N in {70,140,280,560,1120}, default 280. Qwen3.x tokenizes 640×480 to roughly 300 tokens at native resolution (32 px per token after 2×2 merge) [INFERRED]; cap with `max_pixels`.

### Robotics-specific models (future context only)

- **SmolVLA** (450M, LeRobot) — arm manipulation from LeRobot datasets; the natural first VLA for the SO-101 track.
- **π0 / π0-FAST / π0.5** (openpi, PyTorch since 2025-09; π0.5 ~3B; code Apache 2.0, weights carry Gemma terms) — inference >8 GB, LoRA fine-tune >22.5 GB on a 4090 [VENDOR]. Fits one 3090.
- **GR00T N1.7** (2026-04-17, 3B, Cosmos-Reason2-2B backbone, Apache 2.0, Ampere supported) — humanoid and bimanual focus; wheeled base is not a target embodiment.
- **Gemini Robotics On-Device 2** (2026-07-30) — trusted testers only; not obtainable.
- **LeRobot** now lists Pi0, Pi0.5, GR00T N1.7, SmolVLA, XVLA, EO-1, MolmoAct2, WALL-OSS, EVO1 as policies and LeKiwi and EarthRover as robots. Nothing here replaces the 27B command parser for the first build; a VLA drives joints, it does not chat.

## Recommendation for this build

**Primary: `Qwen/Qwen3.8-27B`** (fall back to `Qwen/Qwen3.6-27B` if your vLLM stable does not load 3.8).

- Why: the rover's hard cases are `find(object)`, counting, and left/right, and Qwen leads detection by 17 points and counting by 13 (Roboflow). It publishes embodied-relevant rows (ERQA, RefSpatialBench, CountBench). Tool adherence is at parity on single calls. Q4 footprint is 1.1 GiB smaller than Gemma 4 31B, and at 8K context it used 16.6 GB vs 20.0 GB on a 4090 with F16 KV [MEASURED, llama.cpp], which is KV headroom for a 1k system prompt plus a frame.
- Quantization: on 48 GB you are not forced to int4. Practitioner options in order of preference for this latency-bound loop: (a) int4 AWQ/GPTQ or compressed-tensors W4A16 via Marlin, TP=2, for fastest decode; (b) Qwen's official FP8 checkpoint loaded weight-only through Marlin on Ampere (~27 GB weights across two GPUs, ~9 GB KV per GPU) for better fidelity at roughly half the decode speed [INFERRED]. Confirm a `Qwen3.8-27B` AWQ or FP8 repo exists before committing; otherwise quantize with llm-compressor.
- Serve: `vllm serve Qwen/Qwen3.8-27B<-quant> --tensor-parallel-size 2 --max-model-len 8192 --reasoning-parser qwen3 --default-chat-template-kwargs '{"enable_thinking": false}' --limit-mm-per-prompt '{"image": 1}' --gpu-memory-utilization 0.90`. Do not add `--enable-auto-tool-choice`; you are not using tool parsing.
- Sampling for the parser: temperature 0–0.2, `top_p` 1.0. Qwen's recommended 0.7/0.8 is for chat, not for a controller [INFERRED].

**Fallback: `google/gemma-4-31B-it-qat-w4a16-ct`**, TP=2, `--reasoning-parser gemma4`, `--mm-processor-kwargs '{"max_soft_tokens": 280}'` (560 for `find`). Official QAT, longest-tested vLLM path (docker `vllm/vllm-openai:gemma4`), strongest multi-step JSON in the 4090 test, and the image token budget is an explicit speed knob. Accept weaker counting and detection.

**Fast tier if you want sub-second decode:** Gemma 4 26B-A4B (3.8B active) or Qwen3.6-35B-A3B on one GPU, leaving the other 3090 for faster-whisper + Kokoro + Molmo2-8B. This is the version of the brief's "two more containers" that a 2026 practitioner would actually build.

**Prompt and schema strategy (exact):**

1. System prompt (~600–900 tokens): role, safety rules, the JSON world state format, the skill list with one-line descriptions and argument bounds, and the output schema pasted as text. vLLM's own Gemma 4 recipe states that under constrained decoding "the model does not see the schema or its field descriptions", so the schema must also be in the prompt.
2. Request: `response_format: {"type": "json_schema", "json_schema": {"name": "skill_call", "schema": ...}}` with a top-level object `{"skill": enum["drive","turn","stop","say","describe_scene","find","set_face"], "args": {...}, "say": string}`; use `oneOf` keyed on `skill` if you want per-skill argument objects (xgrammar supports enum/oneOf/anyOf; numeric `minimum`/`maximum` support is partial, so do not depend on it).
3. Pi: parse with pydantic; enforce `drive ≤1 m, ≤0.3 m/s`, `turn ≤180°, ≤60°/s`, TTL, and known-object list. On failure re-ask once with the validator's message appended; on second failure `stop()`. Keep the ESP32 limits as the last gate exactly as the brief says.
4. Skip native tools entirely for the first build. Revisit `tools` + `tool_choice="required"` only if you later need multi-call turns; vLLM then constrains the call array by schema as well.
5. Thinking stays off for command turns. For `describe_scene` you may allow a short budget (Qwen3.8 `reasoning_effort` low) if you find descriptions improve; measure the latency first.

**Latency re-estimate for the brief's voice turn.** With int4 on a single 4090 llama.cpp measured 45–49 tok/s decode for these two models; a 3090 has ~93% of the 4090's memory bandwidth and vLLM TP=2 Marlin single-stream lands in the same 35–50 tok/s band [INFERRED]. 30–60 JSON tokens therefore cost 0.7–1.5 s, not the brief's 1.5–2.5 s. A 640×480 frame is ~300 tokens; prefill plus vision encoder is well under 0.5 s [INFERRED]. Expect ~2–3 s end to end with Vosk and Piper on the Pi, ~1–1.5 s without the image.

## Numbers

| quantity | value | hardware/context | tag | source URL |
|---|---|---|---|---|
| Gemma 4 release; sizes | 2026-04-02; E2B, E4B, 26B-A4B, 31B (12B Unified added later) | Google blog / HF blog | VENDOR | https://blog.google/innovation-and-ai/technology/developers-tools/gemma-4/ |
| Gemma 4 license | Apache 2.0 (first Gemma under Apache) | — | VENDOR | https://blog.google/innovation-and-ai/technology/developers-tools/gemma-4/ |
| Gemma 4 31B VRAM | bf16 59.0 GB; QAT W4A16 19.8 GB | vLLM, Google QAT docs | VENDOR | https://markaicode.com/benchmarks/vllm-gemma-benchmark/ |
| Gemma 4 26B-A4B VRAM | bf16 ~52 GB; AWQ int4 ~15 GB | community checkpoint | VENDOR | https://markaicode.com/benchmarks/vllm-gemma-benchmark/ |
| Gemma 4 image token budgets | 70/140/280/560/1120; default 280 | `--mm-processor-kwargs max_soft_tokens` | VENDOR | https://raw.githubusercontent.com/vllm-project/recipes/main/Google/Gemma4.md |
| Gemma 4 31B MMMU Pro | 76.9% | model card | VENDOR | https://ai.google.dev/gemma/docs/core/model_card_4 |
| Gemma 4 31B τ2-bench | 86.4% (Gemma 3 27B 6.6%); Together lists 76.9% | vendor tables | VENDOR | https://www.labellerr.com/blog/gemma-4-open-weight-ai-model-overview/ |
| Gemma 4 knowledge cutoff | January 2025 | model card | VENDOR | https://ai.google.dev/gemma/docs/core/model_card_4 |
| Gemma 4 26B-A4B BFCL v4 | non-live 89.13%, live 63.80%, multi-turn 45.12% | UD-IQ4_XS 12.65 GiB, RTX 5070 Ti, llama.cpp, 2026-06-01 | MEASURED | https://huggingface.co/unsloth/gemma-4-26B-A4B-it-GGUF/discussions/42 |
| Gemma 4 26B-A4B decode | 131 tok/s | RTX PRO 6000 Blackwell, vLLM | MEASURED | https://markaicode.com/benchmarks/vllm-gemma-benchmark/ |
| Gemma 4 26B-A4B NVFP4 decode | 52 tok/s | DGX Spark GB10, vLLM v0.19 | MEASURED | https://markaicode.com/benchmarks/vllm-gemma-benchmark/ |
| Q4_K_M file sizes | Qwen3.8-27B 15.93 GiB; Qwen3.6-27B 15.66 GiB; Gemma 4 31B 17.07 GiB | llama.cpp b10453 | MEASURED | https://kingy.ai/blog/qwen3-8-27b-vs-qwen3-6-27b-vs-gemma-4-31b/ |
| Decode, 256 tokens | Qwen3.8 49.09; Qwen3.6 49.04; Gemma 4 31B 45.00 tok/s | RTX 4090, llama.cpp Q4_K_M, 2026-08-17 | MEASURED | https://kingy.ai/blog/qwen3-8-27b-vs-qwen3-6-27b-vs-gemma-4-31b/ |
| Prefill, 512-token prompt | ~2,975–3,001 tok/s all three | RTX 4090, llama.cpp | MEASURED | https://kingy.ai/blog/qwen3-8-27b-vs-qwen3-6-27b-vs-gemma-4-31b/ |
| Peak VRAM at 8K ctx | Qwen3.x 16,626 MiB; Gemma 4 19,962 MiB (F16 KV) | RTX 4090 | MEASURED | https://kingy.ai/blog/qwen3-8-27b-vs-qwen3-6-27b-vs-gemma-4-31b/ |
| Tool-call adherence | single 90/90 (Qwen3.8, Gemma 4), 79/90 (Qwen3.6); multi-step 28/30, 30/30, 27/30 | RTX 4090, llama.cpp | MEASURED | https://kingy.ai/blog/qwen3-8-27b-vs-qwen3-6-27b-vs-gemma-4-31b/ |
| Roboflow Vision Evals overall | Qwen3.8-27B 74.7% (#17/53); Qwen3.5-27B 70.8% (#22); Gemma 4 31B 67.0% (#30) | hosted APIs, 2026-09-05 | MEASURED | https://playground.roboflow.com/models/compare/gemma-4-31b-vs-qwen3-8-27b |
| Object detection / counting / OCR | Qwen3.8 65.7/64.9/92.2; Qwen3.5 50.5/67.6/84.7; Gemma 4 48.2/51.4/90.8 | Roboflow Vision Evals | MEASURED | https://playground.roboflow.com/models/compare/gemma-4-31b-vs-qwen3-5-27b |
| Per-sample latency | Qwen3.8-27B 18.0 s; Gemma 4 31B 28.8 s | Roboflow hosted, not local | MEASURED | https://playground.roboflow.com/models/compare/gemma-4-31b-vs-qwen3-8-27b |
| Qwen3.5-27B BFCL-v4 | 0.685, rank 5 of 18 | self-reported, 2026-09-07 | VENDOR | https://llm-stats.com/benchmarks/bfcl-v4 |
| Qwen3.5-27B grounding | RefCOCO 90.9; RefSpatialBench 67.7; MMMU Pro 75.0 | model card | VENDOR | https://huggingface.co/Qwen/Qwen3.5-27B |
| Qwen3.6-27B spatial | RefCOCO avg 92.5; CountBench 97.8; RefSpatialBench 70.0; ERQA 62.5 | model card | VENDOR | https://huggingface.co/Qwen/Qwen3.6-27B |
| Qwen3.8-27B vision | RealWorldQA 85.9; OSWorld-Verified 84.3; OmniDocBench 91.1 | model card | VENDOR | https://huggingface.co/Qwen/Qwen3.8-27B |
| Qwen3.6 vLLM floor | vLLM ≥0.19.0; SGLang ≥0.5.10 | model card | VENDOR | https://huggingface.co/Qwen/Qwen3.6-27B |
| Qwen3-VL coordinate system | relative 0–1000 (Qwen2.5-VL was absolute pixels) | cookbook | VENDOR | https://raw.githubusercontent.com/QwenLM/Qwen3-VL/main/cookbooks/2d_grounding.ipynb |
| Qwen3-VL sizes | 2B/4B/8B/32B dense; 30B-A3B, 235B-A22B MoE; 32B 2025-10-21; vLLM ≥0.11.0 | README | VENDOR | https://github.com/QwenLM/Qwen3-VL |
| Mistral Small 3.2 | ~55 GB bf16; IFEval 84.78%; infinite gen 1.29% (from 2.11%); vLLM ≥0.9.1 | model card | VENDOR | https://huggingface.co/mistralai/Mistral-Small-3.2-24B-Instruct-2506 |
| Mistral Small 4 | 119B MoE, 2026-03-16, Apache 2.0 | — | VENDOR | https://computertech.co/mistral-small-4-review/ |
| Llama 4 Scout Q4 | ~55 GB | community guides | VENDOR | https://insiderllm.com/guides/llama-4-guide-scout-maverick/ |
| Molmo 2 | 2025-12-11; 4B/8B/7B-O; points scaled by 1000 | Ai2 blog / HF | VENDOR | https://allenai.org/blog/molmo2 |
| GLM-4.6V-Flash | 9B, MIT, vLLM ≥0.12.0 | HF | VENDOR | https://huggingface.co/zai-org/GLM-4.6V-Flash |
| vLLM structured outputs API | `guided_*` removed in v0.12.0; `structured_outputs` / `response_format json_schema`; backends xgrammar, guidance, outlines, lm-format-enforcer | docs | VENDOR | https://raw.githubusercontent.com/vllm-project/vllm/main/docs/features/structured_outputs.md |
| vLLM Ampere quantization | AWQ, GPTQ, Marlin (GPTQ/AWQ/FP8/FP4), INT8 W8A8, bnb, GGUF supported; FP8 W8A8 not | docs | VENDOR | https://raw.githubusercontent.com/vllm-project/vllm/main/docs/features/quantization/README.md |
| vLLM v0.19.0 | full Gemma 4 support, gemma4 tool parser, transformers ≥5.5.0, multimodal prefix-cache collision fix | release notes | VENDOR | https://github.com/vllm-project/vllm/releases/tag/v0.19.0 |
| GR00T N1.7 | 2026-04-17; 3B; Cosmos-Reason2-2B backbone; Apache 2.0 | HF blog | VENDOR | https://huggingface.co/blog/nvidia/gr00t-n1-7 |
| openpi | π0.5 2025-09; inference >8 GB; LoRA FT >22.5 GB; full FT >70 GB | README | VENDOR | https://github.com/Physical-Intelligence/openpi |
| Gemini Robotics On-Device 2 | 2026-07-30; trusted testers only | model card | VENDOR | https://deepmind.google/models/model-cards/gemini-robotics-on-device-2/ |
| 2×3090 single-stream decode, 27B int4 TP=2 | 35–50 tok/s | scaled from 4090 llama.cpp; no published 3090 vLLM number found | INFERRED | https://markaicode.com/benchmarks/vllm-gemma-benchmark/ |
| 640×480 frame in Qwen3.x tokens | ~300 | 32 px/token after 2×2 merge | INFERRED | — |

## Corrections to the brief

1. **"Assume Gemma-class: don't rely on native tool-call parsing."** Outdated as a capability claim. Gemma 4 (2026-04) has native function calling with a vLLM parser and τ2-bench 86.4% vs Gemma 3 27B's 6.6% [VENDOR]; Qwen3.5-27B is rank 5 on BFCL-v4 [VENDOR]; both hit 90/90 single-tool calls in a local Q4 test [MEASURED]. Keep the design choice anyway: constrained JSON with `skill` as enum is stricter than tool parsing, and Gemma 4's tool format is non-JSON.
2. **"27B VLM" model identity.** In 2026 the class is Qwen3.8-27B or Gemma 4 31B, not Gemma 3 27B or Qwen2.5-VL-32B. Both are Apache 2.0; the Gemma-terms concern is gone.
3. **Decode "~20–30 tok/s".** Measured 45–49 tok/s on a 4090 at Q4_K_M single-stream; a 3090 pair via vLLM TP=2 should land 35–50 [INFERRED]. The 30–60 token JSON costs 0.7–1.5 s. Total voice turn is ~2–3 s, not 2.5–4.5 s.
4. **"int4" is a choice, not a requirement.** 48 GB fits a 27B at FP8 weight-only through Marlin on Ampere. int4 remains the right pick for this loop because decode is bandwidth-bound; FP8 is the quality fallback [INFERRED].
5. **Thinking modes are missing from the brief.** Qwen3.5+ think by default; Gemma 4 is configurable. Serve with `enable_thinking: false` or every command turn gains a reasoning trace.
6. **"validate with pydantic, reject and retry once."** Keep it, but move format guarantees into the grammar (`response_format json_schema`). pydantic then only catches bounds and unknown objects; retries become rare.
7. **"Enable prefix caching."** Already on by default in vLLM V1 and multimodal-aware; the actionable part is prompt ordering (system + skills first, frame last) and pinning `max_soft_tokens` / `max_pixels` so the frame's token count is constant.
8. **"one 640×480 frame."** Fine for both. Gemma 4: 280 tokens default, 560 for `find`. Qwen: ~300 tokens; set `max_pixels` so a phone still gets the same budget later.
9. **Coordinate conventions.** The brief's `find(object)` needs a documented output convention: Qwen `bbox_2d [x1,y1,x2,y2]` 0–1000, Gemma `box_2d [y1,x1,y2,x2]` 0–1000. The Pi validator must be model-aware or normalize.
10. **"Two more containers" exception.** Still the highest-value change, and more affordable than the brief assumes: a 26B-A4B or 35B-A3B MoE fits one 3090 at int4 and frees the second GPU entirely for STT/TTS/Molmo2.

## Alternatives considered and rejected

- **Gemma 3 27B-it**: two generations old, Gemma Terms of Use, no native tools (τ2 6.6%), BFCL rank 78 for the 12B. Rejected.
- **Qwen2.5-VL-32B / Qwen3-VL-32B**: strong grounders in 2025; Qwen states Qwen3.5 "outperforms Qwen3-VL", and Qwen3-VL-32B has no 2026 successor branding. Use Qwen3.8-27B instead; same coordinate format lineage.
- **Mistral Small 3.2 24B**: solid Apache-2.0 2025 model with a proven `mistral` tool parser and low infinite-generation rate; no published grounding/pointing, and Mistral folded its vision line into the 119B Small 4, which does not fit. Keep as a break-glass fallback only.
- **Mistral Small 4 (119B MoE), GLM-4.6V (106B-A12B), Llama 4 Scout (109B-A17B)**: none fit 48 GB at int4 with KV room; Llama's license is not Apache. Rejected.
- **InternVL3.5-38B / 30B-A3B**: Apache 2.0 and competent, but tokenizer/vLLM friction reports and no 2026 refresh (InternVL-U is a 4B generation model). Rejected.
- **GLM-4.6V-Flash 9B, MiniCPM-V 4.5 8B, Kimi-VL-A3B**: too small to be the sole command parser and describer for a talking robot; GLM-4.6V-Flash is the best of these if you ever want a MIT-licensed tiny VLM on the second GPU.
- **Molmo2-8B as the primary**: best pointer, but not a general instruction follower for a 7-skill controller. Keep as an optional `find` specialist.
- **Gemma 4 26B-A4B as primary**: fast and a strong function caller (89% BFCL non-live), but Google recommends int8 not W4A16 for the MoE, and it trails the 31B on vision. Right choice for the "fast tier", not the primary.
- **Native `tools` + `tool_choice="auto"`**: works on both finalists in vLLM, but adds a parser, a chat template, and a non-JSON format (Gemma 4) between the model and the Pi. Rejected for the first build.
- **Pure prompted JSON with pydantic retry (the brief's approach)**: works, but leaves nonexistent skills representable. Superseded by grammar-constrained JSON.

## Open questions

1. Which vLLM stable release first loads `Qwen/Qwen3.8-27B`? The card does not pin a version; Qwen3.6 needs ≥0.19.0. If 3.8 needs a nightly, start on Qwen3.6-27B.
2. Does an official or community int4 (AWQ/GPTQ/W4A16) or FP8 checkpoint of Qwen3.8-27B exist and load on Ampere Marlin? Unverified.
3. Qwen3.5/3.6/3.8 grounding coordinate format: inherited 0–1000 relative from Qwen3-VL is inferred, not documented on the cards. Check with a calibration image.
4. No published tokens/s for either finalist on 2× RTX 3090 under vLLM TP=2; the 35–50 tok/s figure is scaled from a 4090 llama.cpp run. Measure at build gate 1.
5. No 2026 measurement of nonexistent-tool hallucination or refusal rates under a robot-style ~1k-token system prompt for either model. The grammar removes the first risk; the second needs your own 50-utterance test.
6. Gemma 4 31B τ2-bench is quoted as 86.4% (Google table via Labellerr) and 76.9% (Together AI page). Together's 76.9% equals its MMMU Pro figure and may be a copy error.
7. Whether the Gemma 4 QAT W4A16 checkpoint also quantizes the ~550M-parameter vision tower, and its accuracy delta vs bf16, is not stated by Google.
8. Qwen3.x hybrid attention (GDN) has a known vLLM cudagraph/mamba-cache assertion requiring `--max-cudagraph-capture-size` tuning; confirm it does not bite at `--max-model-len 8192`.

## Sources

- Gemma 4: Byte for byte, the most capable open models — Google, 2026-04-02 — https://blog.google/innovation-and-ai/technology/developers-tools/gemma-4/
- Gemma 4 model card — Google AI for Developers, updated 2026-07-30 — https://ai.google.dev/gemma/docs/core/model_card_4
- Welcome Gemma 4: Frontier multimodal intelligence on device — Hugging Face blog, 2026-04 — https://huggingface.co/blog/gemma4
- google/gemma-4-31B-it model card — Hugging Face, 2026 — https://huggingface.co/google/gemma-4-31B-it
- google/gemma-4-31B-it-qat-w4a16-ct — Hugging Face, 2026 — https://huggingface.co/google/gemma-4-31B-it-qat-w4a16-ct
- Gemma 4 recipe — vLLM recipes repo, 2026 — https://raw.githubusercontent.com/vllm-project/recipes/main/Google/Gemma4.md
- gemma4_tool_parser API — vLLM docs, 2026 — https://docs.vllm.ai/en/latest/api/vllm/tool_parsers/gemma4_tool_parser/
- vLLM v0.19.0 release notes — GitHub, 2026-04 — https://github.com/vllm-project/vllm/releases/tag/v0.19.0
- Structured Outputs — vLLM docs (main) — https://raw.githubusercontent.com/vllm-project/vllm/main/docs/features/structured_outputs.md
- Tool Calling — vLLM docs (main) — https://raw.githubusercontent.com/vllm-project/vllm/main/docs/features/tool_calling.md
- Quantization hardware support — vLLM docs (main) — https://raw.githubusercontent.com/vllm-project/vllm/main/docs/features/quantization/README.md
- Prefix caching design — vLLM docs (main) — https://raw.githubusercontent.com/vllm-project/vllm/main/docs/design/prefix_caching.md
- Supported models (multimodal table) — vLLM docs (main) — https://raw.githubusercontent.com/vllm-project/vllm/main/docs/models/supported_models.md
- vLLM Gemma 4 Benchmark: RTX 3090 Deployment Guide — Markaicode, 2026 — https://markaicode.com/benchmarks/vllm-gemma-benchmark/
- Deploy Gemma 4 QAT on GPU Cloud — Spheron, 2026-06-05 — https://www.spheron.network/blog/deploy-gemma-4-qat-gpu-cloud/
- Gemma 4: What Computer Vision Engineers Actually Need to Know — Datature, 2026-04-08 — https://datature.io/blog/gemma-4-what-computer-vision-engineers-actually-need-to-know
- Google Gemma 4: A Technical Overview — Labellerr, 2026 — https://www.labellerr.com/blog/gemma-4-open-weight-ai-model-overview/
- Gemma 4 31B — Together AI model page, 2026-05-21 — https://www.together.ai/models/gemma-4-31b
- BFCL v4 run on gemma-4-26B-A4B UD-IQ4_XS — HF discussion, 2026-06-01 — https://huggingface.co/unsloth/gemma-4-26B-A4B-it-GGUF/discussions/42
- Qwen3.8 vs Qwen3.6 vs Gemma 4: 24GB GPU Test — kingy.ai, 2026-08-17 (refreshed 08-22) — https://kingy.ai/blog/qwen3-8-27b-vs-qwen3-6-27b-vs-gemma-4-31b/
- Gemma 4 31B vs Qwen3.8 27B: Vision Model Comparison — Roboflow Playground, 2026-09-05 — https://playground.roboflow.com/models/compare/gemma-4-31b-vs-qwen3-8-27b
- Gemma 4 31B vs Qwen3.5-27B: Vision Model Comparison — Roboflow Playground, 2026-09-05 — https://playground.roboflow.com/models/compare/gemma-4-31b-vs-qwen3-5-27b
- Gemma 4 31B vs Qwen3.5 27B: Inference Speed… — The Kaitchup, 2026-04-15 (paywalled) — https://kaitchup.substack.com/p/gemma-4-31b-vs-qwen35-27b-inference
- Qwen/Qwen3.5-27B model card — Hugging Face, 2026-02 — https://huggingface.co/Qwen/Qwen3.5-27B
- Qwen/Qwen3.6-27B model card — Hugging Face, 2026-04 — https://huggingface.co/Qwen/Qwen3.6-27B
- Qwen/Qwen3.8-27B model card — Hugging Face, 2026-08 — https://huggingface.co/Qwen/Qwen3.8-27B
- QwenLM/Qwen3.8 README (release timeline 3.5→3.8) — GitHub, 2026-08 — https://github.com/QwenLM/Qwen3.8
- QwenLM/Qwen3-VL README — GitHub, 2025-10 — https://github.com/QwenLM/Qwen3-VL
- Qwen3-VL 2D grounding cookbook — GitHub — https://raw.githubusercontent.com/QwenLM/Qwen3-VL/main/cookbooks/2d_grounding.ipynb
- Qwen3.5/3.6 recipe — vLLM recipes repo, 2026 — https://raw.githubusercontent.com/vllm-project/recipes/main/Qwen/Qwen3.5.md
- BFCL-V4 Leaderboard — llm-stats, 2026-09-07 — https://llm-stats.com/benchmarks/bfcl-v4
- Berkeley Function Calling Leaderboard V4 — Gorilla — https://gorilla.cs.berkeley.edu/leaderboard.html
- mistralai/Mistral-Small-3.2-24B-Instruct-2506 — Hugging Face, 2025-06 — https://huggingface.co/mistralai/Mistral-Small-3.2-24B-Instruct-2506
- Mistral Small 4 Review — ComputerTech, 2026-03 — https://computertech.co/mistral-small-4-review/
- InternVL3.5 Usage Guide — vLLM recipes, 2025 — https://docs.vllm.ai/projects/recipes/en/latest/InternVL/InternVL3_5.html
- OpenGVLab/InternVL3_5-38B vLLM compatibility discussion — Hugging Face, 2025 — https://huggingface.co/OpenGVLab/InternVL3_5-38B/discussions/2
- Molmo 2 announcement — Ai2, 2025-12-11 — https://allenai.org/blog/molmo2
- allenai/Molmo2-8B model card — Hugging Face — https://huggingface.co/allenai/Molmo2-8B
- zai-org/GLM-V README — GitHub, 2025-12 — https://github.com/zai-org/GLM-V
- zai-org/GLM-4.6V-Flash model card — Hugging Face, 2025-12 — https://huggingface.co/zai-org/GLM-4.6V-Flash
- moonshotai/Kimi-VL-A3B-Instruct — Hugging Face, 2025-04 — https://huggingface.co/moonshotai/Kimi-VL-A3B-Instruct
- openbmb/MiniCPM-V-4_5 and MiniCPM-V-4.6 — Hugging Face, 2025-08 / 2026-05 — https://huggingface.co/openbmb/MiniCPM-V-4_5 , https://huggingface.co/openbmb/MiniCPM-V-4.6
- Llama 4 Guide: Running Scout and Maverick Locally — InsiderLLM, 2026 — https://insiderllm.com/guides/llama-4-guide-scout-maverick/
- Physical-Intelligence/openpi README — GitHub, 2025-09 — https://github.com/Physical-Intelligence/openpi
- huggingface/lerobot README — GitHub, 2026 — https://github.com/huggingface/lerobot
- NVIDIA Isaac GR00T N1.7 — Hugging Face blog, 2026-04-17 — https://huggingface.co/blog/nvidia/gr00t-n1-7
- Gemini Robotics On-Device 2 model card — Google DeepMind, 2026-07-30 — https://deepmind.google/models/model-cards/gemini-robotics-on-device-2/
- The Spatial Blindspot of Vision-Language Models — arXiv, 2026-01 — https://arxiv.org/pdf/2601.09954
- OmniSpatial benchmark — arXiv, 2025 — https://arxiv.org/html/2506.03135v2

## Verification (adversarial review)

Reviewed 2026-09-07. Method: every claim re-checked against primary or independent pages fetched directly (Hugging Face API/config JSON, Google model card and blog, HF Gemma 4 blog, vLLM release notes/PRs/docs, PyPI, Roboflow, Wikipedia GPU specs). Web search was unavailable for this pass, so "independent" means a source other than the one the note cites, or the primary artifact itself (checkpoint files, config.json).

| claim | verdict | evidence | source URL | corrected claim |
|---|---|---|---|---|
| Qwen3.8-27B: 27B dense, native vision, Apache 2.0, released 2026-08-14, `Qwen3.5ForConditionalGeneration` family | confirmed | HF API: `license: apache-2.0`, `pipeline_tag: image-text-to-text`, `architectures: Qwen3_5ForConditionalGeneration`, `model_type: qwen3_5`, 27.78B BF16 params, repo created 2026-08-05, lastModified 2026-08-14; QwenLM/Qwen3.8 News: "2026-08-14 Qwen3.8-27B released". config.json: 64 layers, 48 linear-attention + 16 full-attention, same layout as Qwen3.5-27B. | https://huggingface.co/api/models/Qwen/Qwen3.8-27B ; https://github.com/QwenLM/Qwen3.8 | — (also: official `Qwen/Qwen3.8-27B-FP8` exists, created 2026-08-13; community AWQ-INT4/GPTQ-W4A16/AutoRound W4A16 repos exist, e.g. `cyankiwi/Qwen3.8-27B-AWQ-INT4`, `RedHatAI/Qwen3.8-27B-INT4`, `amd/Qwen3.8-27B-Quark-AWQ-INT4-W4A16` — resolves Open Question 2 as "yes, exists; loading on Ampere still untested") |
| Gemma 4 31B released 2026-04-02, Apache 2.0, official QAT W4A16 checkpoint "19.8 GB vs 59.0 GB bf16" | refuted (size); release/license confirmed | Google blog dated 2026-04-02 and HF blog confirm launch and Apache 2.0. But `google/gemma-4-31B-it-qat-w4a16-ct` was created 2026-06-04 (two months after launch) and its single `model.safetensors` is 23,265,352,448 bytes = 23.27 GB (21.65 GiB): 14.65 GB packed int4 + 8.62 GB of bf16 tensors (4.31B params kept bf16). The 19.8 GB figure (markaicode citing "Google QAT docs") is not reproducible from the checkpoint; the HF card gives no GB number. | https://huggingface.co/api/models/google/gemma-4-31B-it-qat-w4a16-ct/tree/main ; https://blog.google/innovation-and-ai/technology/developers-tools/gemma-4/ | QAT W4A16 checkpoint is 23.3 GB on disk (expect ≥23 GB weight VRAM). Fits 2×3090 at TP=2 (~11.7 GB/GPU) but NOT one 24 GB card with KV. Checkpoint shipped 2026-06-04, not at the 2026-04-02 launch. |
| Gemma 4 31B τ2-bench 86.4% vs Gemma 3 27B 6.6% | refuted | Google model card row: "Tau2 (average over 3) 76.9% (31B), 68.2% (26B A4B), 69.0% (12B), 42.2% (E4B), 24.5% (E2B), 16.2% (Gemma 3 27B)". HF Gemma 4 blog: "Tau2 76.9% (31B) vs 16.2% (Gemma-3 27B)". Together AI page: 76.9%. The note's Open Question 6 guessed Together's 76.9% was a copy of MMMU Pro; both metrics really are 76.9% and Labellerr's 86.4% is the outlier. | https://ai.google.dev/gemma/docs/core/model_card_4 ; https://huggingface.co/blog/gemma4 | Gemma 4 31B τ2-bench 76.9% (avg of 3 domains); Gemma 3 27B 16.2%. Qwen3.5-27B self-reports TAU2-Bench 79.0, i.e. above Gemma 4 31B. |
| Roboflow Vision Evals: Qwen3.8-27B 74.7% (#17/53) vs Gemma 4 31B 67.0% (#30/53); detection 65.7 vs 48.2; counting 64.9 vs 51.4; OCR 92.2 vs 90.8; latency 18.0 s vs 28.8 s | needs_qualifier | Compare page re-fetched: identical numbers, "Last Updated September 5, 2026". Omitted conditions: scores are the "Low Effort" reasoning setting; counting is 64.9 ±4.1 vs 51.4 ±1.4; Gemma leads Data Extraction 80.4 vs 78.0; measured on hosted APIs (bf16/FP8 provider quant, not local int4). Per-model pages return 404, so only compare pages are checkable. | https://playground.roboflow.com/models/compare/gemma-4-31b-vs-qwen3-8-27b | Same numbers, but at low reasoning effort on hosted APIs; counting margin is 13.5 ±4.3 points; Gemma wins Data Extraction 80.4 vs 78.0. |
| Decode 49.09 / 49.04 / 45.00 tok/s (Qwen3.8 / Qwen3.6 / Gemma 4 31B), Q4_K_M sizes 15.93/15.66/17.07 GiB, peak 8K-ctx VRAM 16,626 / 19,962 MiB, tool calls 90/90, 79/90, 90/90 and 28/30, 27/30, 30/30 | needs_qualifier | kingy.ai page re-fetched: all numbers match (test date Aug 16 2026; RTX 4090 450 W; llama.cpp b10453; F16 KV; batch 2048; 5 repetitions; tool tests 30 runs × 3 seeds, temperature 0). Author's own caveat: "One GPU, one llama.cpp commit, one GGUF publisher and a synthetic suite—not a claim about every runtime or workload." No second measurement found anywhere. | https://kingy.ai/blog/qwen3-8-27b-vs-qwen3-6-27b-vs-gemma-4-31b/ | Single-source, llama.cpp single-stream numbers; tool-call tests used tools-in-prompt on llama.cpp, not vLLM structured outputs. Treat as indicative, not as a vLLM figure. |
| 2×3090 vLLM TP=2 int4 single-stream decode 35–50 tok/s (3090 has ~93% of 4090 bandwidth) | unverifiable | Bandwidth ratio confirmed: RTX 3090 936 GB/s, RTX 4090 1008 GB/s (92.9%). markaicode (2026-07-04) states outright: "No verifiable published number exists" for vLLM Gemma 4 throughput on an RTX 3090. TP=2 adds a per-layer all-reduce over PCIe unless the 3090s are NVLink-bridged (3090 supports 2-way NVLink). No 3090 vLLM measurement found for either model. | https://en.wikipedia.org/wiki/GeForce_30_series ; https://en.wikipedia.org/wiki/GeForce_40_series ; https://markaicode.com/benchmarks/vllm-gemma-benchmark/ | Bandwidth scaling gives ~42–46 tok/s for a single 3090; TP=2 interconnect overhead unknown; NVLink bridge presence matters. Measure at gate 1 before quoting a latency. |
| vLLM ≥0.19.0 is the serving floor: full Gemma 4 support, `gemma4` tool parser, transformers ≥5.5.0, multimodal prefix-cache collision fix | needs_qualifier | v0.19.0 notes: "Full Google Gemma 4 architecture support ... (#38826, #38847)", "Requires transformers>=5.5.0", "Gemma 4 tool parser (#38847)", "multimodal prefix cache key collisions (#36708)"; PyPI upload 2026-04-03; Qwen3.6-27B card: vLLM ≥0.19.0. But PyPI latest is 0.28.0, and `Qwen/Qwen3.8-27B` config.json declares `transformers_version: 5.8.0.dev0`, so a 0.19-era stack likely will not load 3.8 cleanly. Qwen3.8 README serve command: `--reasoning-parser qwen3 --enable-auto-tool-choice --tool-call-parser qwen3_coder`, no version stated. | https://pypi.org/pypi/vllm/json ; https://github.com/vllm-project/vllm/releases/tag/v0.19.0 ; https://huggingface.co/Qwen/Qwen3.8-27B/raw/main/config.json | 0.19.0 (2026-04-03) is the floor for Gemma 4 and Qwen3.6; for Qwen3.8 use a current release (0.28.0 as of 2026-09) because its config targets transformers 5.8. Architecture class is unchanged from Qwen3.5, so loader support is inherited; config/template pins are the risk. |
| vLLM removed `guided_json/guided_regex/guided_choice/guided_grammar` in v0.12.0; use `structured_outputs` or `response_format json_schema`; backends xgrammar/guidance/outlines/lm-format-enforcer, default `auto`, CLI `--structured-outputs-config.backend` | confirmed | PR #29326 "Scheduled removal of guided_* config fields" merged 2025-11-25, description: "scheduled for removal in v0.12.0, which will be the next release"; v0.12.0 release (2025-12-03) lists it; docs map each `guided_*` field to `structured_outputs` and say default backend `auto`, flag `--structured-outputs-config.backend`. Not verified: the note's "xgrammar numeric minimum/maximum partial" claim (docs page did not state it). Docs add: structured outputs can be disabled while reasoning is on unless `--structured-outputs-config.enable_in_reasoning=True`. | https://github.com/vllm-project/vllm/pull/29326 ; https://raw.githubusercontent.com/vllm-project/vllm/main/docs/features/structured_outputs.md | — (add: if `describe_scene` ever runs with thinking on, set `enable_in_reasoning=True` or the schema is not enforced) |
| Gemma 4 image token budget 70/140/280/560/1120, default 280, via `--mm-processor-kwargs '{"max_soft_tokens": N}'` | confirmed | Google model card: "The supported token budgets are: 70, 140, 280, 560, and 1120." HF blog: same five values. vLLM Gemma 4 recipe: "70, 140, 280 (default), 560, 1120 tokens per image", flag `--mm-processor-kwargs '{"max_soft_tokens": N}'`. | https://ai.google.dev/gemma/docs/core/model_card_4 ; https://huggingface.co/blog/gemma4 | — |
| Gemma 4 tool format is custom non-JSON `<\|tool_call>call:func{key:<\|"\|>value<\|"\|>}<tool_call\|>`; Gemma emits `{"box_2d": [y1,x1,y2,x2], "label"}` in 0–1000 space unprompted; Qwen3.5+ coordinate format undocumented | confirmed | ai.google.dev function-calling doc: `<\|tool_call>call:get_current_weather{location:<\|"\|>Tokyo, JP<\|"\|>}<tool_call\|>`, "custom serialization (not JSON)". HF blog: `{"box_2d": [171, 75, 245, 308], "label": ...}` "normalized to 1000×1000 ... without explicit instruction". Datature: "coordinate format is [y1, x1, y2, x2]"; Gemini API docs: "[ymin, xmin, ymax, xmax] normalized to 0-1000". Qwen3.5/3.6/3.8 cards: no coordinate statement (confirmed absent). | https://ai.google.dev/gemma/docs/capabilities/function-calling ; https://huggingface.co/blog/gemma4 ; https://ai.google.dev/gemini-api/docs/image-understanding | — (Qwen side stays INFERRED, as the note says) |
| Qwen3.5-27B BFCL-v4 0.685, rank 5 | needs_qualifier | Qwen3.5-27B card: "BFCL-V4: 68.5" (self-reported). llm-stats board: 18 models, "Self-reported (18 models; 0 verified)", Qwen3.5-27B 0.685 at #5, updated 2026-09-07; Gemma 4, Qwen3.6, Qwen3.8 absent. Official Gorilla leaderboard is JS-rendered and could not be read; the rank is not from it. | https://huggingface.co/Qwen/Qwen3.5-27B ; https://llm-stats.com/benchmarks/bfcl-v4 | 68.5 self-reported by Qwen; "rank 5" is among 18 self-reported entries on llm-stats, not on the official BFCL leaderboard. |
| Qwen3.5/3.6/3.8 think by default; disable with `chat_template_kwargs: {"enable_thinking": false}`; Qwen3.8 adds `reasoning_effort` | confirmed | Qwen3.5-27B card: thinking "Enabled by default", disable via `"chat_template_kwargs": {"enable_thinking": False}`. Qwen3.8-27B card: "Thinking mode is on by default and can be disabled per request"; `reasoning_effort` levels `xhigh` (default), `medium`, `low`. Qwen3.6-27B card: thinking mode by default. Non-thinking recommended sampling (3.8): temperature 0.7, top_p 0.80, top_k 20, presence_penalty 1.5. | https://huggingface.co/Qwen/Qwen3.5-27B ; https://huggingface.co/Qwen/Qwen3.8-27B | — (add: default `reasoning_effort` is `xhigh`, so a forgotten flag costs more than on 3.5/3.6; Qwen recommends presence_penalty 1.5 in non-thinking mode) |
| Gemma 4 26B-A4B: no official W4A16 QAT; Google says `--quantization int8_per_channel_weight_only`; community AWQ int4 ~15 GB fits one 3090 | confirmed (but see contradiction below) | HF google org listing: 26B-A4B has `qat-q4_0-gguf` and `qat-q4_0-unquantized` only; `qat-w4a16-ct` exists for 31B, 12B, E4B, E2B, not 26B-A4B. vLLM recipe: "its small expert dimensions (704) cause excessive quality loss with 4-bit quantization ... deploy with `--quantization int8_per_channel_weight_only` (~47% memory reduction)". markaicode: int8 ≈ 28 GB, AWQ int4 ≈ 15 GB (community estimate). | https://huggingface.co/api/models?author=google&search=gemma-4&limit=100 ; https://raw.githubusercontent.com/vllm-project/recipes/main/Google/Gemma4.md | — |
| 640×480 frame ≈ 300 Qwen3.x tokens (32 px/token after 2×2 merge) | confirmed | Qwen3.8-27B config.json: `vision_config.patch_size: 16`, `spatial_merge_size: 2` → 32 px per token; preprocessor: `patch_size 16`, `merge_size 2`, `shortest_edge 65536` px, `longest_edge 16777216` px, so 640×480 (307,200 px) is neither up- nor down-scaled → 20×15 = 300 tokens plus vision start/end tokens. | https://huggingface.co/Qwen/Qwen3.8-27B/raw/main/config.json ; https://huggingface.co/Qwen/Qwen3.8-27B/raw/main/preprocessor_config.json | — |

### Stale or missing

- **Wrong τ2-bench numbers (Summary, Corrections 1, Alternatives, Open Q6).** 86.4% and 6.6% are not Google's figures; the model card says 76.9% (Gemma 4 31B) and 16.2% (Gemma 3 27B). Open Question 6 should close the other way: Together and the model card agree at 76.9%; Labellerr is the copy error. Drop Labellerr as a source.
- **Qwen3.5-27B's own TAU2-Bench is 79.0**, above Gemma 4 31B's 76.9. The note frames Gemma 4 as the function-calling leader; on the vendors' own τ2 numbers the primary pick already leads.
- **QAT W4A16 size.** The 31B checkpoint is 23.27 GB on disk, not 19.8 GB, and shipped 2026-06-04, not at launch. It fits 2×3090 at TP=2 but not a single 24 GB card with KV. markaicode's "tight at 19.8 GB" single-3090 statement is wrong; the note's fallback (TP=2) survives, but any single-GPU Gemma 4 31B plan does not.
- **Fast tier silently contradicts the note's own rejection.** "Gemma 4 26B-A4B on one GPU" requires 4-bit (int8 is ~28 GB > 24 GB), which the vLLM/Google recipe says causes "excessive quality loss" for the 704-dim experts; the Alternatives section already cites that recommendation. Either the fast tier needs both GPUs for int8 (which kills the "frees the second GPU" argument) or it must use Qwen3.6-35B-A3B only. It also presumes the brief's "negotiable, not assumed" second-container permission.
- **vLLM version is stale.** PyPI latest is 0.28.0; "≥0.19" is April 2026's floor. `Qwen/Qwen3.8-27B` config pins `transformers_version 5.8.0.dev0`, so a 0.19-era transformers 5.5 stack is unlikely to load 3.8. Open Question 1 narrows to: same architecture class and layer layout as Qwen3.5-27B, so support is inherited; verify config/template pins on a current vLLM.
- **Open Question 2 is answered.** Official `Qwen/Qwen3.8-27B-FP8` (2026-08-13) exists; community AWQ-INT4, GPTQ-W4A16, AutoRound and `RedHatAI/Qwen3.8-27B-INT4` exist. Still unverified: whether Qwen's block-wise FP8 loads weight-only via Marlin on Ampere (vLLM fp8 doc URL 404 during review).
- **Speculative decoding / MTP omitted.** The vLLM Qwen3.5 recipe recommends "MTP-1 speculative decoding to reduce per-token latency in low-concurrency scenarios", and many Qwen3.8-27B quant repos carry MTP heads. For a single-stream 30–60-token JSON reply this is the largest 2026 decode lever and the note does not mention it.
- **No 3090 TP=2 measurement exists** (markaicode says so explicitly). The note should also state whether the two 3090s are NVLink-bridged; TP=2 over PCIe changes the single-stream number.
- **Structured outputs while reasoning.** vLLM docs: structured outputs may be disabled when reasoning is enabled unless `--structured-outputs-config.enable_in_reasoning=True`. The note's item 5 (allow a thinking budget for `describe_scene`) needs this flag or the schema is not enforced on those turns.
- **Roboflow qualifiers missing.** Scores are at "Low Effort" reasoning on hosted APIs; counting is ±4.1; Gemma leads Data Extraction (80.4 vs 78.0). Local int4 numbers will differ.
- **BFCL "rank 5"** is from llm-stats' 18-model self-reported board (0 verified), not the official Gorilla leaderboard.
- **Kingy tool-call results** were llama.cpp tools-in-prompt at temperature 0, 30 runs × 3 seeds, one GGUF publisher; not a vLLM structured-output test. Cite as indicative only.
- **Qwen3.8 sampling.** Qwen's non-thinking recommendation includes presence_penalty 1.5; the note's temperature 0–0.2 is reasonable for a parser but should be paired with a repetition check on `say` strings.
- **Date typo.** Kingy test date is 2026-08-16 (note: 08-17). vLLM v0.12.0 released 2025-12-03 (not "2024" as one release-page render shows).
- **Brief vs note.** The note explicitly corrects the brief's decode rate and turn time; no silent contradictions found on skills, bounds, validator chain, or Pi stack. The one implicit tension is above: the fast tier assumes the box hosts extra containers, which the brief lists as negotiable, not assumed.
