# Moving STT and TTS onto the GPU box (2× RTX 3090, next to a 27B VLM)

Research note, 2026-09-07. Method: the session's WebSearch budget was exhausted (200/200) before this task, so every fact comes from ~55 direct WebFetch reads of primary pages (READMEs, release feeds, model cards, vLLM source, vendor docs). Numbers with no measurement on named hardware are tagged [INFERRED] with the derivation.

## Summary

- The move is worth doing, but for different reasons than the brief gives. The Pi→box change buys **accuracy** (Vosk small 9.85 % WER on LibriSpeech clean vs Whisper-class 1.5–2 %) and **Pi CPU headroom** first; the wall-clock saving is **~0.5–1.0 s per turn, not 1–1.5 s**. STT latency barely moves because the silence timer, not compute, dominates once STT is fast. TTS is where the time comes from (Piper medium on a Pi 4 runs at RTF ≈ 0.6, Kokoro on a GPU gives ~0.3 s to first audio).
- VRAM is not a constraint. Gemma-3-27B int4 with tensor parallel 2 needs ~12–14 GiB per 3090 including KV cache for 4 × 8k-token sequences, because vLLM's hybrid KV manager stores only 1024 tokens for the 52 sliding-window layers. Set `--gpu-memory-utilization 0.65` (per-instance limit, documented as such) and ~8 GiB per card stays free. Whisper large-v3-turbo fp16 (~2–3 GB) plus Kokoro-FastAPI (2.4–3.1 GB measured) fit on one card with room left. No MPS.
- The brief's "<1 GB TTS" is wrong for the mainstream container: Kokoro-FastAPI measures a 2.37 GB floor (CUDA context + PyTorch) and 3.11 GB loaded on a 4060 Ti. Under 1 GB is only true for ONNX-runtime Kokoro or for Piper on the box CPU.
- Recommended stack: `rhasspy/wyoming-faster-whisper` 3.7.0 GPU image (CUDA 12.8) with `Systran/faster-whisper-large-v3-turbo`-class weights, fp16, and `remsky/kokoro-fastapi-gpu` 0.8.2 streaming PCM over `/v1/audio/speech`. Zero-VRAM alternative for English: Parakeet-TDT-0.6B-v2 through `--stt-library onnx-asr` on the box CPU (RTFx 36 on a Ryzen 9800X3D → ~85 ms for a 3 s clip).
- Wake word and VAD/endpointing stay on the Pi. Every 2026 satellite design (wyoming-satellite, its successor linux-voice-assistant) does exactly this. Stream 16 kHz PCM over Wyoming TCP from wake until endpoint; the upload finishes the instant the endpoint fires, so transport adds ~10–50 ms on LAN Wi-Fi. Opus is unnecessary at 256 kbit/s.

## State of the art (2026)

### STT servers

**faster-whisper / CTranslate2.** Current release 1.2.1 (2025-10-31, Silero VAD v6). 1.1.0 (2024-11-21) added `large-v3-turbo` and a batched pipeline "4x faster"; 1.2.0 (2025-08-06) added `distil-large-v3.5`. Needs CUDA 12 + cuDNN 9. README benchmark: large-v2 fp16, beam 5, 13 min audio on an RTX 3070 Ti = 63 s, 4525 MB VRAM; int8 = 59 s, 2926 MB. Turbo cuts the decoder from 32 to 4 layers (809M params) at Open-ASR-Leaderboard mean WER 7.83; distil-large-v3.5 (756M, English-only, MIT) is ~1.5× faster than turbo with short-form WER 7.08. `transcribe()` exposes `hotwords`, `initial_prompt`, `vad_filter`, `without_timestamps`, `beam_size` (default 5); for commands use `beam_size=1`, `without_timestamps=True`, and skill/object names in `hotwords`.

**Servers wrapping it.**
- `rhasspy/wyoming-faster-whisper` 3.7.0 (2026-09-01). Now a multi-backend server: `--stt-library` = faster-whisper | transformers | sherpa | onnx-asr | qwen3-asr | funasr; `--device cuda` for all backends except sherpa (CPU even in the GPU image); GPU image is CUDA 12.8 and ~10.7 GB; `WYO_WHISPER_*` env config; Home Assistant name biasing. Processes the utterance after `audio-stop` (not streaming). Active: 9 releases Jan–Sep 2026.
- `speaches-ai/speaches` (ex faster-whisper-server): STT + Kokoro/Piper TTS + OpenAI `/v1/audio/transcriptions`, `/v1/audio/speech`, SSE streaming transcription, Realtime API (WebSocket/WebRTC), dynamic model load/unload with per-task TTL, Parakeet support (v0.9.0-rc.1, 2025-09-25). Images `ghcr.io/speaches-ai/speaches:latest-cuda|latest-cpu`. Last release v0.9.0-rc.3 on 2025-12-27; no release in the nine months since. Check maintenance before betting on it.
- `whisper.cpp` `whisper-server`: "OAI-like" HTTP API, `-DGGML_CUDA=1`, `--vad` with Silero. Turnkey tooling above is easier; whisper.cpp suits Pi-local Whisper, not the box.

**NVIDIA Parakeet TDT 0.6B.** v2 (2025-05-01, English, CC-BY-4.0): Open ASR mean WER 6.05, LibriSpeech clean 1.69, RTFx 3386 on A100. v3 (2025-08-14, 25 languages): mean WER 6.34, RTFx 3333. Both beat whisper-large-v3-turbo (7.83) on the leaderboard. Runs without NeMo via ONNX: `istupakov/onnx-asr` reports RTFx 36 on a Ryzen 9800X3D CPU, 57 on a T4 (CUDA), 320 on an RTX 5070 Ti (TensorRT), 1.0 on a Cortex-A53; sherpa-onnx int8 export is 622 MB encoder + 12 MB decoder + 6 MB joiner and runs RTF 0.088 at 4 threads on an RK3588 (Cortex-A76). Offline only; no hotword biasing in the ONNX path.

**Streaming models (new in 2026).** `nvidia/nemotron-speech-streaming-en-0.6b` (2026-01-05, updated 2026-03-13; NVIDIA Open Model License): cache-aware streaming with selectable 0.08/0.16/0.56/1.12 s latency, mean WER 6.93 at 1.12 s. RealtimeSTT now recommends `sherpa-onnx-nemotron-3.5-asr-streaming-0.6b-560ms-int8` for live text plus `parakeet-tdt-0.6b-v3-int8` for the final pass, on CPU. Kyutai `stt-1b-en_fr` (CC-BY-4.0) streams with a fixed 0.5 s delay and a **semantic VAD** that predicts end-of-turn; served by `moshi-server` (Rust) over WebSocket at 24 kHz; 64 streams at 3× real time on an L40S. Voxtral Realtime 4B (2026-02-04, Apache 2.0) claims sub-200 ms configurable delay. Qwen3-ASR 0.6B/1.7B (2026-01-29, Apache 2.0) serves through vLLM with streaming; the 0.6B (actually 0.9B params) scores LibriSpeech 2.11/4.55 vs whisper-large-v3 1.51/3.97.

### TTS servers

- **Kokoro-82M** v1.0 (2025-01-27, Apache 2.0, 24 kHz, 8 languages/54 voices). **Kokoro-FastAPI** 0.8.2 (2026-09-05): `/v1/audio/speech` with `stream`, formats mp3/wav/opus/flac/aac/pcm; "~300 ms (GPU) @ 400" first-token latency and "35x–100x realtime, 137.67 tok/s" on an RTX 4060 Ti 16 GB (CUDA 12.1); "~3500 ms (CPU) @ 200 (older i7)"; VRAM 2.37 GB floor, 3.11 GB loaded. Images `ghcr.io/remsky/kokoro-fastapi-gpu:latest` (cu126) and `:latest-cu128`. 0.8.1 added lazy phonemization for faster streaming.
- **Piper** moved to `OHF-Voice/piper1-gpl` (GPL-3, `pip install piper-tts`); `synthesize()` yields per-sentence chunks, `use_cuda=True` with onnxruntime-gpu. `wyoming-piper` 2.4.3 (2026-09-03) adds a GPU image and an experimental OmniVoice backend (CPU-heavy, desktop only). Home Assistant docs: on a Raspberry Pi, medium voices "generate 1.6 s of voice in a second" (RTF ≈ 0.62).
- **Orpheus 3B** (Apache 2.0): "~200 ms streaming latency", but a 3B Llama on vLLM (6+ GB, second engine). **Chatterbox** (MIT; 500M/350M/110M): no streaming server. **Qwen3-TTS** 0.6B/1.7B (2026-01-22, Apache 2.0): 97 ms latency claim, vLLM-Omni, no OpenAI endpoint yet. **Fish S2-Pro** 4B: research license. Dia and F5-TTS: non-streaming, not re-verified this session.
- OpenAI `/v1/audio/speech` compatible today: Kokoro-FastAPI, Speaches (Kokoro/Piper), third-party wrappers for Orpheus. Wyoming `synthesize` streaming exists since wyoming 1.7.0 (2025-06-23; current 1.10.2, 2026-08-27).

### Client-side pieces (Pi)

openWakeWord v0.6.0: "a single core of a Raspberry Pi 3 can run 15–20 models simultaneously"; 80 ms frames; optional Silero `vad_threshold` and Speex noise suppression. Silero VAD: <1 ms per 30 ms chunk on one CPU thread (x86), ~2 MB model, MIT. wyoming-satellite (archived) and linux-voice-assistant (ESPHome protocol, Pi Zero 2 W up) both run wake word and VAD locally and stream audio only after wake. Wyoming itself is JSONL headers + binary PCM (16 kHz, 16-bit, mono) over TCP, no auth, `audio-start/audio-chunk/audio-stop` → `transcript`.

## Recommendation for this build

1. **vLLM sizing.** Weights: `gaunernst/gemma-3-27b-it-int4-awq` (Google's QAT int4 in AWQ layout) is 18.5 GB on disk → ~9.3 GB per GPU at TP=2 [INFERRED]. KV: 16 KV heads × 128 dim × 2 (K,V) × 2 B = 8 KiB per token per layer. vLLM's hybrid KV cache manager (documented with Gemma-3-27b as the example: 52 sliding + 10 full layers) frees blocks outside the 1024-token window, so KV per sequence = 10 × 8 KiB × tokens + 52 × 1024 × 8 KiB = 80 KiB/token + 416 MiB. Four concurrent 8k sequences ≈ 4.1 GiB total, ~2 GiB per GPU [INFERRED, arithmetic from config.json]. Add ~1–2 GiB activations + CUDA graphs [INFERRED]. Total ≈ 12.5–13.5 GiB per card. Set `--gpu-memory-utilization 0.65` (15.6 GiB) or pin `--kv-cache-memory-bytes 3000000000`; both flags exist in current vLLM and the utilization flag is documented as "a per-instance limit". Confirm on the startup log line that reports KV cache size and the free-memory breakdown. Start vLLM **before** the speech containers: older vLLM versions raised when free memory at startup was below utilization × total; current main logs a breakdown instead, but start order is free insurance.
2. **STT container.** `rhasspy/wyoming-faster-whisper:3.7.0` GPU image, `--device cuda`, fp16, model = faster-whisper's `large-v3-turbo` alias (multilingual, 4-layer decoder) or `distil-whisper/distil-large-v3.5-ct2` if English-only is acceptable. Pin to GPU 1 with `NVIDIA_VISIBLE_DEVICES=1`. Expected budget for a 3 s command: mel + 30 s-window encoder + ~15 decoded tokens ≈ 0.15–0.35 s on a 3090 [INFERRED from README RTF and the 32→4 decoder reduction; measure at gate 3]. VRAM 2–3 GB [INFERRED from the README's 4.5 GB for large-v2 fp16 beam 5]. Alternative with zero VRAM: `--stt-library onnx-asr` with `parakeet-tdt-0.6b-v2` int8 on the box CPU (~85 ms per 3 s clip on a 9800X3D-class CPU, [MEASURED] RTFx 36; a 4-core older desktop will be 2–4× slower [INFERRED]). Choose Whisper if you want `hotwords` biasing for skill vocabulary; choose Parakeet if the box CPU is decent and you want the 6.05-vs-7.83 WER edge with no GPU footprint.
3. **TTS container.** `ghcr.io/remsky/kokoro-fastapi-gpu:latest` (0.8.2), same GPU, request `response_format=pcm`, `stream=true`, 24 kHz, play chunks through ALSA on the Pi. Budget: ~0.3 s to first audio [VENDOR, 4060 Ti] plus LAN. Keep pre-rendered filler WAVs on the Pi so the "speak a filler immediately" trick costs 0 ms regardless of TTS location. If the 2.4–3.1 GB bothers you, run `wyoming-piper` 2.4.3 on the box CPU instead: medium voice, sub-0.2 s first sentence on a desktop core [INFERRED from Pi RTF 0.62 and ~5–10× core speed], zero VRAM, lower voice quality.
4. **Transport.** Pi: openWakeWord (80 ms frames) → on detection open a Wyoming TCP connection, send `audio-start`, stream 16 kHz PCM `audio-chunk`s live, run Silero VAD locally, send `audio-stop` after 600–800 ms of trailing silence. Because audio streams while the user talks, upload completes with the endpoint; transport cost is one LAN round trip plus the last chunk. TTS: HTTP POST to Kokoro-FastAPI. Both are plain TCP on a trusted LAN. Disable Pi Wi-Fi power save (`iw dev wlan0 set power_save off`); power-save wake-ups cause 100–300 ms spikes [INFERRED, common practice].
5. **Wake word and VAD stay on the Pi.** Streaming the mic 24/7 puts Wi-Fi in the always-on path; every 2026 satellite (wyoming-satellite, linux-voice-assistant) keeps wake + VAD local. Later: Kyutai semantic VAD or Nemotron streaming on the box can replace the fixed silence timer and recover 0.3–0.5 s.
6. **Co-tenancy.** Plain process co-tenancy; the driver time-slices. MPS would overlap kernels but adds a daemon and touches vLLM's NCCL setup. Expect the small models to run 1.5–2× slower while the 27B decodes [INFERRED]; tolerable. No MPS.
7. **Version pins to write down:** faster-whisper 1.2.1; wyoming 1.10.2; wyoming-faster-whisper 3.7.0; wyoming-piper 2.4.3; Kokoro-FastAPI 0.8.2 (Kokoro-82M v1.0); onnx-asr ≥ 0.11.0; parakeet-tdt-0.6b-v2 (2025-05-01); NVIDIA driver supporting CUDA 12.8 for the wyoming GPU image.

## Numbers

| quantity | value | hardware/context | tag | source URL |
|---|---|---|---|---|
| Vosk small-en-us-0.15 WER | 9.85 % LibriSpeech test-clean, 10.38 % TEDLIUM; 40 MB | Pi/Android model | VENDOR | https://alphacephei.com/vosk/models |
| whisper-large-v3-turbo | 809M params, 4 decoder layers, Open ASR mean WER 7.83 | leaderboard eval | VENDOR | https://huggingface.co/openai/whisper-large-v3-turbo |
| distil-large-v3.5 | 756M, short-form WER 7.08 vs turbo 7.30; ~1.5× faster than turbo; English only | leaderboard eval | VENDOR | https://huggingface.co/distil-whisper/distil-large-v3.5 |
| faster-whisper large-v2 fp16 beam 5 | 13 min audio in 63 s, 4525 MB VRAM; int8: 59 s, 2926 MB | RTX 3070 Ti 8 GB, CUDA 12.4 | MEASURED | https://github.com/SYSTRAN/faster-whisper |
| faster-whisper turbo, 3 s command | 0.15–0.35 s end-to-end, 2–3 GB VRAM | RTX 3090, fp16, beam 1 | INFERRED | derived from the row above and the 32→4 decoder-layer reduction |
| Parakeet-TDT-0.6B-v2 | mean WER 6.05, LibriSpeech clean 1.69, RTFx 3386 | A100, batched | VENDOR | https://huggingface.co/nvidia/parakeet-tdt-0.6b-v2 |
| Parakeet-TDT-0.6B-v3 | mean WER 6.34, RTFx 3333, 25 languages | A100 | VENDOR | https://huggingface.co/nvidia/parakeet-tdt-0.6b-v3 |
| Parakeet v2/v3 via onnx-asr | RTFx 36 (Ryzen 9800X3D CPU), 57 (T4 CUDA), 320 (RTX 5070 Ti TensorRT), 1.0 (Cortex-A53) | per README table | MEASURED | https://github.com/istupakov/onnx-asr |
| Parakeet v2 int8 via sherpa-onnx | RTF 0.088 at 4 threads, 0.220 at 1 thread; encoder 622 MB | RK3588 Cortex-A76 | MEASURED | https://k2-fsa.github.io/sherpa/onnx/pretrained_models/offline-transducer/nemo-transducer-models.html |
| Nemotron streaming 0.6B | latency options 0.08/0.16/0.56/1.12 s; mean WER 6.93 at 1.12 s | leaderboard eval | VENDOR | https://huggingface.co/nvidia/nemotron-speech-streaming-en-0.6b |
| Kyutai stt-1b-en_fr | 0.5 s delay, semantic VAD; 64 streams at 3× RT | L40S | VENDOR | https://github.com/kyutai-labs/delayed-streams-modeling |
| Qwen3-ASR-0.6B | 0.9B params; LibriSpeech 2.11/4.55 vs whisper-large-v3 1.51/3.97 | vendor eval | VENDOR | https://huggingface.co/Qwen/Qwen3-ASR-0.6B |
| Voxtral-Mini-3B-2507 | ~9.5 GB VRAM bf16, vLLM ≥ 0.10.0 | vendor | VENDOR | https://huggingface.co/mistralai/Voxtral-Mini-3B-2507 |
| Whisper on Pi 4 (HA add-on) | "around 8 seconds" per command; Piper medium 1.6 s audio per second | Raspberry Pi 4 | MEASURED (vendor-run) | https://www.home-assistant.io/voice_control/voice_remote_local_assistant/ |
| Piper medium first-sentence audio on Pi 4 | 0.6–1.6 s for a 1–2.5 s sentence (RTF ≈ 0.62) | Pi 4 | INFERRED | from the HA figure above |
| Kokoro-FastAPI TTFA | ~300 ms GPU (chunk 400); ~3500 ms CPU (older i7, chunk 200) | RTX 4060 Ti 16 GB, CUDA 12.1 | VENDOR (maintainer-measured) | https://github.com/remsky/Kokoro-FastAPI |
| Kokoro-FastAPI throughput / VRAM | 35–100× realtime, 137.67 tok/s; 2.37 GB floor, 3.11 GB loaded | RTX 4060 Ti | VENDOR (maintainer-measured) | https://github.com/remsky/Kokoro-FastAPI |
| Kokoro ONNX model size | ~300 MB fp32, ~80 MB quantized | any | VENDOR | https://github.com/thewh1teagle/kokoro-onnx |
| Orpheus TTS | 3B params, ~200 ms streaming latency, ~100 ms with input streaming; needs vLLM | vendor | VENDOR | https://github.com/canopyai/Orpheus-TTS |
| Qwen3-TTS | 0.6B/1.7B; "latency as low as 97 ms" single-character input | vendor | VENDOR | https://github.com/QwenLM/Qwen3-TTS |
| Gemma-3-27B geometry | 62 layers, 16 KV heads, head_dim 128, sliding_window 1024, pattern 6, 256 tokens/image at 896 px | config.json | VENDOR | https://huggingface.co/unsloth/gemma-3-27b-it/raw/main/config.json |
| Gemma-3-27B int4 checkpoint | 18.5 GB (4 safetensors shards) | gaunernst QAT-int4 AWQ layout | VENDOR | https://huggingface.co/gaunernst/gemma-3-27b-it-int4-awq/tree/main |
| KV per token, full-attention layers only | 80 KiB/token + 416 MiB fixed per sequence (sliding layers) | vLLM hybrid KV manager, bf16 | INFERRED | arithmetic from config + https://raw.githubusercontent.com/vllm-project/vllm/main/docs/design/hybrid_kv_cache_manager.md |
| vLLM footprint per 3090 | ~12.5–13.5 GiB (9.3 weights + ~2 KV for 4×8k + 1–2 activations/graphs) | TP=2, int4 | INFERRED | see above |
| vLLM `gpu_memory_utilization` | default 0.92; "a per-instance limit ... does not matter if you have another vLLM instance running on the same GPU" | vLLM main, Sep 2026 | VENDOR | https://raw.githubusercontent.com/vllm-project/vllm/main/vllm/config/cache.py |
| openWakeWord CPU | 15–20 models on one Pi 3 core; 80 ms frames | Raspberry Pi 3 | VENDOR | https://github.com/dscripka/openWakeWord |
| Silero VAD | <1 ms per 30 ms chunk, one CPU thread; ~2 MB | x86 CPU | VENDOR | https://github.com/snakers4/silero-vad |
| Audio transport | 16 kHz × 16-bit mono = 256 kbit/s; a 3 s command = 96 KB; ≤ 50 ms on LAN Wi-Fi | Pi 4 Wi-Fi | INFERRED | arithmetic |
| Wyoming streaming TTS/ASR | added in wyoming 1.7.0 (2025-06-23); current 1.10.2 (2026-08-27) | protocol | VENDOR | https://pypi.org/project/wyoming/#history |

## Corrections to the brief

1. **"STT/TTS move off the Pi, voice turns drop ~1–1.5 s."** Too high. STT: with Vosk streaming on the Pi the final text already lands ~0.3 s after end of speech; with box STT you still wait for the Pi's silence timer (0.6–0.8 s) and then ~0.2–0.35 s of GPU work, so STT gets no faster and may lose 0.1–0.3 s. TTS: Pi Piper medium first sentence 0.6–1.6 s (HA's RTF 0.62) vs Kokoro GPU ~0.3 s: saves 0.3–1.3 s. Net ~0.5–1.0 s. The real wins are WER (9.85 % → ~2 % on clean speech, larger gap in a room with motor noise), punctuation/casing for the LLM, and ~2 Pi cores freed. State it that way.
2. **"realistic headroom for a 1–3 GB STT model and <1 GB TTS."** STT is right for faster-whisper turbo fp16. TTS is wrong for Kokoro-FastAPI: 2.37 GB floor, 3.11 GB loaded (maintainer measurement on a 4060 Ti). Under 1 GB only via ONNX Runtime Kokoro or Piper on CPU. Headroom is not the issue anyway: ~8 GiB per card stays free at `--gpu-memory-utilization 0.65`.
3. **"Piper first audio 0.3–1 s [INFERRED]."** HA's own figure gives RTF ≈ 0.62 for medium voices on a Pi; a typical 1.5–2.5 s first sentence therefore takes 0.9–1.6 s. Use a `low` voice or a short first sentence if TTS stays on the Pi.
4. **"faster-whisper or Parakeet, plus Kokoro."** The choice holds in 2026. Add: the Wyoming server now hosts both (`wyoming-faster-whisper` 3.7.0 with `--stt-library onnx-asr`), so "faster-whisper or Parakeet" is a flag, not a different container. Parakeet-TDT-0.6B-v2 beats turbo on WER (6.05 vs 7.83) and runs fast enough on a desktop CPU to skip the GPU entirely.
5. **"Vosk final text ~0.3 s after end of speech."** Plausible, unverified this session; the point that matters is Vosk small's 9.85 % WER.
6. **"Speak a filler immediately."** Pre-render the fillers as WAVs on the Pi; then filler latency is 0 ms in both designs, which further shrinks the perceived difference between Pi and box TTS.
7. **Box "dumb" constraint.** Two more containers cost ~5–6 GB on one GPU (and a ~10.7 GB wyoming GPU image on disk). The LLM is unaffected only if `gpu_memory_utilization` is set explicitly; the 0.92 default reserves 22 GiB per card and starves the speech containers.
8. **Image tokens (adjacent).** Gemma 3 resizes any frame to 896×896 = 256 tokens (`mm_tokens_per_image`); JPEG size does not matter.

## Alternatives considered and rejected

- **Speaches as the single container.** Best feature set (STT + Kokoro + Realtime API), but last release v0.9.0-rc.3 (2025-12-27) vs nine 2026 releases for the Wyoming pair. Its model TTL unloads idle models and costs a reload on the next command; set it to never if used.
- **Voxtral Mini 3B / Voxtral Realtime 4B / Qwen3-ASR via vLLM.** Each needs a second vLLM instance (~9.5 GB bf16 for Voxtral Mini; 2–4 GB plus engine overhead for Qwen3-ASR-0.6B). Qwen3-ASR-0.6B is less accurate than whisper-large-v3 on English. No gain for single-user commands.
- **vLLM's own `/v1/audio/transcriptions` with Whisper.** Same second-instance overhead; faster-whisper is lighter for one stream at a time.
- **Kyutai STT 1B (semantic VAD, 0.5 s delay).** Best latency architecture, but needs `moshi-server`, 24 kHz audio, English/French only, ~3 GB VRAM [INFERRED]. Revisit if end-of-turn latency is the bottleneck.
- **Nemotron streaming 0.6B via sherpa-onnx.** Good CPU streaming option; adds a live-vs-final merge on the Pi. Later.
- **Canary-1B-v2 (WER 7.15, RTFx 749), Moonshine.** Canary is slower than Parakeet for no WER gain; Moonshine publishes no WER on its README.
- **Other TTS (Orpheus, Chatterbox, Qwen3-TTS, Fish S2-Pro, Dia, F5).** See above: second engine, no streaming server, no OpenAI endpoint, restrictive license, or non-streaming.
- **Piper with `use_cuda`.** Already faster than needed on a desktop CPU.
- **MPS, Opus, wake word on the box.** Irrelevant for one STT burst per turn; 256 kbit/s PCM is trivial; wake word on the box puts Wi-Fi in the always-on path.
- **Posting a WAV after endpointing.** Works, but adds the whole upload (~50–100 ms) after the endpoint; streaming chunks during speech hides it.

## Open questions

1. What CPU is in the GPU box? It decides whether Parakeet on CPU (zero VRAM) is ~0.1 s or ~0.4 s per command.
2. No source measured faster-whisper large-v3-turbo on a 3090 for a 3 s clip; the 0.15–0.35 s figure is derived. Measure at gate 3 with `curl` timing against the container.
3. Which vLLM version is on the box? Confirm the hybrid KV manager is active (the startup log's KV-cache size should be far larger than 62-layer arithmetic predicts) and whether that version still hard-fails when free VRAM at startup is below `gpu_memory_utilization × total`.
4. Exact Hugging Face id resolved by faster-whisper's `large-v3-turbo` alias (a `mobiuslabsgmbh` or `deepdml` CT2 conversion); verify before pinning.
5. Kokoro-FastAPI first-audio latency on a 3090 while the 27B is decoding; the 300 ms figure is idle on a 4060 Ti.
6. Speaches maintenance status after 2025-12-27.
7. Can the orchestrator stream the `say` field to TTS before the JSON object closes (put `say` first in the schema and parse incrementally)? Otherwise TTS starts only after all 30–60 JSON tokens.
8. Silero VAD + openWakeWord + 10 fps JPEG on the Pi 4 concurrently: CPU share not measured here.

## Sources

- SYSTRAN/faster-whisper README (benchmarks, CUDA 12/cuDNN 9) — https://github.com/SYSTRAN/faster-whisper — read 2026-09-07
- faster-whisper releases (1.2.1 2025-10-31; 1.2.0 2025-08-06; 1.1.0 2024-11-21) — https://github.com/SYSTRAN/faster-whisper/releases
- faster-whisper transcribe.py (hotwords, initial_prompt, beam_size) — https://raw.githubusercontent.com/SYSTRAN/faster-whisper/master/faster_whisper/transcribe.py
- rhasspy/wyoming-faster-whisper README and releases (3.7.0 2026-09-01) — https://github.com/rhasspy/wyoming-faster-whisper ; https://github.com/rhasspy/wyoming-faster-whisper/releases
- rhasspy/wyoming-piper releases (2.4.3 2026-09-03) — https://github.com/rhasspy/wyoming-piper/releases
- OHF-Voice/wyoming protocol README; PyPI history (1.7.0 2025-06-23, 1.10.2 2026-08-27) — https://github.com/OHF-Voice/wyoming ; https://pypi.org/project/wyoming/#history
- speaches-ai/speaches README, releases (0.9.0-rc.3 2025-12-27), docs — https://github.com/speaches-ai/speaches ; https://github.com/speaches-ai/speaches/releases ; https://speaches.ai/installation/
- remsky/Kokoro-FastAPI README and releases (0.8.2 2026-09-05) — https://github.com/remsky/Kokoro-FastAPI ; https://github.com/remsky/Kokoro-FastAPI/releases
- hexgrad/Kokoro-82M (v1.0 2025-01-27) — https://huggingface.co/hexgrad/Kokoro-82M
- thewh1teagle/kokoro-onnx — https://github.com/thewh1teagle/kokoro-onnx
- nvidia/parakeet-tdt-0.6b-v2 (2025-05-01) — https://huggingface.co/nvidia/parakeet-tdt-0.6b-v2
- nvidia/parakeet-tdt-0.6b-v3 (2025-08-14) — https://huggingface.co/nvidia/parakeet-tdt-0.6b-v3
- istupakov/onnx-asr (RTFx table) — https://github.com/istupakov/onnx-asr
- sherpa-onnx NeMo offline transducer models (RK3588 RTF) — https://k2-fsa.github.io/sherpa/onnx/pretrained_models/offline-transducer/nemo-transducer-models.html
- nvidia/canary-1b-v2 (2025-08-14) — https://huggingface.co/nvidia/canary-1b-v2
- nvidia/nemotron-speech-streaming-en-0.6b (2026-01-05, upd. 2026-03-13) — https://huggingface.co/nvidia/nemotron-speech-streaming-en-0.6b
- distil-whisper/distil-large-v3.5 (2025-03-25) — https://huggingface.co/distil-whisper/distil-large-v3.5
- openai/whisper-large-v3-turbo — https://huggingface.co/openai/whisper-large-v3-turbo
- QwenLM/Qwen3-ASR (2026-01-29); Qwen/Qwen3-ASR-0.6B — https://github.com/QwenLM/Qwen3-ASR ; https://huggingface.co/Qwen/Qwen3-ASR-0.6B
- QwenLM/Qwen3-TTS (2026-01-22) — https://github.com/QwenLM/Qwen3-TTS
- mistralai/Voxtral-Mini-3B-2507; Voxtral Transcribe 2 announcement (2026-02-04) — https://huggingface.co/mistralai/Voxtral-Mini-3B-2507 ; https://mistral.ai/news/voxtral-transcribe-2
- kyutai-labs/delayed-streams-modeling; kyutai/stt-1b-en_fr — https://github.com/kyutai-labs/delayed-streams-modeling ; https://huggingface.co/kyutai/stt-1b-en_fr
- canopyai/Orpheus-TTS — https://github.com/canopyai/Orpheus-TTS
- resemble-ai/chatterbox — https://github.com/resemble-ai/chatterbox
- fishaudio/fish-speech (S2-Pro) — https://github.com/fishaudio/fish-speech
- moonshine-ai/moonshine — https://github.com/moonshine-ai/moonshine
- ggml-org/whisper.cpp — https://github.com/ggml-org/whisper.cpp
- KoljaB/RealtimeSTT (nemotron streaming + parakeet final recommendation) — https://github.com/KoljaB/RealtimeSTT
- vLLM cache config (gpu_memory_utilization, kv_cache_memory_bytes) — https://raw.githubusercontent.com/vllm-project/vllm/main/vllm/config/cache.py
- vLLM hybrid KV cache manager design doc — https://raw.githubusercontent.com/vllm-project/vllm/main/docs/design/hybrid_kv_cache_manager.md
- vLLM V1 gpu_worker.py (memory profiling messages) — https://raw.githubusercontent.com/vllm-project/vllm/main/vllm/v1/worker/gpu_worker.py
- unsloth/gemma-3-27b-it config.json — https://huggingface.co/unsloth/gemma-3-27b-it/raw/main/config.json
- gaunernst/gemma-3-27b-it-int4-awq file tree (18.5 GB) — https://huggingface.co/gaunernst/gemma-3-27b-it-int4-awq/tree/main
- dscripka/openWakeWord (v0.6.0 2024-02-11) — https://github.com/dscripka/openWakeWord
- snakers4/silero-vad — https://github.com/snakers4/silero-vad
- rhasspy/wyoming-satellite (archived); OHF-Voice/linux-voice-assistant — https://github.com/rhasspy/wyoming-satellite ; https://github.com/OHF-Voice/linux-voice-assistant
- OHF-Voice/piper1-gpl Python API — https://github.com/OHF-Voice/piper1-gpl/blob/main/docs/API_PYTHON.md
- Home Assistant local voice assistant docs (Whisper on Pi 4 ~8 s; Piper 1.6 s/s) — https://www.home-assistant.io/voice_control/voice_remote_local_assistant/
- Vosk models (small-en-us-0.15 WER) — https://alphacephei.com/vosk/models
- NVIDIA MPS documentation — https://docs.nvidia.com/deploy/mps/index.html
