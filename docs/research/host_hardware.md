# Onboard host: is a Raspberry Pi 4 (4 GB) still the right choice in 2026?

Research date: 2026-09-07. Dimension: host hardware for a Pi + ESP32 + "dumb GPU box" indoor rover where all heavy inference (27B VLM, optionally STT/TTS) runs on the box.

Method note: the session's web-search budget was exhausted before the first query, so every fact comes from direct fetches of primary pages (Raspberry Pi Ltd docs/news/product briefs, NVIDIA docs, GitHub sources, Rockchip repos, REP 2000, reseller listings on 2026-09-07). Tags: [MEASURED] run on named hardware; [VENDOR] spec sheet or vendor claim; [INFERRED] my estimate.

## Summary

- Keep the Pi 4 4 GB for the first build. With the box doing the 27B and (ideally) STT/TTS, the Pi's residual work is wake word, audio I/O, a 640x480 JPEG at 10 fps, a state machine, and serial. Every measured data point says a Pi 4 carries that with three cores idle.
- The 2026 price picture has flipped against casual upgrades. Memory-driven increases on 1 Dec 2025, 2 Feb 2026 and 1 Apr 2026 put the official list price of a Pi 5 4 GB at $110 (was $60), Pi 5 8 GB at $175 (was $80), Pi 5 16 GB at $305 (was $120), and a Pi 4 4 GB at $100 (was $55) [VENDOR]. Raspberry Pi says it will reverse these when LPDDR4 prices fall. Buying a Pi 5 today costs roughly 1.8x what it did a year ago, and the 4 GB variant is out of stock at Adafruit (in stock at PiShop).
- The Pi 5 is a real 2-3x CPU step (Geekbench 6 ~3x, 7-zip ~2x, YOLOv8n NCNN 4.4x) at 3.0-3.6 W idle and 8.8-11.6 W all-core versus 2.7 W / 4.8-6.4 W for the Pi 4, needs the $5 Active Cooler, and has no hardware H.264 or JPEG encoder. None of that matters for a 640x480 JPEG at 10 fps, which is software JPEG on both boards.
- Upgrade triggers (detailed below): Whisper-class STT on the robot (Pi 4 ~7-8 s per command, Pi 5 ~2.2 s), on-robot detection above ~2 fps beyond what the $70 AI Camera on the Pi 4 gives, wanting an AI HAT+ / AI HAT+ 2 (PCIe, Pi 5 only), matching the LeKiwi BOM (Pi 5 4 GB), or `vcgencmd get_throttled` flags in the all-on gate.
- When a trigger fires or the Pi 4 dies, buy a Pi 5 4 GB ($110 list) + Active Cooler ($5) + Standard-Mini camera cable (~$2) and reuse the 5 V/5 A buck. Do not buy another Pi 4 ($100 for a 2-3x slower board) or a Jetson Orin Nano Super for this architecture ($249 MSRP; $399 and backordered at SparkFun on 2026-09-07).

## State of the art (2026)

### Raspberry Pi 5 vs Raspberry Pi 4

Hardware. Pi 5: BCM2712, 4x Cortex-A76 @ 2.4 GHz, 512 KB L2/core, 2 MB L3, VideoCore VII, LPDDR4X-4267 in 1/2/4/8/16 GB, PCIe 2.0 x1 (Gen 3 by config), 2x 4-lane MIPI camera/display transceivers on 22-way mini FPC, 5 V/5 A USB-C with PD, "4Kp60 HEVC decoder" as the only listed hardware codec, in production until at least January 2036 [VENDOR, product brief published April 2026]. Pi 4: BCM2711, 4x Cortex-A72 @ 1.8 GHz, "H.265 (4kp60 decode), H264 (1080p60 decode, 1080p30 encode)", one 2-lane 15-way CSI port, 5 V/3 A, in production until at least January 2034 [VENDOR].

Speed. Raspberry Pi claims "a 2-3x increase in CPU performance" [VENDOR]. Measured: Geekbench 6 "3x the performance on both single and multi-core", PassMark CPU "over 4x", UnixBench 2-3x (bret.dk, Nov 2023) [MEASURED]; 7-zip 10,930 MIPS vs ~5,400 MIPS for the Pi 4 (cnx-software, Nov 2023) [MEASURED]; AES-256 21x faster thanks to the Armv8 crypto extension [MEASURED]. For the workload that matters here, Ultralytics' own benchmark (docs at ultralytics v8.3.0, Bookworm, FP32, imgsz 640) gives YOLOv8n NCNN 414.73 ms on Pi 4 vs 94.28 ms on Pi 5 (4.4x), ONNX 560.04 ms vs 198.69 ms [MEASURED]. The current guide adds YOLO26n on Pi 5: NCNN 67.03 ms, ONNX 125.99 ms (7.79 fps) [MEASURED].

Power and thermals. Pi 4B: idle 2.7 W, `stress --cpu 4` 6.4 W (pidramble) [MEASURED]; 4.84 W under stress-ng with the performance governor (bret.dk) [MEASURED]. Pi 5: power-off 1.7 W, idle headless Wi-Fi 3.0 W, idle with Ethernet + HDMI + peripherals 3.6 W, 4-core stress 8.8 W, 4K video + USB I/O + stress 15.9-16.8 W (cnx-software) [MEASURED]; 11.6 W under stress-ng with the performance governor (bret.dk) [MEASURED]. Raspberry Pi's launch post says "around 12 W" peak vs the Pi 4's 8 W [VENDOR]. Bare Pi 5 under stress-ng throttled after 9 seconds at 85 C; with a fan it settled around 83 C after 4 minutes (bret.dk) [MEASURED]; with the official Active Cooler it peaked ~66 C with no throttling at 28 C ambient (cnx-software) [MEASURED]. Treat the Active Cooler as mandatory in a rover enclosure.

Power supply. Pi 5 spec: "5 V at 5 A (25 W); or 5 V at 3 A (15 W) with a 600 mA peripheral limit" [VENDOR]. A bench buck converter does no PD negotiation, so a Pi 5 fed from the BOM's 5 V/5 A buck will assume a 3 A supply and cap the USB ports at 600 mA (USB mic + USB serial are fine; a USB camera or SSD is not) unless you set `usb_max_current_enable=1` in `config.txt` [INFERRED from the docs; I could not fetch the exact paragraph in this session, confirm on the config.txt documentation page].

Video encode. Camera docs: "Raspberry Pi 5 uses software video encoders" for H.264 via libav; `--low-latency` "will still easily achieve 1080p30" [VENDOR]. In picamera2, `_hw_encoder_available = get_platform() == Platform.VC4`; otherwise `H264Encoder` is `LibavH264Encoder` and `MJPEGEncoder` is `LibavMjpegEncoder` [MEASURED, source]. The product brief lists only the HEVC decoder, so no hardware JPEG path either [VENDOR/INFERRED]. The Pi 4 keeps hardware H.264 1080p30 encode and hardware MJPEG. Irrelevant at 640x480/10 fps; it bites only for future 1080p H.264 teleop video, where the Pi 4 is cheaper on CPU.

Camera connectors. Camera Module 3 (IMX708, 12 MP, PDAF, "from $25", in production until at least January 2030) ships with a 15-way standard cable; a Pi 5 needs the Standard-Mini 22-way camera cable (200/300/500 mm) [VENDOR].

Prices (official list, April 2026 product brief plus the three price-rise posts) [VENDOR]: Pi 5 1/2/4/8/16 GB $45/$65/$110/$175/$305. The Pi 5 4 GB went $60 -> $70 (1 Dec 2025) -> $85 (2 Feb 2026) -> $110 (1 Apr 2026); the 8 GB $80 -> $175; the 16 GB $120 -> $305; the Pi 4 4 GB $55 -> $100 and 8 GB $75 -> $165; a 3 GB Pi 4 appeared at $83.75. Stated cause: "a seven-fold increase over the last year in the price of the LPDDR4 DRAM"; stated intent: "we will reverse our price increases" when it abates. Resellers on 2026-09-07 [MEASURED listing]: PiShop at list (Pi 5 4 GB $110, 8 GB $175 in stock, Pi 4 4 GB $100); Adafruit above list (Pi 5 4 GB $130 out of stock, 8 GB $200, 16 GB $350, Pi 4 4 GB $120).

### Pi 5 AI add-ons and the AI Camera

- AI HAT+ (Hailo-8L 13 TOPS / Hailo-8 26 TOPS): "starting at $70", Pi 5 only, apt `hailo-all`, Hailo toolchain 4.17-4.19, PCIe Gen 3 applied automatically, rpicam-apps post-processing stages for YOLOv5/v6/v8/YOLOX detection, segmentation and 17-point pose [VENDOR]. Requires 64-bit Raspberry Pi OS Trixie.
- AI HAT+ 2 (Hailo-10H, "40 TOPS (INT4)", 8 GB on-board RAM): launched 15 Jan 2026 at $130, raised $50 on 1 Apr 2026, product page now says $200 [VENDOR]. Pi 5 only, apt `hailo-h10-all` (cannot co-exist with `hailo-all`), Hailo Gen-AI Model Zoo 5.1.1, launch models DeepSeek-R1-Distill 1.5B, Llama 3.2 1B, Qwen2.5 1.5B variants, served through hailo-ollama [VENDOR]. Raspberry Pi's one published comparison: Qwen2.5-1.5B int4, 96 prefill tokens, Pi 5 CPU 2039 ms vs Hailo-10H 320 ms [MEASURED, vendor blog]. It does not speed up a 27B on the box; it is a path to on-robot small-VLM/LLM only.
- AI Camera (Sony IMX500, $70): on-sensor inference, documented for "either a Raspberry Pi 4 Model B or Raspberry Pi 5", also Zero 2 W and 3B+ "with minor changes"; apt `imx500-all`; bundled MobileNet SSD and PoseNet; convert your own with `pip install edge-mdt[pt]`, `imxconv-pt`, `imx500-package` [VENDOR]. This is the one on-robot detection upgrade that does not require replacing the Pi 4.

### Jetson Orin Nano Super Developer Kit

67 INT8 TOPS, 1024 CUDA cores, 6x Cortex-A78AE, 8 GB LPDDR5 at 102 GB/s, 7-25 W [VENDOR]. Announced 17 Dec 2024 at "$249, down from $499" [VENDOR]; street on 2026-09-07: SparkFun $399.00 "Backorder", OKdo EUR 348 ex VAT [MEASURED listing]. Software: JetPack 6.2.1 (Jun 2025, Ubuntu 22.04, CUDA 12.6, TensorRT 10.3) or JetPack 7.2.1 (Aug 2026, Ubuntu 24.04, CUDA 13.0, Orin family, Super Mode default for the Orin Nano dev kit) [VENDOR]. Measured (NVIDIA Jetson AI Lab, Super mode): Llama 3.2 3B 43.07 tok/s, Qwen2.5 7B 21.75 tok/s, Qwen2-VL 2B 4.4 fps, SmolVLM 2B 12.9 fps [MEASURED, vendor]; YOLO26n TensorRT FP16 4.57 ms at 640 [MEASURED]. A Jetson could run faster-whisper, YOLO and a 2B VLM on the robot at once. It also draws 7-25 W plus fan, wants a 19 V supply, lacks the Pi camera stack (Camera Module 3's IMX708 needs a third-party driver [INFERRED]), and costs 3.6x a Pi 5 4 GB. For a design whose thesis is "the box does inference", it is overkill.

### RK3588 boards (Orange Pi 5, Radxa Rock 5)

Software has matured: rknn-toolkit2 v2.3.2 (Python 3.6-3.12), rknn-llm v1.3.0 (Qwen3/3.5, Gemma 3/4, Qwen3-VL, SmolVLM, InternVL, MiniCPM-V) [VENDOR]. rknn_model_zoo 2.3.2 lists RK3588 single-NPU-core numbers: yolov8n INT8 640 73.5 fps, yolo11n 60.0 fps, whisper_base (20 s) RTF 0.215 [VENDOR-run measurement]. A useful 6 TOPS NPU for STT and detection. Costs: vendor kernels/BSPs, no libcamera/picamera2 ecosystem, per-model RKNN conversion, and the same 2026 LPDDR price hit. Worth it only for on-robot Whisper + YOLO without the box, off the Pi software path.

### x86 N100/N150 boards and mini PCs

Radxa X4 (N100, 85x56 mm, 40-pin header via RP2040, LPDDR5 4-16 GB, M.2 2230 NVMe, USB-C PD 12 V >= 2.5 A) [VENDOR]. Measured (cnx-software, Sept 2024, Ubuntu 24.04): idle 6 W with fan, 12 W load, Speedometer 2.0 175 vs 56 on a Pi 5, but 7-zip 8,120 MIPS vs the Pi 5's 10,930 because the default 6 W PL1 throttles; 97 C peak with PL1 raised [MEASURED]. "From $80" in 2024, pre-DRAM run-up. ROS 2 Jazzy and Kilted are Tier 1 on Ubuntu 24.04 amd64 and arm64 alike (REP 2000) [VENDOR], so x86 buys no ROS advantage, only 2x the idle power, a 12 V rail and no CSI camera.

### The speech and camera loads the Pi actually carries

- openWakeWord 0.6.0: "a single core of a Raspberry Pi 3 can run 15-20 openWakeWord models simultaneously in real-time" [VENDOR README].
- Piper: "optimized for the Raspberry Pi 4" [VENDOR]; Home Assistant measured 2 s of audio per 1 s of Pi 4 CPU (2023) and "1.6s of voice in a second" with medium voices (current docs) [MEASURED].
- Whisper on a Pi 4: "around 7 seconds" (HA, 2023) to "around 8 seconds" (HA docs, 2026) per command [MEASURED]. Pi 5: wyoming-faster-whisper tiny-int8, 4 threads, 3.2 s command, 2.20 s latency [MEASURED]. Whisper-class STT on the Pi 4 is not viable for a conversational robot; Vosk or the box is.
- Vosk on a Pi 4: no measured source found; the brief's "~1 core, ~0.3 s final" stays [INFERRED] until gate 3 measures it.
- Camera: 640x480 JPEG at 10 fps is software JPEG through picamera2 on both boards; the brief's 20-40% of a Pi 4 core is plausible [INFERRED].

### Robotics ecosystem anchors

- TurtleBot 4 ships a "Raspberry Pi 4B 4GB" with an RPLIDAR A1M8 and launches localization and Nav2 on the robot (Humble, Jazzy) [VENDOR]: Nav2 + AMCL on a Pi 4 4 GB is a shipped configuration, on Ubuntu (Tier 1), not Raspberry Pi OS (Debian, Tier 3).
- LeRobot LeKiwi: BOM lists "Raspberry Pi 5 (4GB)" (stale $60); docs say the host "can be any PC that can run on 5V and has enough usb ports (2 or more)"; policy inference runs on the laptop [VENDOR].

## Recommendation for this build

Keep the Pi 4 4 GB. Reasons, in order:

1. The architecture already moved every heavy load off the robot. Measured residuals on a Pi 4 (openWakeWord a few percent of a core, Piper 1.6-2x real time, software JPEG at 10 fps under a core) leave two or three cores free. The only load that would not fit is Whisper-class STT, which the brief already keeps off the Pi.
2. A Pi 5 4 GB is $110 list ($130 and out of stock at Adafruit), 1.8x last year's, and the vendor says the increase is temporary. Paying the peak for headroom you do not use is the wrong trade.
3. A Pi 5 draws 0.3-0.9 W more idle and 3-5 W more under load on the 3S pack and needs the Active Cooler in an enclosure. The Pi 4 is the lower-risk choice for the "30 min all-on, no get_throttled flags" gate.

Do these three things now so an upgrade later is a swap, not a redesign:
- Keep the 5 V/5 A buck in the BOM (a Pi 4 needs 3 A; a Pi 5 needs 5 A) and plan for `usb_max_current_enable=1` on a Pi 5.
- Run the Pi on 64-bit Raspberry Pi OS (Trixie if you want the Hailo packages later) or Ubuntu 24.04 arm64 if Nav2 on-robot is likely; both boards boot the same image, so migration is a card move plus the 22-way camera cable.
- Log CPU per process during the desk gate (`top -H`, `vcgencmd measure_temp`, `vcgencmd get_throttled`). Those numbers are the upgrade evidence.

Upgrade to a Pi 5 4 GB (about $117: $110 board + $5 Active Cooler + ~$2 Standard-Mini camera cable; reuse the buck and Camera Module 3) when any of these happens:
- STT must live on the robot and Vosk accuracy is not enough. Pi 4 whisper: 7-8 s per command; Pi 5 faster-whisper tiny-int8: ~2.2 s for a 3.2 s utterance. (Cheaper fix first: STT on the box, which the brief already lists as the highest-value change.)
- You want on-robot detection faster than the Pi 4's 2.4 fps YOLOv8n (NCNN, 640) and the $70 AI Camera on the Pi 4 does not cover it. Pi 5 CPU alone gives 10.6 fps (YOLOv8n) to 15 fps (YOLO26n); Pi 5 + AI HAT+ 13 TOPS ($70) moves detection off the CPU entirely.
- You want the AI HAT+ 2 for a local 1-3B fallback LLM/VLM when Wi-Fi to the box drops (Pi 5 only, $200).
- You merge into LeKiwi and want to match the reference BOM (Pi 5 4 GB), or you run two USB cameras plus the Feetech bus on one host and hit USB/CPU limits on the Pi 4.
- `vcgencmd get_throttled` is non-zero during the all-on gate after you have ruled out the power supply.
- The Pi 4 dies: replace with a Pi 5 4 GB, not a $100 Pi 4.

Memory: 4 GB is sufficient for this stack on either board (Vosk small model ~50 MB, Piper medium voice tens of MB, Python orchestrator, picamera2). 8 GB ($175) only if you plan Nav2 + SLAM + ROS 2 tooling on the robot alongside speech; 16 GB ($305) has no use here.

## Numbers

| quantity | value | hardware/context | tag | source URL |
|---|---|---|---|---|
| Pi 5 list price 1/2/4/8/16 GB | $45 / $65 / $110 / $175 / $305 | official list, product brief published April 2026 | VENDOR | https://pip-assets.raspberrypi.com/categories/892-raspberry-pi-5/documents/RP-008348-DS-6-raspberry-pi-5-product-brief.pdf |
| Pi 5 4 GB price steps | $60 -> $70 -> $85 -> $110 | launch -> 1 Dec 2025 -> 2 Feb 2026 -> 1 Apr 2026 | VENDOR | https://www.raspberrypi.com/news/a-new-3gb-raspberry-pi-4-for-83-75-and-more-memory-driven-price-increases/ |
| Pi 4 4 GB price steps | $55 -> $60 -> $75 -> $100 | same dates | VENDOR | https://www.raspberrypi.com/news/more-memory-driven-price-rises/ |
| LPDDR4 price rise | "seven-fold increase over the last year" | Raspberry Pi Ltd, 1 Apr 2026 | VENDOR | https://www.raspberrypi.com/news/a-new-3gb-raspberry-pi-4-for-83-75-and-more-memory-driven-price-increases/ |
| Pi 5 4 GB reseller | $110 (PiShop); $130 out of stock (Adafruit) | listings 2026-09-07 | MEASURED | https://www.pishop.us/product/raspberry-pi-5-4gb/ |
| Pi 4 4 GB reseller | $100 (PiShop); $120 (Adafruit) | listings 2026-09-07 | MEASURED | https://www.pishop.us/product/raspberry-pi-4-model-b-4gb/ |
| Pi 5 CPU uplift claim | "2-3x increase in CPU performance" | vs Pi 4, Raspberry Pi Ltd | VENDOR | https://pip-assets.raspberrypi.com/categories/892-raspberry-pi-5/documents/RP-008348-DS-6-raspberry-pi-5-product-brief.pdf |
| Geekbench 6 | ~3x single and multi-core | Pi 5 vs Pi 4, bret.dk Nov 2023 | MEASURED | https://bret.dk/raspberry-pi-5-review/ |
| 7-zip | 10,930 vs ~5,400 MIPS | Pi 5 vs Pi 4, cnx-software Nov 2023 | MEASURED | https://www.cnx-software.com/2023/11/05/raspberry-pi-5-review-raspberry-pi-os-bookworm-benchmarks-power-consumption/ |
| YOLOv8n NCNN 640 FP32 | 414.73 ms (Pi 4) / 94.28 ms (Pi 5) | Bookworm, Ultralytics v8.3.0 docs | MEASURED | https://raw.githubusercontent.com/ultralytics/ultralytics/v8.3.0/docs/en/guides/raspberry-pi.md |
| YOLOv8n ONNX 640 | 560.04 ms (Pi 4) / 198.69 ms (Pi 5) | same | MEASURED | https://raw.githubusercontent.com/ultralytics/ultralytics/v8.3.0/docs/en/guides/raspberry-pi.md |
| YOLO26n Pi 5 | NCNN 67.03 ms; ONNX 125.99 ms (7.79 fps) | Pi 5, Bookworm, 2026 guide | MEASURED | https://docs.ultralytics.com/guides/raspberry-pi/ |
| Pi 4B power | idle 2.7 W; 4-core stress 6.4 W | pidramble bench, no USB | MEASURED | https://www.pidramble.com/wiki/benchmarks/power-consumption |
| Pi 5 power | off 1.7 W; idle 3.0 W (headless Wi-Fi) / 3.6 W (Eth+HDMI); 4-core stress 8.8 W; worst 15.9-16.8 W | cnx-software Nov 2023 | MEASURED | https://www.cnx-software.com/2023/11/05/raspberry-pi-5-review-raspberry-pi-os-bookworm-benchmarks-power-consumption/ |
| stress-ng perf governor | Pi 4 4.84 W; Pi 5 11.6 W | bret.dk | MEASURED | https://bret.dk/raspberry-pi-5-review/ |
| Pi 5 bare-board throttle | 85 C after 9 s | stress-ng, no cooler | MEASURED | https://bret.dk/raspberry-pi-5-review/ |
| Pi 5 with Active Cooler | ~66 C peak, no throttling, 28 C ambient | cpuminer | MEASURED | https://www.cnx-software.com/2023/11/05/raspberry-pi-5-review-raspberry-pi-os-bookworm-benchmarks-power-consumption/ |
| Pi 5 PSU | 5 V/5 A (25 W), or 5 V/3 A with 600 mA USB peripheral limit | official docs | VENDOR | https://www.raspberrypi.com/documentation/computers/raspberry-pi.html |
| Pi 5 hardware codecs | "4Kp60 HEVC decoder" only | product brief | VENDOR | https://pip-assets.raspberrypi.com/categories/892-raspberry-pi-5/documents/RP-008348-DS-6-raspberry-pi-5-product-brief.pdf |
| Pi 4 hardware codecs | H.265 4Kp60 decode; H.264 1080p60 decode, 1080p30 encode | spec page | VENDOR | https://www.raspberrypi.com/products/raspberry-pi-4-model-b/specifications/ |
| Pi 5 software H.264 | "uses software video encoders"; `--low-latency` "will still easily achieve 1080p30" | rpicam-vid docs | VENDOR | https://www.raspberrypi.com/documentation/computers/camera_software.html |
| picamera2 encoder select | `_hw_encoder_available = get_platform() == Platform.VC4`; else LibavH264Encoder / LibavMjpegEncoder | source | MEASURED | https://raw.githubusercontent.com/raspberrypi/picamera2/main/picamera2/encoders/__init__.py |
| Pi 5 camera cable | Standard-Mini 22-way, 200/300/500 mm | for Pi 5 and Zero | VENDOR | https://www.raspberrypi.com/products/camera-cable/ |
| Camera Module 3 | from $25, IMX708 12 MP PDAF, production to Jan 2030 | product page | VENDOR | https://www.raspberrypi.com/products/camera-module-3/ |
| AI HAT+ | 13 TOPS (Hailo-8L) / 26 TOPS (Hailo-8), "starting at $70", Pi 5 | product page | VENDOR | https://www.raspberrypi.com/products/ai-hat/ |
| AI HAT+ 2 | Hailo-10H, 40 TOPS INT4, 8 GB, $130 (15 Jan 2026) -> $200 now | product page + launch post | VENDOR | https://www.raspberrypi.com/products/ai-hat-plus-2/ |
| AI HAT+ 2 vs CPU | Qwen2.5-1.5B int4, 96 prefill tokens: Pi 5 CPU 2039 ms vs Hailo-10H 320 ms | Raspberry Pi blog 25 Feb 2026 | MEASURED | https://www.raspberrypi.com/news/when-and-why-you-might-need-the-raspberry-pi-ai-hat-plus-2/ |
| Hailo OS requirement | Pi 5 with 64-bit Raspberry Pi OS Trixie; `hailo-all` vs `hailo-h10-all` cannot co-exist | docs | VENDOR | https://www.raspberrypi.com/documentation/computers/ai.html |
| AI Camera | IMX500, $70, "Raspberry Pi 4 Model B or Raspberry Pi 5", `apt install imx500-all` | product + docs | VENDOR | https://www.raspberrypi.com/documentation/accessories/ai-camera.html |
| Jetson Orin Nano Super | 67 INT8 TOPS, 8 GB LPDDR5 102 GB/s, 7-25 W | NVIDIA | VENDOR | https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/nano-super-developer-kit/ |
| Jetson MSRP | "$249, down from $499" | 17 Dec 2024 | VENDOR | https://blogs.nvidia.com/blog/jetson-generative-ai-supercomputer/ |
| Jetson street | $399.00 "Backorder" (SparkFun); EUR 348 ex VAT (OKdo) | 2026-09-07 | MEASURED | https://www.sparkfun.com/products/22098 |
| JetPack 6.2.1 | Jetson Linux 36.4.4, CUDA 12.6.10, TensorRT 10.3, cuDNN 9.3 | 26 Jun 2025 | VENDOR | https://docs.nvidia.com/jetson/jetpack/release-notes/index.html |
| JetPack 7.2.1 | Jetson Linux 39.2.1, Ubuntu 24.04, kernel 6.8, CUDA 13.0, Orin family, Super Mode default for Orin Nano dev kit | 11 Aug 2026 | VENDOR | https://developer.nvidia.com/embedded/jetson-linux |
| Orin Nano Super LLM/VLM | Llama 3.2 3B 43.07 tok/s; Qwen2.5 7B 21.75 tok/s; Gemma 2 2B 34.97 tok/s; Qwen2-VL 2B 4.4 fps; SmolVLM 2B 12.9 fps | NVIDIA Jetson AI Lab | MEASURED | https://www.jetson-ai-lab.com/archive/benchmarks.html |
| Orin Nano Super YOLO26n | TensorRT FP16 4.57 ms; INT8 3.80 ms | imgsz 640, Ultralytics 8.4.33 | MEASURED | https://docs.ultralytics.com/guides/nvidia-jetson/ |
| RK3588 NPU | yolov8n INT8 640 73.5 fps; yolo11n 60.0 fps; whisper_base RTF 0.215 | single NPU core, rknn_model_zoo 2.3.2 | VENDOR | https://github.com/airockchip/rknn_model_zoo |
| Rockchip toolchains | rknn-toolkit2 v2.3.2; rknn-llm v1.3.0 (Qwen3/3.5, Gemma 3/4, Qwen3-VL, SmolVLM) | READMEs | VENDOR | https://github.com/airockchip/rknn-llm |
| Radxa X4 (N100) | idle 6 W (fan), load 12 W; Speedometer 175 vs Pi 5 56; 7-zip 8,120 MIPS (6 W PL1 throttle); 97 C peak | Ubuntu 24.04, cnx-software Sept 2024 | MEASURED | https://www.cnx-software.com/2024/09/29/radxa-x4-review-an-intel-n100-alternative-to-raspberry-pi-5-tested-with-ubuntu-24-04/ |
| Radxa X4 power input | USB-C PD 12 V, >= 2.5 A | docs | VENDOR | https://docs.radxa.com/en/x/x4 |
| ROS 2 Jazzy / Kilted tiers | Ubuntu 24.04 amd64 Tier 1, arm64 Tier 1; Debian Bookworm Tier 3 | REP 2000 | VENDOR | https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-2000.rst |
| TurtleBot 4 host | "Raspberry Pi 4B 4GB", RPLIDAR A1M8; Nav2/localization launched on robot | Clearpath docs | VENDOR | https://turtlebot.github.io/turtlebot4-user-manual/overview/features.html |
| LeKiwi host | "Raspberry Pi 5 (4GB)" in BOM; "can be any PC that can run on 5V" with 2+ USB | LeRobot docs | VENDOR | https://raw.githubusercontent.com/SIGRobotics-UIUC/LeKiwi/main/BOM.md |
| openWakeWord | "a single core of a Raspberry Pi 3 can run 15-20 openWakeWord models simultaneously" | v0.6.0 README | VENDOR | https://github.com/dscripka/openWakeWord |
| Piper on Pi 4 | 2 s audio per 1 s CPU (2023); 1.6 s per 1 s with medium voices (2026 docs) | Home Assistant | MEASURED | https://www.home-assistant.io/voice_control/voice_remote_local_assistant/ |
| Whisper on Pi 4 | ~7 s (2023) / ~8 s (2026 docs) per voice command | Home Assistant | MEASURED | https://www.home-assistant.io/voice_control/voice_remote_local_assistant/ |
| faster-whisper on Pi 5 | tiny-int8, 4 threads, 3.2 s command: 2.20 s latency | wyoming-faster-whisper README | MEASURED | https://github.com/rhasspy/wyoming-faster-whisper |
| whisper.cpp on Pi 4 | tiny encode 13,839 ms; base 30,552 ms (4 threads, commit fcf515d) | bench thread | MEASURED | https://github.com/ggml-org/whisper.cpp/issues/89 |

## Corrections to the brief

- "Host is a Raspberry Pi 4, 4 GB, already owned." Holds; the 2026 price rises strengthen it. Replacement cost is now $100 (Pi 4 4 GB) vs $110 (Pi 5 4 GB); if it ever needs replacing, buy the Pi 5.
- "Four cores, so it fits with headroom." Holds if STT stays Vosk or moves to the box. Whisper-class STT on the Pi 4 measures 7-8 s per command and breaks the 2.5-4.5 s turn budget.
- "Vosk ~1 core, final text ~0.3 s." No measured source found; keep [INFERRED] and measure at gate 3.
- "Piper near real time [INFERRED]." Now [MEASURED]: 1.6-2x faster than real time on a Pi 4 with medium voices (Home Assistant).
- "openWakeWord ~5-10% of a core." Holds; the README's Pi 3 figure implies a few percent of a Pi 4 core per model.
- "YOLO-nano on the Pi 4 CPU is 2-5 fps." Holds at the low end: YOLOv8n NCNN 640 measures 2.4 fps on a Pi 4, 10.6 fps on a Pi 5 (15 fps with YOLO26n). Input 320 would roughly double both [INFERRED].
- "640x480 JPEG at 10 fps ~20-40% of a core." Plausible; picamera2 JPEG is software on both boards, 2-3x cheaper on a Pi 5.
- "5 V/5 A buck (Pi) $20." Right for either board. On a Pi 5 a non-PD supply caps USB at 600 mA until `usb_max_current_enable=1` is set.
- "Pi Cam 3 $25." Holds ("from $25"); a Pi 5 also needs the Standard-Mini cable.
- Premise "Pi 5 has no hardware JPEG/H.264 encoder." Confirmed: docs say software H.264; the brief lists only an HEVC decoder; picamera2 uses LibavMjpegEncoder off VC4. Irrelevant at 640x480/10 fps.
- Premise "Pi 5 CPU speedup 2-3x." Holds (2x 7-zip, 3x Geekbench 6, 4.4x YOLOv8n NCNN).
- Premise "Jetson Orin Nano Super $249." MSRP unchanged on NVIDIA's blog; street on 2026-09-07 is $399 backordered (SparkFun), EUR 348 ex VAT (OKdo). Budget $350-400.
- Premise "Pi 5 16 GB." Now $305 list, not $120. No use here.
- "RPLIDAR C1 + Nav2 on the box." Fine. On-robot Nav2 is proven on a Pi 4 4 GB (TurtleBot 4), on Ubuntu (Tier 1), not Raspberry Pi OS (Debian, Tier 3).
- "Merge as LeKiwi with the Pi as host." LeRobot's BOM names a Pi 5 4 GB; docs accept any 5 V PC with 2+ USB ports. The Pi 4 works; the Pi 5 matches the reference.

## Alternatives considered and rejected

- Raspberry Pi 5 4 GB now ($110 + $5 cooler + $2 cable). Rejected for the first build: no measured load needs it, it costs more power and needs active cooling, and its price is at a vendor-acknowledged temporary peak. It is the designated upgrade when a trigger fires.
- Raspberry Pi 5 8 GB / 16 GB ($175 / $305). Rejected: nothing in the stack needs more than 4 GB; 8 GB only if Nav2 + SLAM + speech all move on-robot.
- Pi 5 + AI HAT+ (13/26 TOPS, from $70) or AI HAT+ 2 ($200). Rejected now: the box already does inference; the HAT+ 2 accelerates 1-3B local models, not the 27B. Revisit if you want a local fallback when Wi-Fi drops, or on-robot detection off the CPU. Requires Pi 5 and Raspberry Pi OS Trixie.
- Raspberry Pi AI Camera ($70) on the Pi 4. Not rejected, deferred: it is the cheapest way to add on-robot person detection without changing the host, because inference runs on the IMX500 and the Pi 4 is a documented target. Adopt it at build step 5 if `follow_person` should not depend on the phone page.
- Jetson Orin Nano Super Dev Kit ($249 MSRP, $399 street, backordered). Rejected: 3.6x a Pi 5 4 GB, 7-25 W plus fan, 19 V supply, no libcamera stack, and it duplicates the box's job. Right only if the robot must be autonomous without the box.
- RK3588 board (Orange Pi 5 / Rock 5B). Rejected: mature NPU toolchain now, but vendor kernels, per-model RKNN conversion, no picamera2/rpicam ecosystem, same LPDDR price hit. Only for on-robot Whisper + YOLO at less than Jetson money.
- N100/N150 mini PC or Radxa X4. Rejected: 6 W idle / 12 W load, 12 V PD, throttling at the default 6 W PL1, no CSI camera, no ROS 2 tier advantage over Ubuntu arm64.
- Replacing a dead Pi 4 with another Pi 4 ($100). Rejected: $10 more buys the Pi 5.

## Open questions

- Vosk on Pi 4: no measured CPU or end-of-speech latency source found; gate 3 must log it.
- `usb_max_current_enable`: the fetched doc excerpts did not include the paragraph; confirm on the config.txt documentation page.
- AI HAT+ 2 price path: $130 launch plus the documented $50 rise is $180, yet the product page says $200; the extra $20 step is unconfirmed.
- Jetson MSRP in 2026: only reseller prices ($399 SparkFun, EUR 348 OKdo) confirmed; NVIDIA's store page timed out.
- Camera Module 3 (IMX708) on Jetson: native support unverified; assume a third-party driver.
- Pi 5 power under this rover's actual load (Wi-Fi + USB mic + camera stream) was not measured by any source; 3.0-3.6 W idle and 8.8 W all-core bracket it.
- If Raspberry Pi rolls prices back as promised and a Pi 5 4 GB returns to $60-70, the Pi 5 becomes the default host.

## Sources

- Raspberry Pi 5 product brief (RP-008348-DS-6, published April 2026) - https://pip-assets.raspberrypi.com/categories/892-raspberry-pi-5/documents/RP-008348-DS-6-raspberry-pi-5-product-brief.pdf - 2026-04
- Raspberry Pi 5 product page - https://www.raspberrypi.com/products/raspberry-pi-5/ - accessed 2026-09-07
- Raspberry Pi 4 Model B specifications - https://www.raspberrypi.com/products/raspberry-pi-4-model-b/specifications/ - accessed 2026-09-07
- Raspberry Pi 4 Model B product page - https://www.raspberrypi.com/products/raspberry-pi-4-model-b/ - accessed 2026-09-07
- "1GB Raspberry Pi 5 now available at $45, and memory-driven price rises" - https://www.raspberrypi.com/news/1gb-raspberry-pi-5-now-available-at-45-and-memory-driven-price-rises/ - 2025-12-01
- "More memory-driven price rises" - https://www.raspberrypi.com/news/more-memory-driven-price-rises/ - 2026-02-02
- "A new 3GB Raspberry Pi 4 for $83.75, and more memory-driven price increases" - https://www.raspberrypi.com/news/a-new-3gb-raspberry-pi-4-for-83-75-and-more-memory-driven-price-increases/ - 2026-04-01
- "Introducing Raspberry Pi 5" - https://www.raspberrypi.com/news/introducing-raspberry-pi-5/ - 2023-09-28
- "Introducing the Raspberry Pi AI HAT+ 2: Generative AI on Raspberry Pi 5" - https://www.raspberrypi.com/news/introducing-the-raspberry-pi-ai-hat-plus-2-generative-ai-on-raspberry-pi-5/ - 2026-01-15
- "When and why you might need the Raspberry Pi AI HAT+ 2" - https://www.raspberrypi.com/news/when-and-why-you-might-need-the-raspberry-pi-ai-hat-plus-2/ - 2026-02-25
- Raspberry Pi AI HAT+ 2 product page - https://www.raspberrypi.com/products/ai-hat-plus-2/ - accessed 2026-09-07
- Raspberry Pi AI HAT+ product page - https://www.raspberrypi.com/products/ai-hat/ - accessed 2026-09-07
- Raspberry Pi AI Camera product page - https://www.raspberrypi.com/products/ai-camera/ - accessed 2026-09-07
- Raspberry Pi AI Camera documentation - https://www.raspberrypi.com/documentation/accessories/ai-camera.html - accessed 2026-09-07
- Raspberry Pi AI documentation (AI Kit / AI HAT+ / AI HAT+ 2) - https://www.raspberrypi.com/documentation/computers/ai.html - accessed 2026-09-07
- Raspberry Pi camera software documentation (rpicam-apps) - https://www.raspberrypi.com/documentation/computers/camera_software.html - accessed 2026-09-07
- Raspberry Pi hardware documentation - https://www.raspberrypi.com/documentation/computers/raspberry-pi.html - accessed 2026-09-07
- Raspberry Pi getting started documentation - https://www.raspberrypi.com/documentation/computers/getting-started.html - accessed 2026-09-07
- Raspberry Pi camera cable (Standard-Mini) - https://www.raspberrypi.com/products/camera-cable/ - accessed 2026-09-07
- Raspberry Pi Camera Module 3 - https://www.raspberrypi.com/products/camera-module-3/ - accessed 2026-09-07
- Raspberry Pi Active Cooler - https://www.raspberrypi.com/products/active-cooler/ - accessed 2026-09-07
- picamera2 encoders/__init__.py - https://raw.githubusercontent.com/raspberrypi/picamera2/main/picamera2/encoders/__init__.py - accessed 2026-09-07
- picamera2 encoders/mjpeg_encoder.py - https://raw.githubusercontent.com/raspberrypi/picamera2/main/picamera2/encoders/mjpeg_encoder.py - accessed 2026-09-07
- picamera2 encoders/h264_encoder.py - https://raw.githubusercontent.com/raspberrypi/picamera2/main/picamera2/encoders/h264_encoder.py - accessed 2026-09-07
- bret.dk Raspberry Pi 5 review - https://bret.dk/raspberry-pi-5-review/ - 2023-11-18
- cnx-software Raspberry Pi 5 review part 2 (benchmarks, power) - https://www.cnx-software.com/2023/11/05/raspberry-pi-5-review-raspberry-pi-os-bookworm-benchmarks-power-consumption/ - 2023-11-05
- pidramble power consumption benchmarks - https://www.pidramble.com/wiki/benchmarks/power-consumption - accessed 2026-09-07
- Ultralytics Raspberry Pi guide (YOLO26) - https://docs.ultralytics.com/guides/raspberry-pi/ - accessed 2026-09-07
- Ultralytics Raspberry Pi guide at v8.3.0 (YOLOv8n Pi 4 vs Pi 5) - https://raw.githubusercontent.com/ultralytics/ultralytics/v8.3.0/docs/en/guides/raspberry-pi.md - 2024-09
- Ultralytics NVIDIA Jetson guide - https://docs.ultralytics.com/guides/nvidia-jetson/ - accessed 2026-09-07
- NVIDIA Jetson Orin Nano Super Developer Kit - https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/nano-super-developer-kit/ - accessed 2026-09-07
- NVIDIA blog: Jetson Orin Nano Super announcement - https://blogs.nvidia.com/blog/jetson-generative-ai-supercomputer/ - 2024-12-17
- NVIDIA JetPack SDK page - https://developer.nvidia.com/embedded/jetpack - accessed 2026-09-07
- NVIDIA Jetson Linux page (39.2.1 / JetPack 7.2.1) - https://developer.nvidia.com/embedded/jetson-linux - 2026-08-11
- JetPack 6.2.1 release notes - https://docs.nvidia.com/jetson/jetpack/release-notes/index.html - 2025-06-26
- Jetson AI Lab benchmarks (archive) - https://www.jetson-ai-lab.com/archive/benchmarks.html - accessed 2026-09-07
- SparkFun Jetson Orin Nano Super Developer Kit listing - https://www.sparkfun.com/products/22098 - accessed 2026-09-07
- OKdo Jetson Orin Nano Super Developer Kit listing - https://www.okdo.com/p/nvidia-jetson-orin-nano-super-developer-kit/ - accessed 2026-09-07
- rknn-toolkit2 - https://github.com/airockchip/rknn-toolkit2 - v2.3.2, accessed 2026-09-07
- rknn-llm - https://github.com/airockchip/rknn-llm - v1.3.0, accessed 2026-09-07
- rknn_model_zoo - https://github.com/airockchip/rknn_model_zoo - 2.3.2, accessed 2026-09-07
- Radxa X4 documentation - https://docs.radxa.com/en/x/x4 - accessed 2026-09-07
- cnx-software Radxa X4 review part 2 (Ubuntu 24.04, power) - https://www.cnx-software.com/2024/09/29/radxa-x4-review-an-intel-n100-alternative-to-raspberry-pi-5-tested-with-ubuntu-24-04/ - 2024-09-29
- cnx-software Radxa X4 review part 1 - https://www.cnx-software.com/2024/08/19/radxa-x4-sbc-kit-review-unboxing-case-assembly-ubuntu-24-04-installation/ - 2024-08-19
- REP 2000 (ROS 2 target platforms) - https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-2000.rst - accessed 2026-09-07
- TurtleBot 4 features - https://turtlebot.github.io/turtlebot4-user-manual/overview/features.html - accessed 2026-09-07
- TurtleBot 4 navigation tutorial - https://turtlebot.github.io/turtlebot4-user-manual/tutorials/navigation.html - accessed 2026-09-07
- LeRobot LeKiwi docs - https://huggingface.co/docs/lerobot/lekiwi - accessed 2026-09-07
- LeKiwi BOM - https://raw.githubusercontent.com/SIGRobotics-UIUC/LeKiwi/main/BOM.md - accessed 2026-09-07
- openWakeWord README - https://github.com/dscripka/openWakeWord - v0.6.0 (2024-02-11)
- Piper README v1.2.0 - https://raw.githubusercontent.com/rhasspy/piper/v1.2.0/README.md - 2023
- Piper (piper1-gpl) README - https://raw.githubusercontent.com/OHF-Voice/piper1-gpl/main/README.md - accessed 2026-09-07
- Home Assistant "Year of the Voice - Chapter 2" - https://www.home-assistant.io/blog/2023/04/27/year-of-the-voice-chapter-2/ - 2023-04-27
- Home Assistant local assistant docs - https://www.home-assistant.io/voice_control/voice_remote_local_assistant/ - accessed 2026-09-07
- wyoming-faster-whisper README - https://github.com/rhasspy/wyoming-faster-whisper - accessed 2026-09-07
- whisper.cpp benchmark thread - https://github.com/ggml-org/whisper.cpp/issues/89 - accessed 2026-09-07
- Vosk site - https://alphacephei.com/vosk/ - accessed 2026-09-07
- hailo-apps - https://github.com/hailo-ai/hailo-apps - accessed 2026-09-07
- hailo-rpi5-examples (marked outdated) - https://github.com/hailo-ai/hailo-rpi5-examples - accessed 2026-09-07
- Adafruit Raspberry Pi 5 4 GB - https://www.adafruit.com/product/5812 - accessed 2026-09-07
- Adafruit Raspberry Pi 5 8 GB - https://www.adafruit.com/product/5813 - accessed 2026-09-07
- Adafruit Raspberry Pi 4 4 GB - https://www.adafruit.com/product/4296 - accessed 2026-09-07
- PiShop Raspberry Pi 5 4 GB - https://www.pishop.us/product/raspberry-pi-5-4gb/ - accessed 2026-09-07
- PiShop Raspberry Pi 5 8 GB - https://www.pishop.us/product/raspberry-pi-5-8gb/ - accessed 2026-09-07
- PiShop Raspberry Pi 4 Model B 4 GB - https://www.pishop.us/product/raspberry-pi-4-model-b-4gb/ - accessed 2026-09-07
