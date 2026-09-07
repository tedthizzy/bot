# Android phone as a removable web-only peripheral (face, stills, browser person detection)

Research note, 2026-09-07. Scope: the "Phone, later, as a peripheral" section of the brief. Method note: the session's web-search budget was exhausted before this task started, so every source below was opened directly with WebFetch (official docs, npm/jsDelivr registries, GitHub, vendor spec pages) plus three PDFs extracted locally with `pdftotext`. Where I could not reach a source the number is tagged [INFERRED] and said so.

## Summary

- The web-only design holds in 2026. Chrome for Android is at stable 152 (caniuse, September 2026). Screen Wake Lock is Baseline 2025 and shipped in Chrome for Android; `getUserMedia({video:{facingMode:{exact:"environment"}}})` selects the rear camera; `ImageCapture.takePhoto()` (Chrome 59+, Android included) returns a full-resolution still as a Blob; a manifest with `display: "fullscreen"` gives an installed-PWA kiosk face with no browser chrome.
- The brief's biggest omission is the secure-context requirement. `getUserMedia`, `wakeLock` and WebGPU all refuse to run on `http://pi.local`. Pick one of two paths: (a) Tailscale on the Pi and phone, `tailscale cert` / `tailscale serve` for a Let's Encrypt cert on the Pi's `*.ts.net` name (nothing to install on the phone); or (b) mkcert v1.4.4 root CA installed on the phone as a user CA plus a cert for the Pi's LAN IP and `.local` name. `chrome://flags/#unsafely-treat-insecure-origin-as-secure` works for bench testing only.
- Browser person detection: `@mediapipe/tasks-vision` 1.0.1 (npm `latest`; nightly RCs dated 2026-09-06) with `baseOptions.delegate: "GPU"`. Google's own native TFLite benchmark for EfficientDet-Lite0 is 28–29 ms per frame on a Pixel 6; in-browser on a 2024–2025 Snapdragon phone expect roughly 15–30 fps [INFERRED]. Run it at 5 Hz, not frame rate, to hold thermals and because the Pi's `follow_person` validator does not need more.
- The brief should change "getUserMedia → JPEG POST" to "getUserMedia for preview/detection, `ImageCapture.takePhoto()` for stills". Canvas grabs of the video track are capped at the video resolution; `takePhoto()` reconfigures the camera for a still.
- Transport: one WebSocket for ≤5 Hz person-position JSON (advisory only, token-bucketed on the Pi) and HTTP POST multipart for stills. WebRTC is not needed.
- OxygenOS: a foreground, screen-on, charging PWA does not hit doze; the real failure modes are screen timeout, "Sleep standby optimization", and the page being backgrounded (Wake Lock and camera both drop). This is exactly why the phone stays out of the stop/TTL path; the Pi must behave identically with the phone gone.
- Strong alternative: Android 14 QPR1+ "device as webcam" makes the phone a standard UVC camera (MJPEG 1080p30) into the Pi's USB port with no app. It gives the Pi a better camera but moves detection back onto the Pi CPU (2–5 fps YOLO-nano). Choose it if the goal is only better stills.
- Mount and power: a OnePlus 12/13 weighs 210–220 g; charge it from its own 5 V/3 A buck or a 12 V→USB-PD module, not from the Pi's 5 V/5 A buck (the Pi 4 product brief requires a 3 A minimum supply for the Pi alone).

## State of the art (2026)

### Web APIs on Chrome for Android

**Version baseline.** caniuse lists Chrome for Android 152 as current (September 2026) and desktop Chrome 153–155 in preview. Chrome's four-week cadence means "Chrome 152" is the floor to design against; nothing below needs a newer build.

**Screen Wake Lock.** MDN marks the API "Baseline 2025, newly available since March 2025" (Firefox 126 was the last major engine). It is secure-context only. The lock is released automatically whenever the document becomes hidden or the OS enters power-save/low-battery, so the page must re-request on `visibilitychange`. Permissions-Policy directive is `screen-wake-lock`, default `self`. Chrome desktop shipped it in 85; Chrome for Android has it through 152; Safari iOS 16.4+.

**getUserMedia and the rear camera.** `facingMode: { exact: "environment" }` is a mandatory constraint; the request fails if no rear camera is exposed. Use `ideal` for width/height (for example `{width:{ideal:1280}, height:{ideal:720}}`) so the phone can pick the nearest supported preview mode; the preview stream is what the detector consumes. MDN documents no Android-specific quirks, but every multi-camera phone exposes only its primary rear module to `facingMode`; the telephoto/ultra-wide are not selectable by the web page [INFERRED, consistent with Android's Camera2 logical-camera design].

**ImageCapture.takePhoto().** Chrome's own article states `takePhoto()` "interrupts the MediaStream, reconfigures the camera, takes the photo" at the camera's still-image resolution, while `grabFrame()` (and any canvas draw of the `<video>`) is limited to the video resolution. `getPhotoCapabilities()` returns `imageWidth`/`imageHeight` as `{min,max,step}`; request `{imageWidth: caps.imageWidth.max}`. Shipped in Chrome 59 on Android and desktop; caniuse: Chrome for Android yes, Samsung Internet 5+, Firefox only behind a flag (takePhoto only), Safari iOS none. This is a Chrome-family-only API; that is fine for a single dedicated Android phone. Two practical consequences: pause the detector loop while a still is in flight (the stream stalls), and expect the still to take 0.3–1 s [INFERRED].

**Secure contexts.** Chromium's policy doc lists getUserMedia among the features restricted to secure origins. `http://localhost` is secure; a LAN IP or `pi.local` is not. Options:

1. **Tailscale HTTPS.** Enable MagicDNS + HTTPS in the admin console, then `tailscale cert <node>.<tailnet>.ts.net` or simply `tailscale serve 8000` to front the Pi's web app; certificates come from Let's Encrypt via DNS-01, private keys stay on the Pi. Caveats from the KB: 90-day expiry (Serve renews automatically; `tailscale cert` files do not), machine names appear in the public CT log, and repeated requests can hit Let's Encrypt rate limits with a 34-hour lockout. The phone needs the Tailscale app running, and all traffic stays on the LAN when both peers are on the same Wi-Fi (direct WireGuard path).
2. **mkcert.** `mkcert 192.168.x.x pi.local` produces a leaf for both names; copy `rootCA.pem` (from `mkcert -CAROOT`) to the phone and install it under Settings → Security → Encryption & credentials → Install a certificate → CA certificate. Chrome's certificate verifier "considers local trust decisions for adding trust" over TCP/TLS (Chrome Root Store FAQ; the Chrome Root Store has been default on Android since Chrome 115), so a user-installed CA is honoured by the browser even though Android apps targeting API 24+ ignore user CAs by default (Android network-security-config doc). Android shows a persistent "network may be monitored" warning while a user CA is installed. Latest mkcert is v1.4.4; the project is stable but slow-moving.
3. **`chrome://flags/#unsafely-treat-insecure-origin-as-secure`** with `http://192.168.x.x:8000` typed in. Chromium notes that the command-line form needs root on Android; the flags-UI form does not. Use it for the first bench test, not for the build, because it is per-profile and silently lost on a Chrome reset.

**PWA install and kiosk.** Manifest `display: "fullscreen"` hides all browser UI on Android Chrome; fallback chain is fullscreen → standalone → minimal-ui → browser, and `display_override` lets you list preferences. Installability needs `name`/`short_name`, 192×192 and 512×512 icons, and a `start_url` (web.dev). Manifest fullscreen is distinct from the Fullscreen API. Pair it with Developer options → "Stay awake" (screen never sleeps while charging) as a belt to the Wake Lock's braces. Web Serial/WebUSB are not needed and are not in the design.

### Browser inference

**MediaPipe Tasks Vision.** The package is `@mediapipe/tasks-vision`, current `latest` = 1.0.1 on npm, with nightly pre-releases `1.0.1-rc.20260902` … `1.0.1-rc.20260906` on jsDelivr, so the web runtime is actively maintained in September 2026. The README's privacy notice is dated 2026-06-05. It ships 11 vision tasks including Object Detector and Pose Landmarker. `BaseOptions.delegate` accepts `CPU` (default) or `GPU`. Google's docs do not say which graphics API the web GPU delegate uses; the web runtime is WebGL-based rather than WebGPU as far as I can tell [INFERRED; the docs I opened do not state it]. The official web sample loads WASM from `https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@latest/wasm` and the model from `storage.googleapis.com`; for the rover, pin `@1.0.1`, and self-host the `wasm/` folder and the `.tflite` on the Pi so the phone page works with no internet.

Model choices and Google's published latencies (native TFLite on a Pixel 6, not browser):

- EfficientDet-Lite0, 320×320: int8 CPU 29.31 ms; float16 GPU 27.97 ms; float32 GPU 27.83 ms.
- EfficientDet-Lite2, 448×448: int8 CPU 70.91 ms; float32 GPU 41.15 ms.
- SSD MobileNetV2, 256×256: float32 GPU 24.01 ms, CPU 36.30 ms.
- Pose Landmarker (BlazePose GHUM 3D model card, Pixel 3): Lite ~44 fps CPU / ~49 fps GPU; Full ~18 / ~40; Heavy ~4 / ~19. The 2026 pose_landmarker pages no longer carry a latency table.

The web runtime adds WASM↔GPU copies and JS scheduling; on a Snapdragon 8 Gen 3 / 8 Elite phone a realistic in-browser figure is 15–30 fps for EfficientDet-Lite0 on GPU and similar for Pose Landmarker Lite [INFERRED; no 2025–2026 measured browser numbers on named Android phones were reachable this session]. For `follow_person` the useful output is a bounding box centre and height (a range proxy) at a few Hz. Object Detector with `categoryAllowlist: ["person"]` is simpler and more robust to partial bodies than Pose Landmarker; Pose Landmarker is worth it only if you want gesture commands later.

**Alternatives.** ONNX Runtime Web's WebGPU EP (`executionProviders:['webgpu']`) is "available out-of-box" on Chrome Android; WebGPU shipped on Android in Chrome 121 for Android 12+ with Qualcomm and ARM GPUs (`chrome://gpu` must read "WebGPU: Hardware accelerated"). transformers.js runs DETR/YOLOS with `device:'webgpu'`, 10–40× heavier than EfficientDet-Lite0. TF.js's WebGPU README still points at Chrome Canary and lists no benchmarks. None reach the phone's NPU; WebNN is not shipped on Android Chrome [INFERRED].

### OxygenOS and background behaviour

dontkillmyapp.com still ranks OnePlus third worst (after Huawei and Xiaomi) for killing background work, with reports of battery-optimization settings reverting and of "Deep optimization" / "Sleep standby optimization" cutting background apps. The OnePlus page has not been updated for OxygenOS 14–16 (OnePlus 12 ships OxygenOS 14/Android 14; OnePlus 13 ships OxygenOS 15/Android 15 per their spec pages), so treat those specifics as historical [VENDOR page, undated]. What matters for this design:

- A PWA that is foreground, screen-on and charging is not subject to doze or background kill. The moment it is not foreground (notification shade, incoming call, OxygenOS auto-lock, a second app), Chrome drops the Wake Lock and stops the camera track. The page must handle `visibilitychange` and re-acquire, and the Pi must treat silence as "no phone".
- Mitigations: Developer options → Stay awake; Settings → Battery → disable Sleep standby optimization and set the face PWA to "Don't optimize"; lock the PWA in Recents; disable adaptive brightness; turn off the always-on-display; keep the phone on the charger.
- These are user-side settings that OxygenOS updates have been known to reset. That is the operational argument, on top of the safety argument, for keeping the phone out of stop/TTL: the ESP32's 300 ms TTL and the Pi's validator do not learn the phone exists.

### Transport

- **Stills:** `fetch('/api/still', {method:'POST', body: formData})` with the `takePhoto()` Blob as `multipart/form-data`. A 12–50 MP JPEG is 3–12 MB; on a Wi-Fi 5 LAN that is 0.2–1 s [INFERRED]. Request the size the 27B can use: the brief sends 640×480 to the box, so ask `takePhoto({imageWidth: 1600})` for the "good still" and let the Pi downscale.
- **Person position:** a WebSocket (browser-native, no library) carrying `{seq, t_phone_ms, box:[x,y,w,h], conf}` at ≤5 Hz. The Pi accepts it only from the authenticated phone origin, drops messages older than 500 ms or out of sequence, and applies a token bucket (10 msg/s, burst 20) before anything reaches the validator; the validator still clamps `drive`/`turn` to the same limits as any other skill. This makes flooding impossible and keeps the phone a hint source, not a controller.
- **WebRTC** (aiortc on the Pi, VP8/H.264) streams video for Pi-side inference, which the Pi 4 cannot afford. Reject.

### UVC webcam alternative

AOSP "Device as webcam" (Android 14 QPR1+) exposes the phone as a standard UVC device on Linux hosts when the user selects Webcam under USB preferences (or `adb shell svc usb setFunctions uvc`). Recommended formats are MJPEG at 480p/720p/1080p 30 fps and YUYV up to 720p30 over USB 2.0; the phone runs a preview activity that lets the user pick front/back camera and zoom. It requires the OEM to set `ro.usb.uvc.enabled=true`; Pixels do, OnePlus support is unverified (open question). Third-party bridges are worse on a Pi: DroidCam 2.1.5's Linux client ships 64-bit x86 binaries only and must be built from source on ARM (v4l2loopback-dc, adb over USB or Wi-Fi, 640×480–1920×1080); Camo Studio hosts are macOS 12.3+ and Windows 10+ only, no Linux.

Pros: no web page, no HTTPS, no browser lifecycle; the Pi's existing V4L2 640×480 capture works unchanged with better optics. Cons: the phone's GPU idles, detection returns to the Pi CPU (2–5 fps YOLO-nano), stills are capped at the 1080p UVC stream, and the preview activity must stay foreground, so the OxygenOS screen-lock issues return.

### Mounting and power

OnePlus 13: 210–213 g, 162.9 × 76.5 × 8.5 mm, 6,000 mAh, USB 3.2 Gen 1 Type-C. OnePlus 12: 220 g, 5,400 mAh. Neither has built-in magnets; a Qi2/MagSafe-ring case plus a magnetic mount gives removability and repeatable camera alignment. Mount it above the Pi Cam 3 on the same forward axis, low and near the axle, since 220 g shifts a small chassis's centre of mass.

Power: the Pi 4 product brief requires a 5 V/3 A minimum supply for the Pi (2.5 A only if USB peripherals draw <500 mA). Screen-on, camera-on, GPU-inference phone draw is 3–6 W [INFERRED], and a phone on a plain 5 V USB-A port with a legacy A-to-C cable negotiates only USB default current (0.5 A) unless the port implements BC 1.2 (up to 1.5 A) [USB spec values from memory; not verified this session]. So charge the phone from its own 5 V/3 A buck with a BC 1.2-capable port, or a 12 V→USB-PD module (the OnePlus fast-charge path is proprietary SUPERVOOC; at PD it takes ~18 W). Do not hang it off the Pi's 5 V/5 A buck.

## Recommendation for this build

1. **Keep the phone as an optional, foreground-only web peripheral.** Nothing in the Pi or ESP32 code paths may reference it; the `follow_person` skill consumes a `person_hint` topic that times out after 1 s and degrades to "no target".
2. **Solve HTTPS first, with Tailscale.** Install Tailscale on the Pi and phone, enable MagicDNS + HTTPS, run `tailscale serve 8000` in front of the Pi's FastAPI/uvicorn app. No CA on the phone, valid Let's Encrypt cert, auto-renew. Fall back to mkcert only if you refuse a tailnet.
3. **Page structure.** One PWA (`display: "fullscreen"`, 192/512 px icons) with three modes: face (Wake Lock + `visibilitychange` re-acquire), stills (`ImageCapture.takePhoto({imageWidth: caps.imageWidth.max})` → POST), follow (`@mediapipe/tasks-vision@1.0.1`, ObjectDetector, `efficientdet_lite0` int8 or float16, `delegate:"GPU"`, `runningMode:"VIDEO"`, `categoryAllowlist:["person"]`, `scoreThreshold:0.5`, throttled to 5 Hz with `requestVideoFrameCallback` or a `setInterval`). Self-host wasm and model on the Pi.
4. **Phone settings checklist.** Developer options → Stay awake; Battery → Don't optimize for Chrome; disable Sleep standby optimization and auto-lock; lock the PWA in Recents; plugged in.
5. **Pi side.** Token bucket 10 msg/s, burst 20; drop stale/out-of-order frames; the validator clamps as for any skill; log phone hint age alongside observation-to-action age from build gate 3.
6. **Power and mount.** Separate 5 V/3 A buck (BC 1.2 DCP or a PD module) for the phone; Qi2-ring case and a magnetic mount above the Pi Cam.
7. **If you only want better stills,** skip all of the above and use Android 14 QPR1 UVC webcam mode if the OnePlus firmware exposes it; the Pi's existing V4L2 capture code works unchanged.

Would a 2026 practitioner make the brief's choice? Yes, with three edits: name the HTTPS path, use `ImageCapture.takePhoto()` for stills, and pin/self-host MediaPipe 1.0.x. The UVC option is the practitioner's alternative for "camera only".

## Numbers

| quantity | value | hardware/context | tag | source URL |
|---|---|---|---|---|
| Chrome for Android current stable | 152 | September 2026 (desktop 153–155 in preview) | VENDOR | https://caniuse.com/wake-lock |
| Screen Wake Lock support | Chrome 85+, Chrome Android through 152, Firefox 126+, Safari iOS 16.4+ | Baseline 2025 (March 2025) | VENDOR | https://caniuse.com/wake-lock ; https://developer.mozilla.org/en-US/docs/Web/API/Screen_Wake_Lock_API |
| ImageCapture support | Chrome 59+ (Android and desktop), Samsung Internet 5+, Firefox flag-only, Safari iOS none | 78.05% global usage | VENDOR | https://caniuse.com/imagecapture ; https://developer.chrome.com/blog/imagecapture |
| WebGPU on Android | Chrome 121+, Android 12+, Qualcomm and ARM GPUs | default-on since Jan 2024 | VENDOR | https://developer.chrome.com/blog/new-in-webgpu-121 |
| Chrome Root Store default on Android | Chrome 115 (rollout began 114) | local trust additions honoured over TLS | VENDOR | https://chromium.googlesource.com/chromium/src/+/main/net/data/ssl/chrome_root_store/faq.md |
| @mediapipe/tasks-vision latest | 1.0.1; nightlies 1.0.1-rc.20260902 … rc.20260906 | npm / jsDelivr registry, 2026-09-07 | VENDOR | https://registry.npmjs.org/@mediapipe/tasks-vision/latest ; https://data.jsdelivr.com/v1/packages/npm/@mediapipe/tasks-vision |
| EfficientDet-Lite0 latency | int8 CPU 29.31 ms; float16 GPU 27.97 ms; float32 GPU 27.83 ms | Pixel 6, native TFLite, 320×320 | VENDOR | https://developers.google.com/edge/mediapipe/solutions/vision/object_detector |
| EfficientDet-Lite2 latency | int8 CPU 70.91 ms; float32 GPU 41.15 ms | Pixel 6, native TFLite, 448×448 | VENDOR | same |
| SSD MobileNetV2 latency | float32 CPU 36.30 ms; GPU 24.01 ms | Pixel 6, native TFLite, 256×256 | VENDOR | same |
| BlazePose Lite/Full/Heavy | ~44/~18/~4 fps CPU (XNNPack); ~49/~40/~19 fps GPU | Pixel 3, native TFLite, 256×256 crop | VENDOR | https://storage.googleapis.com/mediapipe-assets/Model%20Card%20BlazePose%20GHUM%203D.pdf |
| In-browser EfficientDet-Lite0 GPU fps | 15–30 fps | Snapdragon 8 Gen 3 / 8 Elite phone, Chrome 152, WebGL delegate | INFERRED | — |
| Person-hint rate / Pi limit | 5 Hz send; token bucket 10 msg/s, burst 20; drop >500 ms old | design value | INFERRED | — |
| Android UVC webcam | Android 14 QPR1+; MJPEG 480p/720p/1080p @30; YUYV ≤720p30 over USB 2.0 | AOSP DeviceAsWebcam | VENDOR | https://source.android.com/docs/core/camera/webcam |
| DroidCam Linux client | 2.1.5; 64-bit binaries only, ARM from source; 640×480–1920×1080 | v4l2loopback-dc | VENDOR | https://www.dev47apps.com/droidcam/linux/ |
| Camo Studio hosts | macOS 12.3+, Windows 10+; no Linux | phone: iOS 15+, Android 7+ | VENDOR | https://camo.com/studio |
| OnePlus 13 | 210–213 g; 6,000 mAh; USB 3.2 Gen 1 Type-C; 80 W SUPERVOOC; OxygenOS 15 / Android 15 | spec page | VENDOR | https://www.oneplus.com/us/13/specs |
| OnePlus 12 | 220 g; 5,400 mAh; USB 3.2 Gen 1; OxygenOS 14 / Android 14 | spec page | VENDOR | https://www.oneplus.com/us/12/specs |
| Pi 4 supply | 5 V / 3 A minimum; 2.5 A only if USB peripherals <500 mA | product brief | VENDOR | https://pip-assets.raspberrypi.com/categories/545-raspberry-pi-4-model-b/documents/RP-008344-DS-5-raspberry-pi-4-product-brief.pdf |
| Phone draw, screen + camera + GPU inference | 3–6 W | 2024–2025 flagship | INFERRED | — |
| Legacy 5 V USB charge current | 0.5 A default; ≤1.5 A with BC 1.2 DCP | USB Type-C / BC 1.2 spec values, not re-verified this session | INFERRED | — |
| Tailscale certs | Let's Encrypt, 90-day expiry; rate-limit lockout ~34 h | `tailscale cert` / `tailscale serve` | VENDOR | https://tailscale.com/kb/1153/enabling-https |
| mkcert | v1.4.4 (latest release) | root CA install on Android via Settings | VENDOR | https://github.com/FiloSottile/mkcert/releases/latest |
| takePhoto() still latency | 0.3–1 s including camera reconfigure | Chrome Android, 12–50 MP sensor | INFERRED | — |
| dontkillmyapp OnePlus rank | 3rd worst (after Huawei, Xiaomi) | page undated, OxygenOS ≤11 evidence | VENDOR | https://dontkillmyapp.com/ |

## Corrections to the brief

1. **"on-demand stills from the good camera via `getUserMedia` → JPEG POST"** — partly wrong. `getUserMedia` plus canvas gives the preview resolution only. Use `new ImageCapture(track).takePhoto({imageWidth})` for a sensor-resolution still (Chrome 59+ on Android). Keep `getUserMedia` for the preview the detector reads. Expect the preview to stall during the still; pause detection.
2. **Missing: secure context.** None of Wake Lock, `getUserMedia`, `ImageCapture` or WebGPU runs on `http://pi.local`. Add a step: Tailscale HTTPS on the Pi (preferred) or mkcert root CA on the phone. The flags-UI insecure-origin override is a bench tool only.
3. **"Face page with Wake Lock"** — holds, but the lock is released on every visibility change and by low-battery/power-save mode. Re-request on `visibilitychange`, and enable Developer options → Stay awake as a second layer.
4. **"MediaPipe in the browser"** — holds. Be specific: `@mediapipe/tasks-vision@1.0.1` (not the legacy `@mediapipe/pose` solution), ObjectDetector with EfficientDet-Lite0 and `delegate:"GPU"`, self-hosted wasm and model on the Pi, throttled to ~5 Hz. Pin the version; the official sample uses `@latest`, which will break offline and on breaking releases.
5. **"OxygenOS OTG quirks and doze"** — the web design uses neither OTG nor background execution, so doze is not the threat. The threats are screen lock, Sleep standby optimization, and any foreground interruption; the mitigation is settings plus keeping the phone out of the stop path, which the brief already does. The UVC alternative does use USB, and there the OTG concern is real.
6. **"phone's hardware edge (NPU)"** — the browser cannot use the NPU. Browser inference is GPU (WebGL/WebGPU) or WASM CPU. The phone's edge for this design is its camera and its GPU, not the Hexagon NPU [INFERRED: no WebNN on Android Chrome as of this note].
7. **BOM** — add roughly $15–35 for the phone path: Qi2-ring case or adhesive ring, magnetic mount, a second 5 V/3 A buck or 12 V→PD module, a short USB-C cable. The brief's 5 V/5 A Pi buck should not also charge the phone.
8. **"rover must behave identically with the phone unplugged"** — holds and is the right rule. Make it testable: add to gate 4 "phone hint stream cut mid-follow → rover stops following within 1 s and continues to accept voice commands".

## Alternatives considered and rejected

- **WebRTC video to the Pi (aiortc):** moves inference to the Pi 4 CPU (2–5 fps) and adds Python VP8/H.264 decode. Rejected.
- **HTTP POST per detection frame:** 5 Hz × 50–100 KB JPEG with a TLS request each wastes phone battery and Pi CPU; a persistent WebSocket with ~100-byte JSON is cheaper. Rejected for positions; kept for stills.
- **Pose Landmarker as primary tracker:** heavier than the object detector for the same output and brittle with partial bodies. Keep as an upgrade for gestures.
- **transformers.js DETR/YOLOS, TF.js WebGPU:** an order of magnitude heavier, or documented against Chrome Canary. Rejected; MediaPipe Tasks is the maintained path for this model family.
- **Native Android app:** would unlock the NPU and CameraX but breaks the brief's "no native app". Rejected per brief.
- **Phone as UVC webcam:** not rejected; the alternative for "better camera only". Gives up browser detection and sensor-resolution stills; needs OnePlus UVC support (unverified).
- **DroidCam / Camo bridges:** DroidCam needs an ARM source build and adb; Camo has no Linux host. Rejected in favour of native UVC.
- **Self-signed leaf without a CA:** Chrome Android interstitials every session and does not reliably grant `getUserMedia` after "proceed anyway". Rejected; use mkcert's CA or Tailscale.

## Open questions

1. Does the user's OnePlus model (OxygenOS 14/15/16) ship `ro.usb.uvc.enabled=true` (Webcam option under USB preferences)? Not verifiable this session; check Settings → USB preferences with a cable to the Pi.
2. Measured in-browser fps, battery drain and skin temperature for `@mediapipe/tasks-vision@1.0.1` ObjectDetector on the actual phone at 5 Hz and at full frame rate. No 2025–2026 measured browser numbers on named Android phones were reachable; run a 30-minute test as part of the phone gate.
3. Does `ImageCapture.takePhoto()` on this phone return the full sensor resolution or a capped size? Read `getPhotoCapabilities().imageWidth.max` on the device; some Android Camera HALs cap the web still path.
4. Whether the MediaPipe web GPU delegate is WebGL2 or WebGPU in 1.0.x; the docs do not say. It changes nothing in code but matters for `chrome://gpu` troubleshooting.
5. Whether OxygenOS 15/16 still resets "Don't optimize" and "Sleep standby optimization" after updates; the dontkillmyapp evidence is from OxygenOS ≤11.
6. Exact USB Type-C/BC 1.2 current figures for the charging port design; I could not open a spec source this session and quoted the values from memory.

## Sources

- MDN, "Screen Wake Lock API", https://developer.mozilla.org/en-US/docs/Web/API/Screen_Wake_Lock_API (Baseline 2025 marker; opened 2026-09-07)
- caniuse, "Screen Wake Lock API", https://caniuse.com/wake-lock (opened 2026-09-07)
- caniuse, "ImageCapture API", https://caniuse.com/imagecapture (opened 2026-09-07)
- MDN, "ImageCapture: takePhoto()", https://developer.mozilla.org/en-US/docs/Web/API/ImageCapture/takePhoto (opened 2026-09-07)
- MDN, "ImageCapture: getPhotoCapabilities()", https://developer.mozilla.org/en-US/docs/Web/API/ImageCapture/getPhotoCapabilities (opened 2026-09-07)
- Chrome Developers, "Take photos and control camera settings (ImageCapture)", https://developer.chrome.com/blog/imagecapture (Chrome 59 era; opened 2026-09-07)
- MDN, "MediaTrackConstraints: facingMode", https://developer.mozilla.org/en-US/docs/Web/API/MediaTrackConstraints/facingMode (opened 2026-09-07)
- Chromium, "Deprecating Powerful Features on Insecure Origins", https://www.chromium.org/Home/chromium-security/deprecating-powerful-features-on-insecure-origins/ (opened 2026-09-07)
- Chromium, "Chrome Root Store FAQ", https://chromium.googlesource.com/chromium/src/+/main/net/data/ssl/chrome_root_store/faq.md (opened 2026-09-07)
- Android Developers, "Network security configuration", https://developer.android.com/privacy-and-security/security-config (opened 2026-09-07)
- FiloSottile/mkcert README and releases, https://github.com/FiloSottile/mkcert ; https://github.com/FiloSottile/mkcert/releases/latest (v1.4.4)
- Tailscale KB, "Enabling HTTPS", https://tailscale.com/kb/1153/enabling-https ; "Tailscale Serve", https://tailscale.com/kb/1312/serve (opened 2026-09-07)
- MDN, "Web app manifest: display", https://developer.mozilla.org/en-US/docs/Web/Progressive_web_apps/Manifest/Reference/display (opened 2026-09-07)
- web.dev, "Add a web app manifest", https://web.dev/articles/add-manifest (opened 2026-09-07)
- Chrome Developers, "What's New in WebGPU (Chrome 121)", https://developer.chrome.com/blog/new-in-webgpu-121 (January 2024)
- Chrome Developers, "WebGPU troubleshooting tips", https://developer.chrome.com/docs/web-platform/webgpu/troubleshooting-tips (opened 2026-09-07)
- Google AI Edge, "Object detection task guide" and "for Web", https://developers.google.com/edge/mediapipe/solutions/vision/object_detector ; https://developers.google.com/edge/mediapipe/solutions/vision/object_detector/web_js (opened 2026-09-07)
- Google AI Edge, "Pose landmark detection guide" and "for Web", https://developers.google.com/edge/mediapipe/solutions/vision/pose_landmarker ; https://developers.google.com/edge/mediapipe/solutions/vision/pose_landmarker/web_js (opened 2026-09-07)
- Google AI Edge, "MediaPipe Tasks for Web setup", https://developers.google.com/edge/mediapipe/solutions/setup_web (opened 2026-09-07)
- npm registry, @mediapipe/tasks-vision latest, https://registry.npmjs.org/@mediapipe/tasks-vision/latest (1.0.1; opened 2026-09-07)
- jsDelivr data API, @mediapipe/tasks-vision versions, https://data.jsdelivr.com/v1/packages/npm/@mediapipe/tasks-vision (1.0.1-rc.20260906; opened 2026-09-07)
- @mediapipe/tasks-vision README (privacy notice dated 2026-06-05), https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@1.0.1/README.md
- Google, "Model Card: BlazePose GHUM 3D" (PDF), https://storage.googleapis.com/mediapipe-assets/Model%20Card%20BlazePose%20GHUM%203D.pdf (Pixel 3 fps figures)
- ONNX Runtime, "Using WebGPU execution provider", https://onnxruntime.ai/docs/tutorials/web/ep-webgpu.html (opened 2026-09-07)
- Hugging Face, "Transformers.js" docs, https://huggingface.co/docs/transformers.js/index (opened 2026-09-07)
- tensorflow/tfjs, "tfjs-backend-webgpu" README, https://github.com/tensorflow/tfjs/tree/master/tfjs-backend-webgpu (opened 2026-09-07)
- aiortc README, https://github.com/aiortc/aiortc (opened 2026-09-07)
- AOSP, "Device as webcam", https://source.android.com/docs/core/camera/webcam (Android 14 QPR1+; opened 2026-09-07)
- DroidCam Linux client page and GitHub, https://www.dev47apps.com/droidcam/linux/ ; https://github.com/dev47apps/droidcam (2.1.5)
- Camo Studio, https://camo.com/studio (host platforms; opened 2026-09-07)
- Don't kill my app!, https://dontkillmyapp.com/ ; https://dontkillmyapp.com/oneplus (undated; OnePlus rank 3)
- OnePlus 13 specs, https://www.oneplus.com/us/13/specs ; OnePlus 12 specs, https://www.oneplus.com/us/12/specs (opened 2026-09-07)
- Raspberry Pi 4 Model B product brief (PDF), https://pip-assets.raspberrypi.com/categories/545-raspberry-pi-4-model-b/documents/RP-008344-DS-5-raspberry-pi-4-product-brief.pdf (5 V/3 A minimum)
