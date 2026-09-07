# Merge path: LeRobot / LeKiwi / SO-101 and ROS 2 Nav2 + RPLIDAR (state as of 2026-09-07)

Method note: the session's WebSearch budget was exhausted before this task started, so every fact below comes from direct fetches of primary pages (official docs, GitHub raw files, vendor datasheets, REPs). Where a page could not be opened the claim is tagged as unverified. Tags: [MEASURED] someone ran it on named hardware; [VENDOR] spec sheet or vendor/maintainer statement; [INFERRED] my estimate.

## Summary

- LeRobot is at v0.6.1 (released 2026-08-03; `main` says 0.6.2), requires Python >= 3.12 and torch >= 2.7, uses dataset format v3.0, and exposes a small `Robot` abstract class plus a pip-plugin discovery rule (`lerobot_robot_*`). A custom differential-drive base is a ~200-line plugin, not a fork. The published LeKiwi is a Raspberry Pi 5 (4 GB) driving three STS3215 omni wheels and an SO-101 arm on one Feetech bus, with a ZMQ host on the Pi (PULL :5555 commands, PUSH :5556 JPEG observations, 500 ms watchdog, 30 Hz loop) and the policy on a laptop/GPU box. Its wheel command cap works out to about 0.23 m/s.
- SO-101 is independent of the base for the bench phase: a follower is ~$122 (US), leader+follower ~$230, one Waveshare bus adapter per arm (~$10.60), USB to any Linux/macOS host. When merged it must move to the Pi's USB, because the arm rides on the rover.
- SmolVLA (450M params, ~2 GB VRAM at inference) fits next to an int4 27B on 2x 3090; pi0 (14 GB at inference) is marginal; pi0.5 fine-tuning is documented for an 80 GB GPU.
- ROS 2 in September 2026: Jazzy (LTS, Ubuntu 24.04, EOL May 2029), Kilted (EOL Dec 2026), Lyrical Luth (LTS, released 2026-05-22, Ubuntu 26.04 Tier 1 on arm64, EOL May 2031). Default RMW is still rmw_fastrtps_cpp; rmw_zenoh_cpp is Tier 1 in Kilted and Lyrical, router-based, no multicast, which is what you want over home Wi-Fi. Nav2 has binaries for Humble, Jazzy, Lyrical; Kilted and newer default to `TwistStamped`.
- RPLIDAR C1 (datasheet v1.2, 2026-03-12): 12 m, 5 kHz, 10 Hz, 460800 baud, 5 V at 230 mA running / 800 mA start. LDROBOT LD19 is discontinued; its successor STL-19P is 12 m, 5 kHz, 6-13 Hz, 230400 baud, 290 mA. Either is fine; C1 has the more current ROS 2 driver.
- The brief's architecture survives the merge if v1 fixes four things now: SI units and REP-103 axes on the ESP32 link, per-wheel encoder ticks with timestamps (so odometry can be recomputed), `TwistStamped`-shaped velocity commands, and one serial protocol library shared by the future ros2_control hardware interface and the LeRobot plugin. Put a 6-axis IMU on the ESP32 I2C bus now; Nav2 docs call it strongly recommended.

## State of the art (2026)

### LeRobot repository, dataset, training and inference flow

Release cadence has been monthly since January: v0.4.3 (2026-01-22, third-party policy registration via pip, PEFT), v0.4.4 (2026-02-27, CAN motors, streaming video encoding), v0.5.0 (2026-03-09, Python >= 3.12, async inference restored), v0.5.1 (2026-04-07, LeKiwi support in the `lerobot-rollout`/eval CLI), v0.6.0 (2026-07-06, minimal base install with `lerobot[training]` extra, torch >= 2.7, `--dataset.rgb_encoder.vcodec`, 2x faster dataloader, depth support), v0.6.1 (2026-08-03, `lerobot.types` renamed `lerobot.lerobot_types`). `pyproject.toml` on `main` pins `torch>=2.7,<2.12.0`, `numpy>=2.0.0,<2.3.0`, `pyzmq>=26.2.1,<28.0.0` (extra `lekiwi` = `feetech` + `pyzmq`), `grpcio>=1.73.1` (extra `async`) [VENDOR].

Dataset v3.0 shipped with lerobot >= 0.4.0: many episodes per Parquet/MP4 shard, `meta/info.json` (schema, fps, path templates), `meta/stats.json`, `meta/tasks.jsonl`, `meta/episodes/` as chunked Parquet, `StreamingLeRobotDataset` for hub streaming, and a mandatory `dataset.finalize()` before `push_to_hub()`. Converter: `python -m lerobot.scripts.convert_dataset_v21_to_v30 --repo-id=...` [VENDOR].

Flow: `lerobot-find-port` -> `lerobot-setup-motors` -> `lerobot-calibrate` -> `lerobot-teleoperate` -> `lerobot-record --dataset.repo_id=... --dataset.num_episodes=...` -> `lerobot-train --policy.path=lerobot/smolvla_base --dataset.repo_id=...` -> `lerobot-rollout --strategy.type=base --robot.type=... --policy.path=...` (optional `--inference.type=rtc` real-time chunking for weak hosts). Remote inference: `python -m lerobot.async_inference.policy_server --host=0.0.0.0 --port=8080` on the GPU box, `python -m lerobot.async_inference.robot_client --server_address=... --robot.type=...` on the robot; defaults `actions_per_chunk=50`, `chunk_size_threshold=0.7` (docs recommend 0.5-0.6) [VENDOR].

### The Robot abstraction (how a custom diff-drive base plugs in)

Subclass `Robot`, declare `config_class` and `name`, register the config with `@RobotConfig.register_subclass("my_base")`, implement `observation_features` and `action_features` (callable before connect), `connect/disconnect/calibrate/configure/get_observation/send_action`, `is_connected/is_calibrated`. Feature keys follow `joint.pos` / `x.vel` conventions; cameras are `(h, w, 3)`. Ship it as a package named `lerobot_robot_<name>` with `MyBaseConfig`/`MyBase` classes exported from `__init__.py` and the CLI tools discover it with no changes to lerobot [VENDOR]. For a diff-drive base the action dict is `{"x.vel": m/s, "theta.vel": deg/s}`; LeKiwi also carries `y.vel`, which you would fix at 0.0 only if you want LeKiwi-shaped datasets.

### LeKiwi hardware and software split

BOM (SIGRobotics-UIUC, current README): Raspberry Pi 5 4 GB $60, 3x 4-inch omni wheels $9.99 each, 3x STS3215 12 V $13.89 each, 12 V 5 A battery $32.99, 12 V to 5 V 5 A USB-C converter $9.99, 2x USB cameras $12.98 each, SO-101 two-arm set $199.78; total 12 V version $482 (US) / EUR 545.80, base-only $251.50, wired (no Pi) $184 [VENDOR]. The docs say the host is "normally a Raspberry Pi, but can be any PC that can run on 5V and has enough usb ports (2 or more)"; a Pi 4 is not excluded.

Code (`src/lerobot/robots/lekiwi/`): one `FeetechMotorsBus` with arm ids 1-6 (position mode) and wheel ids 7-9 (velocity mode); `x.vel`/`y.vel` in m/s, `theta.vel` in deg/s; kinematics `wheel_radius=0.05`, `base_radius=0.125`, mount angles `[240, 0, 120] - 90` deg, `max_raw=3000` steps/s with `4096` steps/rev and proportional scale-down when any wheel exceeds it; `stop_base()` writes `Goal_Velocity=0` to the wheels. Host (`lekiwi_host.py`): ZMQ PULL on `port_zmq_cmd=5555` with `CONFLATE`, PUSH on `port_zmq_observations=5556` with `SNDHWM=2`, JPEG quality 90 multipart, `max_loop_freq_hz=30`, `watchdog_timeout_ms=500` -> "Stopping the base", `connection_time_s=30`. Client: `polling_timeout_ms=15`, `connect_timeout_s=5`, teleop speed levels 0.1/0.2/0.3 m/s and 30/60/90 deg/s (the docs table still shows 0.4 m/s fast; the code is authoritative) [VENDOR]. 3000 steps/s x (1/4096) rev x 2pi x 0.05 m = 0.23 m/s linear cap [INFERRED from code constants]. `examples/lekiwi/evaluate.py` runs the policy on the laptop at `FPS = 30` and sends actions back over ZMQ; the Pi never runs the network.

### SO-101

Follower: 6x STS3215 1/345. Leader: 1/345 shoulder lift, 1/191 base and elbow, 1/147 wrist and gripper. Two arms = 12 servos at $13.89 each (US), 2x Waveshare "Motor Control Board" at $10.60, 2x PSU at $10; total $229.88 US / EUR 226.30; single follower $121.94 [VENDOR, SO-ARM100 README]. "The leader arm is always 7.4V for the SO101"; the 12 V STS3215 has 30 kg.cm stall vs 16.5 kg.cm (at 6 V) for the 7.4 V part. Bus adapter jumpers on channel `B` for USB. Ports appear as `/dev/ttyACM*`; `sudo chmod 666 /dev/ttyACM0` or a udev rule.

### Policies vs a 2x RTX 3090 box

SmolVLA: 450M params (~350M VLM + ~100M action expert), pretrained on ~10M frames from 487 community SO100 datasets; "~2GB" at inference; fine-tune 20k steps ~4 h on a single A100 (batch 64); 50 episodes recommended, 25 was "not enough" [VENDOR]. pi0: "occupies 14GB of memory at inference time" [VENDOR]. pi0.5: quickstart is "Sized for a single 80 GB GPU" (batch 64, bf16, gradient checkpointing, full fine-tune); `--policy.train_expert_only=true` cuts memory but no 24 GB figure is published [VENDOR]. An int4 27B split across two 3090s leaves roughly 12-15 GB free per card [INFERRED]; SmolVLA fits comfortably, pi0 inference is marginal, pi0/pi0.5 fine-tuning is not a 24 GB job as documented.

### ROS 2 distributions, RMW, Wi-Fi

REP-2000 and the release pages: Humble May 2022 - May 2027 (Ubuntu 22.04); Jazzy Jalisco 2024-05-23 - May 2029, LTS, Ubuntu 24.04 arm64 Tier 1; Kilted Kaiju 2025-05-23 - Dec 2026, Ubuntu 24.04; Lyrical Luth 2026-05-22 - May 2031, LTS, Ubuntu 26.04 (Resolute) amd64/arm64 Tier 1, Ubuntu 24.04 Tier 3; next is Makoa Mata-mata, May 2027 [VENDOR]. Default RMW is `rmw_fastrtps_cpp` in every one of them. `rmw_zenoh_cpp` was elevated to Tier 1 in Kilted and stays Tier 1 in Lyrical; it does not appear in Jazzy's tier table (a `jazzy` branch exists) [VENDOR].

rmw_zenoh design: "relies on a Zenoh router to discover peers" via gossip scouting; "The decision to not rely on UDP multicast for discovery was intentional, aimed at avoiding issues with misconfigured networks"; router listens `tcp/[::]:7447`; start it with `ros2 run rmw_zenoh_cpp rmw_zenohd`; cross-host via a router config `connect: { endpoints: ["tcp/<ip>:7447"] }` through `ZENOH_ROUTER_CONFIG_URI` or `ZENOH_CONFIG_OVERRIDE`; multicast scouting can be re-enabled but is off by default [VENDOR]. That is the practical answer to DDS multicast discovery dying on consumer Wi-Fi. Lyrical also adds `EventsCBGExecutor` (10-15 % less CPU) and `AsyncNode` [VENDOR].

Ubuntu images: 24.04.4 `preinstalled-server-arm64+raspi` supports Pi 3/4/5; 26.04.1 LTS is certified on Pi 4B and Pi 5 [VENDOR].

### Nav2, collision monitor, odometry, ros2_control

Nav2 README CI table: Humble (Jammy), Jazzy (Noble), Lyrical (Resolute); `nav2_collision_monitor` on `main` is 1.5.0 [VENDOR]. Migration notes: "In Kilted and newer, the default `cmd_vel` topic for all `Twist` publishers and subscriptions is changed to `TwistStamped`" (`enable_stamped_cmd_vel: false` restores Twist); Lyrical adds a collision-monitor toggle service, a `costmap` observation source, debounce parameters, and raises `bond_heartbeat_period` 0.1 -> 0.25 s to cut CPU [VENDOR].

Collision monitor: "operate[s] below Nav2 as an independent safety node", pipeline controller -> velocity smoother -> collision monitor -> robot; defaults `cmd_vel_in_topic: cmd_vel_smoothed`, `cmd_vel_out_topic: cmd_vel`, `base_frame_id: base_footprint`, `transform_tolerance: 0.1`, `source_timeout: 2.0` ("If no new data is received within this interval, the robot will be stopped"), `stop_pub_timeout: 1.0`, `enable_stamped_cmd_vel: true`; polygon `action_type` in `stop|slowdown|limit|approach`, `min_points: 4`, `slowdown_ratio: 0.5`, `time_before_collision: 2.0`; sources `scan|pointcloud|range|polygon|costmap` [VENDOR]. Example stop box `[[0.3,0.3],[0.3,-0.3],[0.0,-0.3],[0.0,0.3]]`. Default bringup runs MPPI at `controller_frequency: 20.0`, velocity smoother at 20 Hz with `max_velocity: [0.5, 0.0, 2.0]`, local costmap 5 Hz at 0.05 m [VENDOR].

Odometry: Nav2 requires `nav_msgs/Odometry` plus the `odom -> base_link` transform (REP-105); "IMUs drift over time while wheel encoders drift over distance traveled, thus they are often used together"; `robot_localization` EKF example: `frequency: 30.0`, `world_frame: odom`, `publish_tf: true`, odom0 fuses vx, vy, vyaw, imu0 fuses vyaw [VENDOR]. REP-103: metres, radians, seconds; x forward, y left, z up; right-handed [VENDOR].

`diff_drive_controller` (Jazzy docs, parameter YAML on master): subscribes `~/cmd_vel` as `geometry_msgs/msg/TwistStamped`; publishes `~/odom` and `/tf` (`enable_odom_tf: true`); `wheel_separation`, `wheel_radius` (default 0.0, must set), `cmd_vel_timeout: 0.5`, `open_loop: false`, `position_feedback: true`, `publish_rate: 50.0`, `velocity_rolling_window_size: 10`, limits under `linear.x` / `angular.z` (`max_velocity`, `max_acceleration`, `max_jerk`), suggested covariance diagonal `[0.001,0.001,0.001,0.001,0.001,0.01]`. The hardware interface must expose a velocity command interface and a position (or velocity) state interface per wheel [VENDOR].

### micro-ROS on the ESP32

`micro_ros_espidf_component`: branches humble/iron/jazzy/kilted/rolling, tested on ESP-IDF v5.2-v6.0, targets ESP32/S2/S3/C3/C6, UDP transport by default, UART via `colcon.meta` (`-DRMW_UXRCE_TRANSPORT=custom`, UART_NUM_0/1/2); agent `microros/micro-ros-agent:kilted serial --dev <port> -v6`. No Lyrical branch as of the README fetched today. `micro_ros_arduino` pins "Arduino core for the ESP32 (v2.0.2)" and states "Only USB serial transports are provided" [VENDOR].

### SLAM on a Pi 4

slam_toolbox's own benchmark is "a low power 7th gen i7", 5x real-time up to ~30,000 sq ft [MEASURED, maintainer]. TurtleBot 4 (Pi 4B) docs: "It is recommended to run synchronous SLAM on a remote PC to get a higher resolution map" [VENDOR]. No Pi 4 4 GB CPU-load measurement for Jazzy/Lyrical Nav2 was found in the pages I could open.

### 2D lidars

RPLIDAR C1 (model C1M1, datasheet rev 1.2 dated 2026-03-12): white 0.05-12 m (70 %), black 0.05-6 m (10 %); 5 kHz; 8-12 Hz, 10 Hz typical; 0.72 deg at 10 Hz; +/-30 mm accuracy, 15 mm resolution; TTL UART 460800 8n1; 4.8-5.2 V, start 800 mA, running 230 mA typ / 260 mA max at 10 Hz; 110 g; 55.6 x 55.6 x 41.3 mm; IP54; 40,000 lux; left-handed clockwise angle convention; XH2.54-5P connector [VENDOR]. Driver `sllidar_ros2`: `sllidar_c1_launch.py` with `serial_baudrate: 460800`, `frame_id: laser`, `angle_compensate: true`, `scan_mode: Standard`, `serial_port: /dev/ttyUSB0`; README install text still names foxy/galactic/humble/rolling [VENDOR]. Street price could not be verified (vendor and retailer pages blocked); 2025 listings were in the $70-100 range [INFERRED, unverified].

LDROBOT family (reseller wiki, spec table): FHL-LD19 is marked "Discontinue"; STL-19P (FHL-LD19P): 0.03-12 m white / 0.03-8 m black, 5000 Hz, 6-13 Hz, UART 230400, 4.5-5.5 V, start 540 mA / working 290 mA, 54.0 x 46.3 x 34.8 mm, 45 g; LD19: 0.02-12 m (70 %), 4500 Hz, 5-13 Hz, 300/180 mA, 230400, 47 g; STL-27L: 25 m, 21600 Hz, 921600 baud [VENDOR]. Driver `ldlidar_stl_ros2` ships `ld06.launch.py`, `ld19.launch.py`, `stl27l.launch.py`, `port_baudrate: 230400`, "supports ubuntu 20.04 ros2 foxy version and above"; STL-19P is not named but speaks the LD19 protocol at the same baud [INFERRED]. RPLIDAR A1 is the older triangulation unit (115200 baud, ~5.5 Hz) and is not worth buying new in 2026 [INFERRED; A1 specs not re-fetched].

## Recommendation for this build

1. OS and Python on the Pi 4. Run Ubuntu Server, not Raspberry Pi OS Bookworm: LeRobot requires Python >= 3.12 and Bookworm ships 3.11. Ubuntu 24.04 (Python 3.12) is the low-risk pick and pairs with Jazzy; Ubuntu 26.04 pairs with Lyrical LTS. Whichever you pick, run the same distro on the box.
2. Distro. Start Nav2 work on Lyrical Luth (LTS to May 2031, rmw_zenoh Tier 1, Nav2 binaries). Fall back to Jazzy only if a driver you need lacks Lyrical binaries; sllidar_ros2 and ldlidar_stl_ros2 build from source on any distro.
3. RMW. `RMW_IMPLEMENTATION=rmw_zenoh_cpp`, one `rmw_zenohd` on the Pi, box router config `connect.endpoints=["tcp/<pi-ip>:7447"]`. No multicast on the home Wi-Fi.
4. Split. Pi: lidar driver, camera, serial bridge to the ESP32, `ros2_control` with `diff_drive_controller`, `robot_localization` EKF, `nav2_collision_monitor`, `nav2_velocity_smoother`. Box: `slam_toolbox` (async online), Nav2 planner/controller/BT, RViz. The Pi's node is the only writer to the ESP32 and it is where `cmd_vel` passes through the collision monitor; `source_timeout` then stops the robot when the lidar stream dies, and the ESP32 300 ms TTL stops it when the Pi dies.
5. ESP32 seam. Keep a plain framed serial protocol (COBS or length-prefixed, CRC) instead of micro-ROS. Payloads: command `{seq, t_ms, v_mps, w_radps, ttl_ms}`; telemetry at >= 50 Hz `{seq, t_us, left_ticks, right_ticks, gyro_z_radps (optional), bumper, tof_mm[], current_mA, fault_bits}`. One small Python library speaks it; the ros2_control `SystemInterface` (the `diffdrive_arduino` pattern) and the LeRobot plugin both import it.
6. LeRobot plugin. Package `lerobot_robot_rover` with `RoverConfig(port, cameras, wheel_radius, wheel_separation, ticks_per_rev)` and `Rover.action_features = {"x.vel": float, "theta.vel": float}`, `observation_features = {"x.vel", "theta.vel", <cameras>}`; `send_action` converts deg/s to rad/s and writes the frame; `get_observation` returns velocities from encoder deltas. Copy LeKiwi's host/client pattern (ZMQ 5555/5556, 500 ms watchdog) if you want the policy on the box; or use `policy_server`/`robot_client` over gRPC 8080.
7. SO-101. Buy the 12 V follower servos so the arm shares the rover's 3S rail through the same 12 V-to-5 V topology LeKiwi uses; the leader is always 7.4 V and lives on the desk with the box. Bench phase: follower on the box's USB, `--robot.type=so101_follower`. Merge phase: follower adapter into the Pi, composite robot class = SO-101 `FeetechMotorsBus` (ids 1-6) plus the serial base.
8. Policy. Fine-tune `lerobot/smolvla_base` on the box (~2 GB at inference, hours-scale fine-tune on a 3090 [INFERRED]); do not plan on pi0/pi0.5 next to the 27B.
9. Lidar. RPLIDAR C1 on the Pi's USB via its bundled UART-USB adapter; budget 800 mA start on the 5 V rail; mount with x forward and set `frame_id: laser` with a static transform to `base_link`. STL-19P is the price-driven alternative with the same class of performance.
10. IMU. Add a 6-axis IMU (BNO055/ICM-42688 class, $3-15) on the ESP32 I2C now and stream gyro z; fuse in the EKF later. It is cheap now and painful to retrofit into a sealed chassis.

## Numbers

| quantity | value | hardware/context | tag | source URL |
|---|---|---|---|---|
| LeRobot latest release | v0.6.1, 2026-08-03 (main: 0.6.2) | GitHub releases / pyproject | VENDOR | https://api.github.com/repos/huggingface/lerobot/releases |
| LeRobot Python / torch | >=3.12; torch>=2.7,<2.12 | pyproject.toml on main | VENDOR | https://raw.githubusercontent.com/huggingface/lerobot/main/pyproject.toml |
| Dataset v3.0 introduced | lerobot >= 0.4.0 | docs | VENDOR | https://huggingface.co/docs/lerobot/lerobot-dataset-v3 |
| LeKiwi host ports | PULL 5555 cmd, PUSH 5556 obs | Pi host, ZMQ | VENDOR | https://raw.githubusercontent.com/huggingface/lerobot/main/src/lerobot/robots/lekiwi/config_lekiwi.py |
| LeKiwi watchdog / loop | 500 ms -> stop_base; 30 Hz; connect 30 s | Pi host | VENDOR | https://raw.githubusercontent.com/huggingface/lerobot/main/src/lerobot/robots/lekiwi/lekiwi_host.py |
| LeKiwi action units | x.vel,y.vel m/s; theta.vel deg/s | code | VENDOR | https://raw.githubusercontent.com/huggingface/lerobot/main/src/lerobot/robots/lekiwi/lekiwi.py |
| LeKiwi kinematics | wheel_radius 0.05 m, base_radius 0.125 m, max_raw 3000 steps/s, 4096 steps/rev | code | VENDOR | https://raw.githubusercontent.com/huggingface/lerobot/main/src/lerobot/robots/lekiwi/lekiwi.py |
| LeKiwi linear speed cap | ~0.23 m/s | 3000/4096 rev/s x 2pi x 0.05 m | INFERRED | (derived from the two rows above) |
| LeKiwi teleop speeds | 0.1/0.2/0.3 m/s; 30/60/90 deg/s | client code | VENDOR | https://raw.githubusercontent.com/huggingface/lerobot/main/src/lerobot/robots/lekiwi/lekiwi_client.py |
| LeKiwi compute | Raspberry Pi 5 4 GB, $60 | BOM | VENDOR | https://raw.githubusercontent.com/SIGRobotics-UIUC/LeKiwi/main/BOM.md |
| LeKiwi total | $482 US (12 V), base-only $251.50, wired $184 | BOM | VENDOR | https://raw.githubusercontent.com/SIGRobotics-UIUC/LeKiwi/main/BOM.md |
| STS3215 12 V | $13.89 each | BOM | VENDOR | https://raw.githubusercontent.com/SIGRobotics-UIUC/LeKiwi/main/BOM.md |
| SO-101 leader+follower | $229.88 US; follower alone $121.94; 12 servos | SO-ARM100 README | VENDOR | https://raw.githubusercontent.com/TheRobotStudio/SO-ARM100/main/README.md |
| Waveshare bus adapter | $10.60 each | SO-ARM100 README | VENDOR | https://raw.githubusercontent.com/TheRobotStudio/SO-ARM100/main/README.md |
| STS3215 stall torque | 16.5 kg.cm at 6 V (7.4 V part); 30 kg.cm (12 V part) | README | VENDOR | https://raw.githubusercontent.com/TheRobotStudio/SO-ARM100/main/README.md |
| SmolVLA size / VRAM | 450M params; ~2 GB at inference | docs | VENDOR | https://huggingface.co/docs/lerobot/async |
| SmolVLA fine-tune | 20k steps ~4 h, single A100, batch 64 | docs | VENDOR | https://huggingface.co/docs/lerobot/smolvla |
| SmolVLA async gain | 13.75 s -> 9.7 s task time; 19 vs 9 tasks | SO100 paper setup | MEASURED | https://huggingface.co/blog/smolvla |
| pi0 inference memory | 14 GB | docs | VENDOR | https://huggingface.co/docs/lerobot/async |
| pi0.5 fine-tune sizing | "single 80 GB GPU", batch 64 | docs | VENDOR | https://huggingface.co/docs/lerobot/pi05 |
| Async inference defaults | actions_per_chunk 50; chunk_size_threshold 0.7 (0.5-0.6 advised); gRPC 8080 | docs | VENDOR | https://huggingface.co/docs/lerobot/async |
| Jazzy | 2024-05-23 to May 2029, LTS, Ubuntu 24.04 arm64 Tier 1 | release notes | VENDOR | https://raw.githubusercontent.com/ros2/ros2_documentation/rolling/source/Releases/Release-Jazzy-Jalisco.rst |
| Kilted | 2025-05-23 to Dec 2026; rmw_zenoh Tier 1 | release notes | VENDOR | https://raw.githubusercontent.com/ros2/ros2_documentation/rolling/source/Releases/Release-Kilted-Kaiju.rst |
| Lyrical Luth | 2026-05-22 to May 2031, LTS; Ubuntu 26.04 Tier 1, 24.04 Tier 3; default rmw_fastrtps_cpp; rmw_zenoh Tier 1 | release notes | VENDOR | https://raw.githubusercontent.com/ros2/ros2_documentation/rolling/source/Releases/lyrical/supported-platforms.rst |
| Next distro | Makoa Mata-mata, May 2027 | releases table | VENDOR | https://raw.githubusercontent.com/ros2/ros2_documentation/rolling/source/Releases.rst |
| Zenoh router port | tcp 7447; multicast scouting off by default | rmw_zenoh design | VENDOR | https://raw.githubusercontent.com/ros2/rmw_zenoh/rolling/docs/design.md |
| Nav2 TwistStamped default | Kilted and newer | migration guide | VENDOR | https://docs.nav2.org/rolling/configuration_and_development/migration_guides/jazzy/Jazzy/ |
| Collision monitor defaults | source_timeout 2.0 s; stop_pub_timeout 1.0 s; transform_tolerance 0.1; min_points 4; slowdown_ratio 0.5 | docs | VENDOR | https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/collision_monitor/configuring_collision_monitor_node/ |
| Nav2 bringup rates | MPPI 20 Hz; smoother 20 Hz, max_velocity [0.5,0,2.0]; local costmap 5 Hz, 0.05 m | nav2_params.yaml | VENDOR | https://raw.githubusercontent.com/ros-navigation/navigation2/main/nav2_bringup/params/nav2_params.yaml |
| Lyrical bond heartbeat | 0.1 -> 0.25 s | migration guide | VENDOR | https://docs.nav2.org/rolling/configuration_and_development/migration_guides/kilted/Kilted/ |
| EKF example | frequency 30 Hz; odom vx,vy,vyaw; imu vyaw | Nav2 setup guide | VENDOR | https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/odom/setup_robot_localization/ |
| diff_drive_controller | cmd_vel TwistStamped; cmd_vel_timeout 0.5 s; publish_rate 50 Hz | Jazzy userdoc | VENDOR | https://control.ros.org/jazzy/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html |
| micro-ROS ESP-IDF | IDF v5.2-6.0; ESP32-S3; branches to kilted; UDP default, UART optional | README | VENDOR | https://github.com/micro-ROS/micro_ros_espidf_component |
| slam_toolbox benchmark | 5x real-time to ~30k sq ft on "low power 7th gen i7" | maintainer | MEASURED | https://github.com/SteveMacenski/slam_toolbox |
| TB4 SLAM placement | "recommended to run synchronous SLAM on a remote PC" | Pi 4B robot | VENDOR | https://turtlebot.github.io/turtlebot4-user-manual/tutorials/generate_map.html |
| RPLIDAR C1 range | 0.05-12 m white (70 %); 0.05-6 m black (10 %) | datasheet v1.2, 2026-03-12 | VENDOR | https://download-en.slamtec.com/api/download/rplidar-c1-datasheet/1.2?lang=en |
| RPLIDAR C1 rates | 5 kHz; 8-12 Hz, 10 Hz typ; 0.72 deg; +/-30 mm | datasheet | VENDOR | same |
| RPLIDAR C1 power | 4.8-5.2 V; 800 mA start; 230 mA typ / 260 mA max | datasheet | VENDOR | same |
| RPLIDAR C1 UART | 460800 8n1, 3.3 V TTL; 110 g; 55.6x55.6x41.3 mm | datasheet | VENDOR | same |
| sllidar_ros2 C1 launch | serial_baudrate 460800; frame_id laser | launch file | VENDOR | https://raw.githubusercontent.com/Slamtec/sllidar_ros2/main/launch/sllidar_c1_launch.py |
| STL-19P | 0.03-12 m; 5000 Hz; 6-13 Hz; 230400; 290 mA run / 540 mA start; 45 g | reseller spec table | VENDOR | https://wiki.youyeetoo.com/en/Lidar/D300 |
| LD19 | discontinued; 4500 Hz; 5-13 Hz; 180 mA | reseller spec table | VENDOR | https://wiki.youyeetoo.com/en/Lidar/D300 |
| ldlidar_stl_ros2 baud | 230400; LD06/LD19/STL-27L launch files | README | VENDOR | https://raw.githubusercontent.com/ldrobotSensorTeam/ldlidar_stl_ros2/master/README.md |
| Ubuntu on Pi 4 | 24.04.4 server arm64+raspi; 26.04.1 LTS certified Pi 4B | Canonical | VENDOR | https://cdimage.ubuntu.com/releases/24.04/release/ ; https://ubuntu.com/download/raspberry-pi |

## Corrections to the brief

- "merge as LeKiwi with the Pi as host": a differential-drive ESP32 base is not LeKiwi. LeKiwi is a three-omni-wheel base whose wheels are STS3215 servos on the same Feetech bus as the arm, hosted on a Pi 5. What transfers is the pattern (host on Pi, ZMQ 5555/5556, 500 ms watchdog, `x.vel`/`theta.vel` keys), delivered as your own `lerobot_robot_*` plugin. Holds in spirit; the wording should change.
- "SO-101 on USB to the box": correct for the bench track only. Once the arm rides the rover its bus adapter must be on the Pi, and LeRobot's own guidance is that the host machine needs two or more USB ports for cameras plus the motor board. Plan the Pi's USB budget (lidar, arm bus, camera, mic) now.
- Odometry: the brief lists encoded gearmotors but never says who integrates odometry, at what rate, or in which units. Nav2 needs `nav_msgs/Odometry` plus `odom -> base_link` and calls an IMU strongly recommended. Add per-wheel tick telemetry at >= 50 Hz and an IMU footprint.
- Units: the skill layer's `turn(<=180 deg, <=60 deg/s)` is fine for the LLM, but the ESP32 link should carry m/s and rad/s (REP-103) with x forward, y left, so the later ros2_control interface is a pass-through. LeRobot's LeKiwi uses deg/s for `theta.vel`; convert at the plugin, not in firmware.
- TTL: 300 ms on the ESP32 is consistent with `diff_drive_controller`'s `cmd_vel_timeout: 0.5` and LeKiwi's 500 ms watchdog when the controller runs at 50 Hz. Holds.
- "RPLIDAR C1 + Nav2 on the box": planners on the box is right, but the lidar driver, odometry, ros2_control and the collision monitor belong on the Pi, and the collision monitor only protects you if it is the last writer of `cmd_vel` before the serial bridge.
- Pi OS unstated: current LeRobot needs Python >= 3.12, so Raspberry Pi OS Bookworm (3.11) cannot run the LeKiwi-style host. Use Ubuntu 24.04/26.04 Server.
- GPU budget: SmolVLA fits beside the 27B; pi0 (14 GB) is marginal and pi0.5 fine-tuning is documented for 80 GB. The brief does not claim otherwise, but "LeRobot on the box" should be read as SmolVLA-class.
- Alternatives named in the task (LD19, A1): LD19 is discontinued by the vendor's own table; STL-19P is the current part. A1 is a triangulation unit and not a 2026 buy.

## Alternatives considered and rejected

- micro-ROS on the ESP32-S3 as the seam. Works (ESP-IDF component, UART transport, agent on the Pi), but it binds firmware to a ROS distro branch (no Lyrical branch yet), the Arduino variant pins ESP32 core 2.0.2, and it gives LeRobot nothing. A plain framed serial protocol serves both stacks and keeps the TTL/e-stop logic ROS-free.
- slam_toolbox + Nav2 on the Pi 4 4 GB. TurtleBot 4 docs push synchronous SLAM to a remote PC; the Pi also carries the voice stack. Box hosts the planners.
- Default Fast DDS multicast discovery across Wi-Fi. Replaced by rmw_zenoh (router, TCP, no multicast). Fast DDS discovery server or `ROS_STATIC_PEERS` is the fallback if zenoh misbehaves.
- Building the LeKiwi omni base instead of the brief's diff drive. LeKiwi's wheel cap is ~0.23 m/s and its odometry is open-loop servo velocity; the diff drive with encoders is the better Nav2 citizen. Cost: no direct reuse of LeKiwi datasets, which would not transfer across kinematics and camera placement anyway.
- pi0 / pi0.5 on the 2x 3090. Inference marginal beside the 27B; fine-tuning documented at 80 GB. SmolVLA first.
- Unstamped `Twist` on `cmd_vel`. Kilted and newer default to `TwistStamped`; adopt it from the first ROS node.
- RPLIDAR A1 and LD19. Superseded by C1 and STL-19P respectively.

## Open questions

- No measured CPU/RAM figures for `ros2_control` + EKF + collision monitor + `sllidar_ros2` on a Pi 4 4 GB under Jazzy or Lyrical were found; needs a bench run (gate: `top` while the voice stack also runs).
- Does pi0.5 fine-tune fit 24 GB with `train_expert_only=true`, gradient checkpointing, bf16, batch 8? Not documented.
- rmw_zenoh over consumer Wi-Fi: no published latency/loss measurements found; test `/scan` at 10 Hz and `/cmd_vel` at 20 Hz across the home AP.
- RPLIDAR C1 2026 street price: vendor and retailer pages blocked; unverified.
- micro-ROS Lyrical branch timing, if you ever want the ESP32 as a ROS node.
- LeRobot torch pin (`<2.12`) against the Python shipped by Ubuntu 26.04 on arm64: verify a wheel exists before committing the Pi to 26.04.
- `sllidar_ros2` README names foxy/galactic/humble/rolling only; confirm a clean colcon build on Lyrical.
- Whether `lerobot-rollout --robot.type=<plugin>` accepts a ZMQ-hosted plugin robot the way it accepts `lekiwi` (v0.5.1 added LeKiwi to rollout/eval); check before designing the host.

## Sources

- LeKiwi docs (LeRobot), https://huggingface.co/docs/lerobot/lekiwi , fetched 2026-09-07
- LeRobot releases API, https://api.github.com/repos/huggingface/lerobot/releases , v0.6.1 dated 2026-08-03
- LeRobot pyproject.toml (main), https://raw.githubusercontent.com/huggingface/lerobot/main/pyproject.toml , fetched 2026-09-07
- LeRobotDataset v3.0 docs, https://huggingface.co/docs/lerobot/lerobot-dataset-v3 , fetched 2026-09-07
- Bring your own hardware (Robot base class, plugins), https://huggingface.co/docs/lerobot/integrate_hardware , fetched 2026-09-07
- lekiwi.py / lekiwi_host.py / lekiwi_client.py / config_lekiwi.py, https://github.com/huggingface/lerobot/tree/main/src/lerobot/robots/lekiwi , fetched 2026-09-07
- examples/lekiwi/evaluate.py, https://raw.githubusercontent.com/huggingface/lerobot/main/examples/lekiwi/evaluate.py , fetched 2026-09-07
- LeKiwi BOM (SIGRobotics-UIUC), https://raw.githubusercontent.com/SIGRobotics-UIUC/LeKiwi/main/BOM.md , fetched 2026-09-07
- SO-101 docs, https://huggingface.co/docs/lerobot/so101 , fetched 2026-09-07
- SO-ARM100 README (BOM, prices), https://raw.githubusercontent.com/TheRobotStudio/SO-ARM100/main/README.md , fetched 2026-09-07
- SmolVLA docs, https://huggingface.co/docs/lerobot/smolvla , fetched 2026-09-07
- SmolVLA blog, https://huggingface.co/blog/smolvla , 2025-06-03
- SmolVLA paper abstract, https://arxiv.org/abs/2506.01844 , 2025
- Async inference docs, https://huggingface.co/docs/lerobot/async , fetched 2026-09-07
- pi0 docs, https://huggingface.co/docs/lerobot/pi0 ; pi0.5 docs, https://huggingface.co/docs/lerobot/pi05 , fetched 2026-09-07
- REP-2000 (raw), https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-2000.rst , fetched 2026-09-07
- ROS 2 Releases table, https://raw.githubusercontent.com/ros2/ros2_documentation/rolling/source/Releases.rst , fetched 2026-09-07
- Jazzy release notes, https://raw.githubusercontent.com/ros2/ros2_documentation/rolling/source/Releases/Release-Jazzy-Jalisco.rst
- Kilted release notes, https://raw.githubusercontent.com/ros2/ros2_documentation/rolling/source/Releases/Release-Kilted-Kaiju.rst
- Lyrical release notes and supported platforms, https://raw.githubusercontent.com/ros2/ros2_documentation/rolling/source/Releases/Release-Lyrical-Luth.rst ; https://raw.githubusercontent.com/ros2/ros2_documentation/rolling/source/Releases/lyrical/supported-platforms.rst
- rmw_zenoh README and design, https://github.com/ros2/rmw_zenoh ; https://raw.githubusercontent.com/ros2/rmw_zenoh/rolling/docs/design.md , fetched 2026-09-07
- Nav2 README (distro CI table), https://raw.githubusercontent.com/ros-navigation/navigation2/main/README.md
- Nav2 collision monitor node config, https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/collision_monitor/configuring_collision_monitor_node/
- Nav2 collision monitor tutorial, https://docs.nav2.org/rolling/tutorials/general_tutorials/using_collision_monitor/using_collision_monitor/
- nav2_collision_monitor README, params, package.xml (1.5.0), https://github.com/ros-navigation/navigation2/tree/main/nav2_collision_monitor
- nav2_bringup nav2_params.yaml, https://raw.githubusercontent.com/ros-navigation/navigation2/main/nav2_bringup/params/nav2_params.yaml
- Nav2 odometry setup, https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/odom/setup_odom/ ; robot_localization setup, .../odom/setup_robot_localization/
- Nav2 migration guides Jazzy->Kilted and Kilted->Lyrical, https://docs.nav2.org/rolling/configuration_and_development/migration_guides/jazzy/Jazzy/ ; .../kilted/Kilted/
- diff_drive_controller userdoc (Jazzy), https://control.ros.org/jazzy/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html ; parameter YAML, https://raw.githubusercontent.com/ros-controls/ros2_controllers/master/diff_drive_controller/src/diff_drive_controller_parameter.yaml
- robot_localization ekf.yaml, https://raw.githubusercontent.com/cra-ros-pkg/robot_localization/ros2/params/ekf.yaml
- REP-103, https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0103.rst ; REP-105, https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0105.rst
- micro_ros_espidf_component, https://github.com/micro-ROS/micro_ros_espidf_component ; micro_ros_arduino (kilted), https://github.com/micro-ROS/micro_ros_arduino
- slam_toolbox README, https://github.com/SteveMacenski/slam_toolbox
- TurtleBot 4 manual: generate map, https://turtlebot.github.io/turtlebot4-user-manual/tutorials/generate_map.html ; robot software, https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_robot.html
- linorobot2 README, https://raw.githubusercontent.com/linorobot/linorobot2/humble/README.md
- Slamtec support/downloads, http://www.slamtec.com/en/support ; RPLIDAR C1 datasheet v1.2 (2026-03-12), https://download-en.slamtec.com/api/download/rplidar-c1-datasheet/1.2?lang=en
- sllidar_ros2 README and C1 launch, https://github.com/Slamtec/sllidar_ros2 ; https://raw.githubusercontent.com/Slamtec/sllidar_ros2/main/launch/sllidar_c1_launch.py
- ldlidar_stl_ros2 README, https://raw.githubusercontent.com/ldrobotSensorTeam/ldlidar_stl_ros2/master/README.md ; ldlidar_ros2 README, https://raw.githubusercontent.com/ldrobotSensorTeam/ldlidar_ros2/master/README.md
- LD19 / STL-19P / STL-27L spec table (youyeetoo wiki), https://wiki.youyeetoo.com/en/Lidar/D300 , fetched 2026-09-07
- Ubuntu for Raspberry Pi, https://ubuntu.com/download/raspberry-pi ; 24.04.4 images, https://cdimage.ubuntu.com/releases/24.04/release/
