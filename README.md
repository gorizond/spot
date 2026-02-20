# Spot Robot Stack

Robot stack for SpotMicro/Spot running **ROS 2 Kilted** on **k3s (Raspberry Pi)**, deployed via **Rancher Fleet** (GitOps).

This repository contains deployment manifests, runtime services, and operations notes for the Spot robot stack.

## LCD Service

This repository now includes a microservice architecture implementation for controlling an LCD1602 display on the Spot robot using C++ with ROS2.

### Architecture Overview

The LCD service follows a modular ROS2 architecture with the following components:

- `lcd_core`: Low-level LCD hardware interface
- `lcd_display_service`: High-level display management
- `temp_monitor`: Temperature monitoring service
- `uptime_monitor`: System uptime monitoring service
- `config`: Configuration management

### Features

- Written in C++ with ROS2 integration
- Configurable via environment variables
- LCD redraw interval is configurable via `LCD_REFRESH_INTERVAL` (seconds, default: `10`)
- Dockerized with multistage build for minimal footprint
- Kubernetes-ready deployment manifests
- GPIO control for Raspberry Pi using libgpiod
- Modular architecture with separate ROS2 nodes
- **DDS (LCD service):** supports CycloneDDS (`rmw_cyclonedds_cpp`) for reliable inter-container topic delivery on the robot

### Architecture Options

The system now supports multiple deployment architectures:

#### Monolithic Mode
- Single executable with all modules combined
- Compatible with previous implementations
- Uses command-line flags to specify modules: `-m lcd,temp,uptime`

#### Microservices Mode (Recommended)
- Separate ROS2 nodes for each module:
  - `spot_lcd_cpp_lcd_node`: Handles LCD display and subscribes to temperature/uptime topics
  - `spot_lcd_cpp_temp_node`: Monitors and publishes temperature data
  - `spot_lcd_cpp_uptime_node`: Monitors and publishes uptime data
- Each node can run in separate containers from the same image
- Nodes communicate via ROS2 topics

### Running with Docker

#### Single Container (All Nodes)
```bash
docker build -f docker/lcd/Dockerfile.cpp -t spot-lcd-cpp .
docker run -d --device=/dev/gpiochip0:/dev/gpiochip0 -v /proc:/proc spot-lcd-cpp:latest all
```

#### Separate Containers (Modular Approach)
```bash
# Build the image
docker build -f docker/lcd/Dockerfile.cpp -t spot-lcd-cpp .

# Run individual nodes
docker run -d --name lcd-node spot-lcd-cpp:latest lcd
docker run -d --name temp-node spot-lcd-cpp:latest temperature  
docker run -d --name uptime-node spot-lcd-cpp:latest uptime
```

### Available Commands

The entrypoint script supports the following commands:
- `lcd` - Run only the LCD display node
- `temperature` or `temp` - Run only the temperature monitoring node
- `uptime` - Run only the uptime monitoring node
- `all` or `combined` - Run the legacy combined node

### Fleet/GitOps Deployment

The system is configured for Fleet-based deployment using the files in the root directory:

- `fleet.yaml` - Main Fleet configuration
- `kustomization.yaml` - Kustomize configuration
- `lcd.yaml` - Deployment definitions for LCD services (contains DaemonSets for all three nodes)
- `deployment.yaml` - Main robot deployment

The `lcd.yaml` file defines three separate DaemonSets that can run on nodes with the appropriate labels:

1. `spot-lcd-display` - Handles LCD display functionality
2. `spot-temp-monitor` - Monitors system temperature
3. `spot-uptime-monitor` - Monitors system uptime

To deploy with Fleet, ensure your nodes have the required labels:
- `gorizond.io/robot=true`
- `gorizond.io/spot-lcd=true`

The deployment will automatically use the image from GitHub Container Registry: `ghcr.io/gorizond/spot-lcd-cpp:latest`

### Docker and CI/CD

The LCD service is fully integrated with GitHub Actions for automated building and publishing:

- Docker images are automatically built and published to GitHub Container Registry
- Workflow triggers on changes to LCD-related files
- Images tagged with branch names, semantic versions, and git SHAs.

## HC-SR04 ultrasonic service

`spot-hcsr04` is deployed via `hcsr04.yaml` as a dedicated DaemonSet:

- image: `ghcr.io/gorizond/spot-hcsr04-cpp:latest`
- GPIO access via `/dev/gpiochip0`
- default sensors mapping (from `SENSORS`): `front_left:5:12,front_right:6:13`
- DDS runtime: `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` (kept consistent with LCD stack)

Published topics:

- `/spot/sensor/ultrasonic/front_left/range`
- `/spot/sensor/ultrasonic/front_right/range`

The node performs staggered measurements and emits a **status line every minute** per sensor with:

- TRIG/ECHO pins
- distance in meters and centimeters
- pulse width (`pulse_us`)
- status (`ok`, `rise_timeout`, `fall_timeout`, `echo_read_error`)
- measurement/timeouts counters (+delta per minute)

Example logs:

```bash
kubectl -n spot-system logs ds/spot-hcsr04 --tail=100
```

Quick troubleshooting (field checklist):

1. Verify there is no close obstacle in front of the sensor (bracket/cable/frame at 2–10 cm).
2. If one channel is stuck around ~3–6 cm with frequent `rise_timeout`, swap **physical HC-SR04 modules** between sides:
   - issue moves with module → faulty sensor module;
   - issue stays on channel → wiring/pin/power path issue.
3. Ensure ECHO line uses a proper 5V→3.3V divider/level shifting.
4. Check 5V and GND quality at the sensor under load (brownouts/noisy GND cause unstable echoes).
5. If left/right got physically rewired, update `SENSORS` mapping accordingly.

### Known incident (2026-02-18)

- Symptom: `front_left` was stuck around ~3–6 cm and frequently logged `rise_timeout`.
- Verification: ECHO path used a proper voltage divider; wiring was rechecked/swapped on sensor side.
- Root cause: faulty HC-SR04 module on the left channel.
- Resolution: replacing the module restored stable measurements.
- Related hardening: switched `spot-hcsr04` to `rmw_cyclonedds_cpp` and added minute-level status logging for easier diagnostics.

## Stereo vision (2x Logitech C270 via USB)

`spot-stereo-c270` is deployed via `stereo-c270.yaml` as **three DaemonSets** (one pod each on labeled robot nodes):

- `spot-stereo-c270-left`: left `usb_cam`
- `spot-stereo-c270-right`: right `usb_cam`
- `spot-stereo-c270-core`: `camera_info_restamp.py` + `stereo_image_proc` (`disparity_node` + `point_cloud_node`)
- image: `ghcr.io/gorizond/spot-stereo-c270:latest`
- runtime: ROS 2 Kilted
- sync mode: `approximate_sync=true` (default tolerance `0.35s`)
- clean timestamp fix: `camera_info_restamp.py` publishes `/stereo/{left,right}/camera_info_sync` with `camera_info.header.stamp == image_raw.header.stamp`
- node label gate: `gorizond.io/spot-stereo=true`
- DDS runtime: `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`

Default video mapping on `spot-5` uses **stable by-path symlinks**:

- left: `platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-video-index0`
- right: `platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0-video-index0`

Important: for two identical C270 webcams, `by-id` may collide (same serial), so `by-path` is preferred.

Published topics (namespace `stereo`):

- `/stereo/left/image_raw`, `/stereo/right/image_raw`
- `/stereo/left/camera_info`, `/stereo/right/camera_info`
- `/stereo/left/camera_info_sync`, `/stereo/right/camera_info_sync`
- `/stereo/disparity`
- `/stereo/points2`

Quick enable on robot node:

```bash
kubectl label node spot-5 gorizond.io/spot-stereo=true --overwrite
```

Notes:

- Physical setup baseline `8.5 cm` with both cameras rotated equally (portrait) is supported.
- After any mechanical change (baseline/rotation), redo stereo calibration.
- `usb_cam` configs are embedded in `spot-stereo-c270-config`; tweak resolution/FPS there.

## What runs

`deployment.yaml` deploys two DaemonSets (one pod per robot node):

- `ros2-smoke` (core):
  - `node-label-config`: reads **Kubernetes Node labels** of *its own node* (RBAC-enabled), converts them into a PCA9685 servo mapping, and publishes it to `/spot/config/servo_map` (reacts to label changes via watch; no pod restart).
  - `servo-driver`: subscribes to `/spot/config/servo_map`, drives **PCA9685** over I2C (`/dev/i2c-1`), accepts JSON commands on `/spot/cmd/servo`, and publishes status to `/spot/state/servo` (runs privileged for I2C access).
  - `cmd-mux`: routes commands to `/spot/cmd/servo` from **manual** (`/spot/cmd/servo_manual`) or **auto** (`/spot/cmd/servo_auto`) based on `/spot/ctrl/mode`.
- `spot-champ` (optional):
  - `champ-controller`: runs CHAMP gait controller (`joint_states`, `/cmd_vel`, etc).
  - `champ-bridge`: bridges CHAMP `joint_states` to `/spot/cmd/servo_auto` (applied only when mux is in `auto`).

Safety defaults:

- starts **disarmed** (`START_ARMED=0`)
- clamps each joint to `min/center/max` from node labels
- hard safety clamp via `SAFE_MIN_US`/`SAFE_MAX_US` (defaults: `500..2500`)
- slew-rate limiting (`MAX_SLEW_US_PER_S`)

Pods are scheduled only on nodes matching:

- `kubernetes.io/arch=arm64`
- `gorizond.io/robot=true`

## Hardware notes (power)

- RPi4 power: XL4015 fed directly from 2S (thick wires) to avoid voltage sag/heat.
- Servo power (`PCA9685 V+` / servos): separate `6V` UBEC; tie grounds at a star point.
- Brownouts show up as boot-loops; measure 5V at the GPIO 5V/GND pins under load.

## Deploy with Fleet

Create a Fleet `GitRepo` in your Fleet workspace (e.g. `workspace-negashev`) pointing to this repository and target your robot cluster.

```yaml
spec:
  repo: https://github.com/gorizond/spot
  branch: main
  targets:
    - clusterName: spots
```

## Node-label servo mapping (no ConfigMaps)

Per-robot mapping is stored on the Kubernetes **Node** object as labels.

Label prefix: `gorizond.io/spot-pca9685-`

- `...i2c-bus` (default: `1`)
- `...address` (default: `0x40`)
- Per channel `0..15`:
  - `...chN-joint` (unset/empty => unused)
  - `...chN-us` (min/center/max, default: `1000-1500-2000`)
  - `...chN-invert` (`0`/`1`, default: `0`)

`...chN-us` value must be a valid Kubernetes label value (no commas). Use e.g. `1450-1500-1550`.

(Backward-compatible) `...chN-min-us`, `...chN-center-us`, `...chN-max-us` are still supported if `...chN-us` is not set.

For the current SpotMicro wiring, **CH6–CH9 are empty** (leave `ch6-joint..ch9-joint` unset).

## First movement (safe)

1) Put the robot in a safe position (lifted / legs can move freely).

2) Exec into the DaemonSet pod and use the `servo-driver` container:

```bash
kubectl -n spot-system exec -it ds/ros2-smoke -c servo-driver -- bash
```

If you need a one-liner (non-interactive), use:

```bash
kubectl -n spot-system exec ds/ros2-smoke -c servo-driver -- bash -lc 'source /opt/ros/kilted/setup.bash && ros2 topic list'
```

3) Source ROS 2 environment (required for `rclpy` / `ros2` commands):

```bash
source /opt/ros/kilted/setup.bash
```

4) Switch to **manual** mode (so CHAMP does not overwrite your commands).
Default mux mode is `auto`, so manual control requires this:

```bash
ros2 topic pub /spot/ctrl/mode std_msgs/msg/String "data: manual" -1
```

5) (Optional) Reset targets to home (0.0):

```bash
python3 /opt/spot/spot_cli.py --repeat 1 home
```

6) Arm (does not move anything until you `set` a joint):

```bash
python3 /opt/spot/spot_cli.py --repeat 1 arm
```

7) Enable + move one joint with small steps (calibration-friendly):

```bash
python3 /opt/spot/spot_cli.py --repeat 1 set-us rf_hip=1500
python3 /opt/spot/spot_cli.py --repeat 1 set-us rf_hip=1510
python3 /opt/spot/spot_cli.py --repeat 1 set-us rf_hip=1500
```

8) Disarm when done (note: may drop torque if `DISARM_FULL_OFF=1`):

```bash
python3 /opt/spot/spot_cli.py disarm
```

If something goes wrong:

```bash
python3 /opt/spot/spot_cli.py estop
```

### Known-good manual bend (front-right leg example)

Use limits from `/spot/config/servo_map` or node labels (`gorizond.io/spot-pca9685-ch*-us`).
On `spot-5`, a full front-right bend was visible with:

```bash
python3 /opt/spot/spot_cli.py --repeat 1 set-us rf_hip=500 rf_upper=2500 rf_lower=2450
```

Return to a neutral-ish pose:

```bash
python3 /opt/spot/spot_cli.py --repeat 1 set-us rf_hip=500 rf_upper=1000 rf_lower=1500
```

## Stand + micro-steps

After calibration, a conservative **stand** pose is:

```bash
python3 /opt/spot/spot_cli.py --repeat 1 stand
```

Defaults (override via flags or env `STAND_HIP/STAND_UPPER/STAND_LOWER/STAND_HIP_REAR_OFFSET/STAND_REAR_UPPER_OFFSET/STAND_REAR_LOWER_OFFSET`):

- `hip=0.02`
- `upper=0.05`
- `lower=0.05`
- `rear_hip_offset=0.0` (applied to `rh/lh` only)
- `rear_upper_offset=0.0` (applied to `rh/lh` only)
- `rear_lower_offset=0.0` (applied to `rh/lh` only)

Example: move rear hips back a bit (safer stance):

```bash
python3 /opt/spot/spot_cli.py --repeat 1 stand --rear-hip-offset -0.06
```

One-leg micro-step (lift/return one leg by moving `*_lower`):

```bash
python3 /opt/spot/spot_cli.py --repeat 1 step lh
```

Repeat micro-steps (in-place crawl):

```bash
python3 /opt/spot/spot_cli.py --repeat 1 walk --steps 3
```

Tune lift amplitude and timing (start small):

```bash
python3 /opt/spot/spot_cli.py --repeat 1 walk --steps 1 --lift 0.05 --lift-hold 0.2 --down-hold 0.2
```

(Experimental) add a small hip swing while the leg is lifted:

```bash
python3 /opt/spot/spot_cli.py --repeat 1 walk --steps 1 --hip-swing 0.03
```

## CHAMP gait (walk via `/cmd_vel`)

This uses CHAMP (model-based gait controller) and bridges its `joint_states` to the low-level `servo-driver` via the command mux.

### Enable/disable

`spot-champ` is gated by a node label so it doesn't override manual `spot_cli` commands.

To enable CHAMP on a robot node:

- Add label `gorizond.io/spot-champ=true` to the node (e.g. `spot-1`).

### Image

By default, `spot-champ` uses two images:

- `champ-controller`: `ghcr.io/gorizond/spot-champ:main`
- `champ-bridge`: `ghcr.io/gorizond/spot-champ-bridge-cpp:latest`

Both are built and pushed by GitHub Actions in this repository.
If your package is private, configure an `imagePullSecret` for GHCR.

To build the controller image locally instead, edit `deployment.yaml` and run:

```bash
docker build -t spot-champ:local -f docker/champ/Dockerfile .
```

Then, drive the gait by publishing `/cmd_vel` (start small):

```bash
kubectl -n spot-system exec -it ds/spot-champ -c champ-controller -- bash
source /opt/ros/kilted/setup.bash
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash
ros2 topic pub /spot/ctrl/mode std_msgs/msg/String "data: auto" -1
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.05}}' -r 2
```

Notes:

- `champ-bridge` publishes servo targets only when `servo-driver` is `armed=true`.
- Manual vs auto is controlled by `/spot/ctrl/mode` (`auto` or `manual`).
- Tune bridge scaling via env on `champ-bridge`: `CHAMP_GAIN`, `CHAMP_*_RANGE_RAD`, `STAND_*`.

## Command mux (manual vs auto)

- `cmd-mux` listens:
  - manual: `/spot/cmd/servo_manual` (default for `spot_cli.py`)
  - auto: `/spot/cmd/servo_auto` (used by `champ-bridge`)
  - output: `/spot/cmd/servo`
- Default mode is `auto`. Switch to `manual` before issuing direct commands.
- Optional: `MANUAL_TIMEOUT_S` can auto-return to `auto` after inactivity.

## Verify

- DaemonSets are running in namespace `spot-system` (`ros2-smoke`, `spot-champ`)
- Logs:
  - `node-label-config` prints which channels are mapped
  - `servo-driver` prints PCA9685 connect + updates
- ROS topics:

```bash
ros2 topic echo /spot/config/servo_map --once
ros2 topic echo /spot/state/servo --once
ros2 topic echo /spot/state/mux --once
```

## Command format

`/spot/cmd/servo` expects `std_msgs/String` JSON:

- arm/disarm: `{"cmd":"arm","value":true}` / `{"cmd":"arm","value":false}` (arming gates output; joints move only after they are enabled via `cmd=set`)
- estop: `{"cmd":"estop","value":true}` / `{"cmd":"estop","value":false}`
- home: `{"cmd":"home"}`
- set targets (also enables those joints):
    - normalized: `{"cmd":"set","mode":"norm","targets":{"rf_hip":0.1}}` (range `-1..1`)
  - microseconds: `{"cmd":"set","mode":"us","targets":{"rf_hip":1500}}`

For manual control (via mux), publish to `/spot/cmd/servo_manual` (default in `spot_cli.py`).
For convenience inside the pod, use `python3 /opt/spot/spot_cli.py ...`.
# Update Fri Jan 30 16:58:14 +05 2026