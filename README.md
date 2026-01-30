# Spot Robot Stack

Robot stack for SpotMicro/Spot running **ROS 2 Kilted** on **k3s (Raspberry Pi)**, deployed via **Rancher Fleet** (GitOps).

This repo is intentionally minimal.

## LCD Service

This repository now includes a microservice architecture implementation for controlling an LCD1602 display on the Spot robot using Rust.

### Architecture Overview

The LCD service follows a microservice architecture with the following components:

- `lcd_core`: Low-level LCD hardware interface
- `lcd_display_service`: High-level display management
- `temp_monitor`: Temperature monitoring service
- `uptime_monitor`: System uptime monitoring service
- `ros_publisher`: ROS2 topic publishing
- `config`: Configuration management

### Features

- Written in safe Rust for memory safety
- Configurable via TOML files
- Dockerized with multistage build for minimal footprint
- Kubernetes-ready deployment manifests
- GPIO control for Raspberry Pi
- Selective module loading via command-line arguments

### Module Selection

The LCD service supports selective module loading through command-line arguments:

```bash
# Run all modules
./spot_lcd_node --module all

# Run specific modules only (comma-separated)
./spot_lcd_node --module temp,uptime

# Run only LCD display
./spot_lcd_node --module lcd

# Run only temperature and LCD
./spot_lcd_node --module temp,lcd
```

This allows for flexible deployments depending on the required functionality.

### Docker and CI/CD

The LCD service is fully integrated with GitHub Actions for automated building and publishing:

- Docker images are automatically built and published to GitHub Container Registry
- Workflow triggers on changes to LCD-related files
- Images tagged with branch names, semantic versions, and git SHAs

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

By default, `spot-champ` uses `ghcr.io/gorizond/spot-champ:main` (built and pushed by GitHub Actions in this repo).

- Tags: `main` (default branch) and `sha-<short>` (per-commit, immutable).
- If your package is private, configure an `imagePullSecret` for GHCR.

To build locally instead, edit `deployment.yaml` back to `spot-champ:local` and run:

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
