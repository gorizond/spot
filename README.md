# gorizond/spot

Robot stack for SpotMicro/Spot running **ROS 2 Kilted** on **k3s (Raspberry Pi)**, deployed via **Rancher Fleet** (GitOps).

This repo is intentionally minimal.

## What runs

`deployment.yaml` deploys two DaemonSets (one pod per robot node):

- `ros2-smoke` (core):
  - `node-label-config`: reads **Kubernetes Node labels** of *its own node* (RBAC-enabled), converts them into a PCA9685 servo mapping, and publishes it to `/spot/config/servo_map` (reacts to label changes via watch; no pod restart).
  - `servo-driver`: subscribes to `/spot/config/servo_map`, drives **PCA9685** over I2C (`/dev/i2c-1`), accepts JSON commands on `/spot/cmd/servo`, and publishes status to `/spot/state/servo` (runs privileged for I2C access).
- `spot-champ` (optional):
  - `champ-controller`: runs CHAMP gait controller (`joint_states`, `/cmd_vel`, etc).
  - `champ-bridge`: bridges CHAMP `joint_states` to `servo-driver` commands on `/spot/cmd/servo`.

Safety defaults:

- starts **disarmed** (`START_ARMED=0`)
- clamps each joint to `min/center/max` from node labels
- hard safety clamp via `SAFE_MIN_US`/`SAFE_MAX_US` (defaults: `500..2500`)
- slew-rate limiting (`MAX_SLEW_US_PER_S`)

Pods are scheduled only on nodes matching:

- `kubernetes.io/arch=arm64`
- `gorizond.io/robot=true`

## Hardware notes (power)

- Power the RPi4 from a dedicated `5.1–5.2V` regulator (>= `3A`), ideally from the 2S battery directly.
- Keep servo power (`PCA9685 V+` / servos) on a separate `6V` BEC; tie grounds at a star point.
- Brownouts often show up as boot-loops; measure 5V at the GPIO 5V/GND pins under load.

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

3) Source ROS 2 environment (required for `rclpy` / `ros2` commands):

```bash
source /opt/ros/kilted/setup.bash
```

4) (Optional) Reset targets to home (0.0):

```bash
python3 /opt/spot/spot_cli.py --repeat 1 home
```

5) Arm (does not move anything until you `set` a joint):

```bash
python3 /opt/spot/spot_cli.py --repeat 1 arm
```

6) Enable + move one joint with small steps (calibration-friendly):

```bash
python3 /opt/spot/spot_cli.py --repeat 1 set-us rf_hip=1500
python3 /opt/spot/spot_cli.py --repeat 1 set-us rf_hip=1510
python3 /opt/spot/spot_cli.py --repeat 1 set-us rf_hip=1500
```

7) Disarm when done (note: may drop torque if `DISARM_FULL_OFF=1`):

```bash
python3 /opt/spot/spot_cli.py disarm
```

If something goes wrong:

```bash
python3 /opt/spot/spot_cli.py estop
```

## Stand + micro-steps

After calibration, a conservative **stand** pose is:

```bash
python3 /opt/spot/spot_cli.py --repeat 1 stand
```

Defaults (override via flags or env `STAND_HIP/STAND_UPPER/STAND_LOWER/STAND_HIP_REAR_OFFSET`):

- `hip=0.02`
- `upper=0.05`
- `lower=0.05`
- `rear_hip_offset=0.0` (applied to `rh/lh` only)

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

This uses CHAMP (model-based gait controller) and bridges its `joint_states` to the low-level `servo-driver`.

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
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.05}}' -r 2
```

Notes:

- `champ-bridge` publishes servo targets only when `servo-driver` is `armed=true`.
- Tune bridge scaling via env on `champ-bridge`: `CHAMP_GAIN`, `CHAMP_*_RANGE_RAD`, `STAND_*`.

## Verify

- DaemonSets are running in namespace `spot-system` (`ros2-smoke`, `spot-champ`)
- Logs:
  - `node-label-config` prints which channels are mapped
  - `servo-driver` prints PCA9685 connect + updates
- ROS topics:

```bash
ros2 topic echo /spot/config/servo_map --once
ros2 topic echo /spot/state/servo --once
```

## Command format

`/spot/cmd/servo` expects `std_msgs/String` JSON:

- arm/disarm: `{"cmd":"arm","value":true}` / `{"cmd":"arm","value":false}` (arming gates output; joints move only after they are enabled via `cmd=set`)
- estop: `{"cmd":"estop","value":true}` / `{"cmd":"estop","value":false}`
- home: `{"cmd":"home"}`
- set targets (also enables those joints):
  - normalized: `{"cmd":"set","mode":"norm","targets":{"rf_hip":0.1}}` (range `-1..1`)
  - microseconds: `{"cmd":"set","mode":"us","targets":{"rf_hip":1500}}`

For convenience inside the pod, use `python3 /opt/spot/spot_cli.py ...`.
