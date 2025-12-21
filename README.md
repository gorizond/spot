# gorizond/spot

Robot stack for SpotMicro/Spot running **ROS 2 Kilted** on **k3s (Raspberry Pi)**, deployed via **Rancher Fleet** (GitOps).

This repo is intentionally minimal.

## What runs

`deployment.yaml` deploys a DaemonSet (one pod per robot node) with:

- `node-label-config`: reads **Kubernetes Node labels** of *its own node* (RBAC-enabled), converts them into a PCA9685 servo mapping, and publishes it to `/spot/config/servo_map` (reacts to label changes via watch; no pod restart).
- `servo-driver`: subscribes to `/spot/config/servo_map`, drives **PCA9685** over I2C (`/dev/i2c-1`), accepts JSON commands on `/spot/cmd/servo`, and publishes status to `/spot/state/servo` (runs privileged for I2C access).

Safety defaults:

- starts **disarmed** (`START_ARMED=0`)
- clamps each joint to `min/center/max` from node labels
- hard safety clamp via `SAFE_MIN_US`/`SAFE_MAX_US` (defaults: `500..2500`)
- slew-rate limiting (`MAX_SLEW_US_PER_S`)

Pods are scheduled only on nodes matching:

- `kubernetes.io/arch=arm64`
- `gorizond.io/robot=true`

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

3) (Optional) Reset targets to home (0.0):

```bash
python3 /opt/spot/spot_cli.py --repeat 1 home
```

4) Arm (does not move anything until you `set` a joint):

```bash
python3 /opt/spot/spot_cli.py --repeat 1 arm
```

5) Enable + move one joint with small steps (calibration-friendly):

```bash
python3 /opt/spot/spot_cli.py --repeat 1 set-us rf_hip=1500
python3 /opt/spot/spot_cli.py --repeat 1 set-us rf_hip=1510
python3 /opt/spot/spot_cli.py --repeat 1 set-us rf_hip=1500
```

6) Disarm when done:

```bash
python3 /opt/spot/spot_cli.py disarm
```

If something goes wrong:

```bash
python3 /opt/spot/spot_cli.py estop
```

## Verify

- DaemonSet is running in namespace `spot-system`
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
