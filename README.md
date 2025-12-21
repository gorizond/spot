# gorizond/spot

Robot stack for SpotMicro/Spot running **ROS 2 Kilted** on **k3s (Raspberry Pi)**, deployed via **Rancher Fleet** (GitOps).

This repo is intentionally minimal: `fleet.yaml` + `deployment.yaml`.

## What runs

`deployment.yaml` deploys a `ros2-smoke` **DaemonSet** (one pod per robot node):

- `publisher` / `subscriber` smoke test on `/chatter`
- `node-label-config` publishes PCA9685 servo mapping from **Kubernetes Node labels** to ROS 2 topic `/spot/config/servo_map` (no ConfigMaps)

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
  - `...chN-min-us` (default: `1000`)
  - `...chN-center-us` (default: `1500`)
  - `...chN-max-us` (default: `2000`)
  - `...chN-invert` (`0`/`1`, default: `0`)

For the current SpotMicro wiring, **CH6–CH9 are empty** (leave `ch6-joint..ch9-joint` unset).

Example (CLI):

```bash
kubectl label node spot \
  gorizond.io/spot-pca9685-i2c-bus=1 \
  gorizond.io/spot-pca9685-address=0x40 \
  gorizond.io/spot-pca9685-ch0-joint=lf_hip \
  gorizond.io/spot-pca9685-ch0-min-us=1000 \
  gorizond.io/spot-pca9685-ch0-center-us=1500 \
  gorizond.io/spot-pca9685-ch0-max-us=2000 \
  --overwrite
```

When labels change, `node-label-config` updates `/spot/config/servo_map` **without restarting the pod**.

## Verify (on the robot cluster)

- DaemonSet is running: `ros2-smoke` (namespace `spot-system`)
- Subscriber logs show `hello-from-spot`
- Node-label config logs show mapping publishes
- Topic contains JSON mapping:

```bash
ros2 topic echo /spot/config/servo_map --once
```
