# gorizond/spot

Robot stack for SpotMicro/Spot running **ROS 2 Kilted** on **k3s (Raspberry Pi)**, deployed via **Rancher Fleet** (GitOps).

This repo is intentionally minimal.

## What runs

`deployment.yaml` deploys a DaemonSet (one pod per robot node) that:

- reads **Kubernetes Node labels** of *its own node* (RBAC-enabled)
- converts them into a PCA9685 servo mapping
- publishes mapping to ROS 2 topic `/spot/config/servo_map`
- reacts to label changes via Kubernetes **watch** (no pod restart)

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

## Verify

- DaemonSet is running in namespace `spot-system`
- Logs:
  - `node-label-config` prints which channels are mapped
- ROS topic contains JSON mapping:

```bash
ros2 topic echo /spot/config/servo_map --once
```
