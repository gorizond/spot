# gorizond/spot

Robot stack for SpotMicro/Spot running **ROS 2 Kilted** on **k3s (Raspberry Pi)**, deployed via **Rancher Fleet** (GitOps).

This repo is intentionally structured as a set of Fleet bundles under `bundles/`.

## Bundles

- `bundles/ros2-smoke`: minimal ROS 2 pub/sub smoke test (single Pod, 2 containers).

Why a single Pod? ROS 2 DDS multicast/discovery often behaves poorly across Kubernetes Pods/overlays; starting with a single Pod gives a deterministic “ROS is alive” signal.

## Deploy with Fleet

Create a Fleet `GitRepo` in your Fleet workspace (e.g. `workspace-negashev`) pointing to this repository and target your robot cluster.

Example `spec` (snippet):

```yaml
spec:
  repo: https://github.com/gorizond/spot
  branch: main
  targets:
    - clusterName: spots
```

## Verify (on the robot cluster)

- Namespace exists: `spot-system`
- Pod is running: `ros2-smoke-*`
- Subscriber logs show `hello-from-spot`

