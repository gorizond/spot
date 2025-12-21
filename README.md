# gorizond/spot

Robot stack for SpotMicro/Spot running **ROS 2 Kilted** on **k3s (Raspberry Pi)**, deployed via **Rancher Fleet** (GitOps).

This repo is intentionally minimal: `fleet.yaml` + `deployment.yaml`.

## Deploy with Fleet

Create a Fleet `GitRepo` in your Fleet workspace (e.g. `workspace-negashev`) pointing to this repository and target your robot cluster.

```yaml
spec:
  repo: https://github.com/gorizond/spot
  branch: main
  targets:
    - clusterName: spots
```

## Verify (on the robot cluster)

- Deployment is running: `ros2-smoke`
- Subscriber logs show `hello-from-spot`
