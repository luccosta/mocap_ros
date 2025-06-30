# MoCap ROS driver and ICP Pose Estimator 

This ROS 2 project implements a full pipeline to estimate 6-DoF poses, combining C++ for backend registration with Python-based data streaming over UDP. The system is designed for real-time localization of robots or markers based on raw point clouds from the MoCap system.

> This project was developed in parallel with the [VirtualMoCap](https://github.com/loolirer/VirtualMoCap) project at the same research lab.
> While VirtualMoCap provides a simulated multi-agent motion capture system, this project focuses on pose estimation through geometric alignment.

## Packages

### `mocap_ros_driver`

* Language: Python
* Role: Publishes `PointCloud2` via UDP.

### `poses_estimator`

* Language: C++
* Role: Receives cloud, performs ICP using PCL, publishes resulting transforms.
* Includes: test publishers, RViz config, and YAML cloud references.

## Docker Setup

### 1. Allow GUI forwarding (once per host)

```bash
echo 'xhost +local:docker' >> ~/.zshrc
# If using bash:
# echo 'xhost +local:docker' >> ~/.bashrc
source ~/.zshrc  # or ~/.bashrc
```

### 2. Build the Docker image

From the root of the repository:

```bash
docker build -t icp_ros2_image .
```

### 3. Run the container

```bash
docker run -it --rm \
  --name icp_container \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -v $(pwd)/src:/ros2_ws/src \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  icp_ros2_image
```

> `src` is mounted to allow live editing.
>
> RViz will launch automatically via `mocap_pose_estimator.launch.py`.

---

## Application Launch

If not launched automatically, run manually inside the container:

```bash
ros2 launch poses_estimator mocap_pose_estimator.launch.py
```

This will launch:

* `mocap_ros_driver.py` – Publishes `PointCloud2` messages via UDP.
* `pointcloud_to_poses.cpp` – Runs ICP with stored references.
* RViz – Displays point clouds and resulting pose transforms.

## Access the Running Container

From a second terminal:

```bash
docker exec -it icp_container bash
```

## Test Publisher

To simulate point clouds in motion (useful for development or demo):

```bash
ros2 run poses_estimator moving_cloud_pub
```

## Reference YAML: `clouds.yaml`

Defined in:
`src/poses_estimator/config/clouds.yaml`

Each entry describes a small 3D point cloud, e.g.:

```yaml
clouds:
  - name: wave_rover_1
    points:
      - [0.0, 0.0, 0.0]
      - [1.0, 0.0, 0.0]
      - [1.0, 1.0, 0.0]
```

These are used as static references for ICP.

## Related Project

* **VirtualMoCap** – Simulated motion capture framework developed in the same lab: [VirtualMoCap](https://github.com/loolirer/VirtualMoCap)
