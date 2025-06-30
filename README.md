# MoCap ROS driver and ICP Pose Estimator 

This ROS 2 project implements a full pipeline to estimate 6-DoF poses using the **Iterative Closest Point (ICP)** algorithm, combining **C++ (PCL)** for backend registration with **Python**-based 3D data streaming over UDP. The system is designed for real-time localization of robots or markers based on raw point clouds.

> This project was developed in parallel with the [VirtualMoCap](https://github.com/loolirer/VirtualMoCap) project at the same research lab.
> While VirtualMoCap provides a simulated multi-agent motion capture system, this project focuses on pose estimation through geometric alignment.

### ✅ Features

* 📡 **`mocap_ros_driver`** (Python): Receives point clouds via UDP and publishes them as `sensor_msgs/msg/PointCloud2`.
* 📐 **`poses_estimator`** (C++): Runs ICP between the received cloud and reference clouds defined in YAML.
* 🧪 Includes a test publisher (`moving_cloud_pub.py`) for synthetic motion.
* 📦 Clean ROS 2 package structure with Docker support.
* 🖼️ RViz2 visualization integrated via launch file.

---

## 🧱 Project Structure

```
.
├── Dockerfile
├── entrypoint.sh
├── src/
│   ├── mocap_ros_driver/      # Python UDP-to-PointCloud2 node
│   │   ├── mocap_ros_driver/  # Python module
│   │   └── setup.py, package.xml, etc.
│   └── poses_estimator/       # C++ ICP implementation
│       ├── src/               # C++ code
│       ├── config/            # YAML clouds and RViz config
│       ├── launch/            # ROS 2 launch files
│       ├── test/              # Dynamic cloud publisher for testing
│       └── package.xml, CMakeLists.txt, README.md
```

---

## 🐳 Docker Setup

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

---

## Access the Running Container

From a second terminal:

```bash
docker exec -it icp_container bash
```

---

## Test Publisher

To simulate point clouds in motion (useful for development or demo):

```bash
ros2 run poses_estimator moving_cloud_pub
```

---

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

---

## Packages

### `mocap_ros_driver`

* Language: Python
* Role: Receives and publishes `PointCloud2` via UDP.

### `poses_estimator`

* Language: C++
* Role: Receives cloud, performs ICP using PCL, publishes resulting transforms.
* Includes: test publishers, RViz config, and YAML cloud references.

---

Let me know if you'd like this split into multiple `README.md` files per package, or if you want it in Markdown format with collapsible sections, badges, or GitHub Actions CI integration.

## Related Project

* **VirtualMoCap** – Simulated motion capture framework developed in the same lab: [https://github.com/loolirer/VirtualMoCap](https://github.com/loolirer/VirtualMoCap)
