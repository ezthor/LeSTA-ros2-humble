# LeSTA ROS 2 Humble

[![ROS 2 Humble build](https://github.com/ezthor/LeSTA-ros2-humble/actions/workflows/humble-build.yml/badge.svg?branch=main)](https://github.com/ezthor/LeSTA-ros2-humble/actions/workflows/humble-build.yml)

A community port of [LeSTA](https://github.com/Ikhyeon-Cho/LeSTA) to ROS 2 Humble. LeSTA learns robot-specific terrain traversability from LiDAR and driving experience. The port uses a single native `rclcpp` node for label generation, local prediction, and global mapping. The Python training pipeline is in [pylesta](pylesta/README.md).

## Build

Use Ubuntu 22.04 x86-64 with ROS 2 Humble installed. Inference uses the CPU build of LibTorch 2.6.0 with the C++11 ABI.

```bash
source /opt/ros/humble/setup.bash
sudo apt-get update
sudo apt-get install -y build-essential cmake git wget unzip \
  python3-colcon-common-extensions libpcl-dev libeigen3-dev \
  ros-humble-grid-map-core ros-humble-grid-map-ros ros-humble-grid-map-msgs \
  ros-humble-pcl-conversions ros-humble-tf2-eigen ros-humble-ros2launch
wget -O /tmp/lesta-libtorch.zip \
  'https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.6.0%2Bcpu.zip'
sudo unzip -q /tmp/lesta-libtorch.zip -d /opt/

mkdir -p ~/lesta_ws/src
git clone https://github.com/ezthor/LeSTA-ros2-humble.git ~/lesta_ws/src/lesta-ros2-humble
cd ~/lesta_ws
export CMAKE_PREFIX_PATH="/opt/libtorch:${CMAKE_PREFIX_PATH:-}"
colcon build --packages-up-to lesta --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
export LD_LIBRARY_PATH="/opt/libtorch/lib:${LD_LIBRARY_PATH:-}"
ros2 pkg executables lesta
```

Expected executable: `lesta lesta_node`. Source `/opt/ros/humble/setup.bash` and `~/lesta_ws/install/setup.bash` and set `LD_LIBRARY_PATH` as above in each new terminal.

## Input and playback

- Point cloud: `/velodyne/points`, type `sensor_msgs/msg/PointCloud2`, with `x`, `y`, `z`, `intensity`, a populated acquisition timestamp and a frame ID.
- TF: a timestamp-compatible chain from `map` to `base_link` and from `base_link` to the cloud's sensor frame.
- For bag replay, publish `/clock` and use `use_sim_time:=true` when launching LeSTA. For live data, leave the default `false`.

Use a ROS 2 bag with the above inputs, or see [Prepare a TraverseSet recording](docs/traverseset.md) for the raw dataset path. Install `ros-humble-rosbag2-storage-mcap` if your bag uses MCAP. Inspect the topics before replaying:

```bash
ros2 bag info /absolute/path/to/ros2_bag
ros2 bag play /absolute/path/to/ros2_bag --clock
```

Start one LeSTA mode per run before playing the bag. `config:=/absolute/path/to/params.yaml` overlays the [built-in mode YAML](lesta_ros/config/ros2/) with an ordinary ROS 2 parameter file. Use it to set `lidarscan_topic`, `map_frame`, `robot_frame`, or map dimensions. For global modes, the map is centered on the first usable scan and has a fixed extent. Choose a map large enough to cover the route.

## Run

| Mode | Output | Type |
| --- | --- | --- |
| `labels` | `/lesta/label_generation/map_labeled` | `sensor_msgs/msg/PointCloud2` |
| `prediction` | `/lesta/prediction/traversability` | `grid_map_msgs/msg/GridMap` |
| `mapping` | `/lesta/mapping/traversability` | `grid_map_msgs/msg/GridMap` |

### Generate labels

```bash
ros2 launch lesta lesta.launch.py mode:=labels use_sim_time:=true
```

After valid terrain has been processed, call the service from another sourced terminal. It writes `/tmp/lesta-labels/training_set.pcd` with the terrain features and `traversability_label`.

```bash
ros2 service call /lesta/save_label_map lesta_interfaces/srv/SaveTrainingData \
  '{filename: training_set, directory: /tmp/lesta-labels}'
```

### Predict or map traversability

Supply a TorchScript `.pt` checkpoint for the four default features, in the order step, slope, roughness, curvature. Use the [upstream Model Zoo](https://github.com/Ikhyeon-Cho/LeSTA#model-zoo) or [train and export your own](pylesta/README.md).

```bash
ros2 launch lesta lesta.launch.py mode:=prediction \
  model_path:=/absolute/path/to/model.pt use_sim_time:=true
# Run mapping separately.
ros2 launch lesta lesta.launch.py mode:=mapping \
  model_path:=/absolute/path/to/model.pt use_sim_time:=true
```

Check the topic with `ros2 topic hz /lesta/mapping/traversability`. In the GridMap, `traversability/probability` contains per-scan predictions and `mapping/probability` contains the fused result in mapping mode. A usable replay produces successive maps with populated probability cells while the bag is playing.

## Validation

The Humble workflow builds the three ROS 2 packages, then starts the installed launch in each mode with synthetic LiDAR points, TF, and a replay clock. It checks the nonempty label PCD, TorchScript inference, and sustained GridMap publications with the first scan far from the coordinate origin. See [the workflow](.github/workflows/humble-build.yml) for the exact checks. A real-data replay with TraverseSet and the upstream checkpoint is a separate validation step.

## License and citation

This port is based on [upstream LeSTA at `e5614b0`](https://github.com/Ikhyeon-Cho/LeSTA/tree/e5614b010966ab2b38dd7d30f090940fde40e079), using Apache-2.0. See [LICENSE](LICENSE) and [NOTICE](NOTICE). For research, cite the original [LeSTA paper](https://doi.org/10.1109/LRA.2024.3376148) by Ikhyeon Cho and Woojin Chung. Citation metadata is in [CITATION.cff](CITATION.cff).
