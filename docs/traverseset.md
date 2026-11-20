# Prepare a TraverseSet recording

A campus example is `campus_nav_01_south-002.bag` from [TraverseSet Urban Campus](https://github.com/Ikhyeon-Cho/TraverseSet#download). The matching inference example uses `LeSTA-campus-road` from the [LeSTA Model Zoo](https://github.com/Ikhyeon-Cho/LeSTA#model-zoo). Download both from the authors' pages.

1. In a ROS 1 playback environment, unpack the VLP-16 packets using the [TraverseSet player](https://github.com/Ikhyeon-Cho/TraverseSet#play) or its [VLP-16 decoder launch](https://github.com/Ikhyeon-Cho/TraverseSet/blob/master/vlp16packet_to_pointcloud.launch). Select one decoded cloud stream and preserve its sensor frame and timestamps. Use the matching calibration and static extrinsic transform from the dataset. Format conversion alone leaves the messages as raw packets.
2. Record or bridge the decoded `PointCloud2`, `/tf`, and `/tf_static`. If you first record an expanded ROS 1 bag, convert that bag with [Rosbags](https://ternaris.gitlab.io/rosbags/topics/convert.html). Confirm in Humble that the resulting ROS 2 bag contains valid TF and a cloud stream. Ensure the static transform is published with transient-local durability if the conversion does not preserve it.
3. In a LeSTA overlay YAML, set `lidarscan_topic` to the cloud's recorded topic, `map_frame` and `robot_frame` to the corresponding TF frames, and the global map dimensions to cover the chosen route. Start LeSTA with `config:=/absolute/path/to/params.yaml use_sim_time:=true` and replay with `ros2 bag play /absolute/path/to/ros2_bag --clock`.

A valid cloud and robot pose must share a time base. Use `ros2 bag info` to confirm the message types and `ros2 run tf2_ros tf2_echo map base_link` to inspect the pose during playback, replacing frames with your configuration.
