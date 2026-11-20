# SPDX-License-Identifier: Apache-2.0
"""Run the installed LeSTA launch against synthetic PointCloud2, TF and /clock."""

import argparse
import math
import os
import signal
import struct
import subprocess
import tempfile
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import TransformStamped
from grid_map_msgs.msg import GridMap
from lesta_interfaces.srv import SaveTrainingData
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


OFFSET_X = 320.0
OFFSET_Y = -180.0
PREDICTION_LAYER = "traversability/probability"
MAPPING_LAYER = "mapping/probability"


def transform(parent, child, x=0.0, y=0.0, z=0.0):
    message = TransformStamped()
    message.header.frame_id = parent
    message.child_frame_id = child
    message.transform.translation.x = x
    message.transform.translation.y = y
    message.transform.translation.z = z
    message.transform.rotation.w = 1.0
    return message


def cloud():
    message = PointCloud2()
    message.header.frame_id = "lidar"
    message.height = 1
    message.is_dense = True
    message.fields = [
        PointField(name=name, offset=4 * index, datatype=PointField.FLOAT32, count=1)
        for index, name in enumerate(("x", "y", "z", "intensity"))
    ]
    points = [
        (i * 0.1, j * 0.1, -1 + 0.02 * math.sin(i * 0.2) + 0.02 * math.cos(j * 0.2), 0.2)
        for i in range(-14, 15)
        for j in range(-14, 15)
    ]
    message.width = len(points)
    message.point_step = 16
    message.row_step = 16 * message.width
    message.data = b"".join(struct.pack("<ffff", *point) for point in points)
    return message


def has_scores(message, layer, minimum, maximum):
    if layer not in message.layers:
        return False
    values = message.data[message.layers.index(layer)].data
    return sum(minimum <= value <= maximum for value in values) >= 8


def check_pcd(path):
    text = path.read_text()
    header, data = text.split("DATA ascii\n", maxsplit=1)
    fields = next(line.split()[1:] for line in header.splitlines() if line.startswith("FIELDS "))
    assert {"step", "slope", "roughness", "curvature", "traversability_label"} <= set(fields)
    assert len(data.strip().splitlines()) >= 8


def run_mode(mode, directory, model_path):
    node = Node(f"lesta_smoke_{mode}")
    broadcaster = StaticTransformBroadcaster(node)
    broadcaster.sendTransform([
        transform("map", "base_link", x=OFFSET_X, y=OFFSET_Y),
        transform("base_link", "lidar", z=1.0),
    ])
    scan_pub = node.create_publisher(
        PointCloud2, "/velodyne/points",
        QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
    )
    clock_pub = node.create_publisher(Clock, "/clock", 10)
    scan = cloud()
    clock = Clock()
    valid_stamps = set()
    if mode == "labels":
        client = node.create_client(SaveTrainingData, "/lesta/save_label_map")
    else:
        def on_map(message):
            center = message.info.pose.position
            if (message.header.frame_id != "map"
                    or not math.isclose(center.x, OFFSET_X, abs_tol=0.15)
                    or not math.isclose(center.y, OFFSET_Y, abs_tol=0.15)):
                return
            if not has_scores(message, PREDICTION_LAYER, 0.73, 0.74):
                return
            if mode == "mapping" and not has_scores(message, MAPPING_LAYER, 0.64, 0.96):
                return
            stamp = message.header.stamp
            valid_stamps.add((stamp.sec, stamp.nanosec))

        node.create_subscription(GridMap, f"/lesta/{mode}/traversability", on_map, 10)

    config = directory / "smoke.yaml"
    config.write_text(
        "/**:\n  ros__parameters:\n    map_length_x: 6.0\n    map_length_y: 6.0\n"
        "    scan_filter_range: 2.0\n    map_publish_rate: 10.0\n    raycasting: false\n"
    )
    command = [
        "ros2", "launch", "lesta", "lesta.launch.py", f"mode:={mode}",
        f"config:={config}", "use_sim_time:=true",
    ]
    if mode != "labels":
        command.append(f"model_path:={model_path}")

    future = None
    next_scan = time.monotonic()
    deadline = next_scan + 40.0
    logfile = directory / f"{mode}.log"
    with logfile.open("w+") as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise RuntimeError(f"{mode} exited with code {process.returncode}")
                now = time.monotonic()
                if now >= next_scan:
                    clock.clock.sec += 1
                    clock_pub.publish(clock)
                    scan.header.stamp = clock.clock
                    scan_pub.publish(scan)
                    next_scan = now + 0.1
                rclpy.spin_once(node, timeout_sec=0.04)
                if mode == "labels":
                    if future is None and client.service_is_ready():
                        request = SaveTrainingData.Request(filename="smoke", directory=str(directory))
                        future = client.call_async(request)
                    if future is not None and future.done():
                        if future.result().success:
                            check_pcd(directory / "smoke.pcd")
                            print("PASS: labels produced a nonempty feature/label PCD")
                            return
                        future = None
                elif len(valid_stamps) >= 3:
                    print(f"PASS: {mode} published scored maps from distinct scans")
                    return
            raise TimeoutError(f"{mode} produced no verified result within 40 seconds")
        except Exception:
            log.seek(0)
            print(log.read()[-8000:])
            raise
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3)
            node.destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, help="synthetic checkpoint generated by make_smoke_model.cpp")
    args = parser.parse_args()
    model_path = args.model.resolve(strict=True) if args.model else None
    rclpy.init()
    try:
        with tempfile.TemporaryDirectory(prefix="lesta-smoke-") as directory:
            run_mode("labels", Path(directory), None)
            if model_path:
                for mode in ("prediction", "mapping"):
                    run_mode(mode, Path(directory), model_path)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
