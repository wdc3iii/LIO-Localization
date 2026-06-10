#!/usr/bin/env python3
"""Relay node: livox_ros_driver2/CustomMsg -> sensor_msgs/PointCloud2.

GLIM subscribes to PointCloud2 but the Mid360 driver publishes CustomMsg. This
relay republishes /livox/lidar as /livox/points with per-point fields
x, y, z, intensity, time -- GLIM reads the 'time' field for deskewing.

If the Livox driver is configured with xfer_format: 0 (PointCloud2 output)
at the source, this relay is unnecessary.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField

from livox_ros_driver2.msg import CustomMsg


POINT_DTYPE = np.dtype(
    [("x", np.float32), ("y", np.float32), ("z", np.float32),
     ("intensity", np.float32), ("time", np.float32)],
    align=False,
)


def make_fields():
    fields = []
    offsets = {"x": 0, "y": 4, "z": 8, "intensity": 12, "time": 16}
    for name, off in offsets.items():
        fields.append(PointField(name=name, offset=off, datatype=PointField.FLOAT32, count=1))
    return fields


class LivoxRelay(Node):
    def __init__(self):
        super().__init__("livox_custom_to_pc2")
        self.declare_parameter("input_topic", "/livox/lidar")
        self.declare_parameter("output_topic", "/livox/points")
        self.declare_parameter("frame_id", "")

        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value
        self.override_frame = self.get_parameter("frame_id").value

        self.fields = make_fields()
        self.pub = self.create_publisher(PointCloud2, out_topic, qos_profile_sensor_data)
        self.sub = self.create_subscription(
            CustomMsg, in_topic, self.cb, qos_profile_sensor_data
        )
        self.get_logger().info(f"relaying {in_topic} -> {out_topic}")

    def cb(self, msg: CustomMsg):
        n = msg.point_num
        if n == 0:
            return

        arr = np.empty(n, dtype=POINT_DTYPE)
        pts = msg.points
        arr["x"] = [p.x for p in pts]
        arr["y"] = [p.y for p in pts]
        arr["z"] = [p.z for p in pts]
        arr["intensity"] = [p.reflectivity for p in pts]
        arr["time"] = np.fromiter(
            (p.offset_time for p in pts), dtype=np.float32, count=n
        ) * 1e-9

        pc = PointCloud2()
        pc.header = msg.header
        if self.override_frame:
            pc.header.frame_id = self.override_frame
        pc.height = 1
        pc.width = n
        pc.is_dense = True
        pc.is_bigendian = False
        pc.fields = self.fields
        pc.point_step = POINT_DTYPE.itemsize
        pc.row_step = pc.point_step * n
        pc.data = arr.tobytes()
        self.pub.publish(pc)


def main():
    rclpy.init()
    node = LivoxRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
