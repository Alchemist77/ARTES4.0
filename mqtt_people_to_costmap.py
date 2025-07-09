#!/usr/bin/env python3
"""
mqtt_people_to_costmap.py
Publish PointCloud2 obstacles for people detection **continuously**:

* Whenever an MQTT detection message arrives → publish the converted points.
* If no detection arrives for `empty_publish_period` seconds → publish an
  **empty** PointCloud2 so Nav2 can clear the previous obstacles.

Author: <you>
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
from nav2_msgs.srv import ClearEntireCostmap

class MqttPeopleToCostmap(Node):
    def __init__(self):
        super().__init__("mqtt_people_to_costmap")

        self.declare_parameter("target_frame", "base_link")
        self.target_frame = (
            self.get_parameter("target_frame").get_parameter_value().string_value
        )

        self.empty_publish_period = 0.5   # seconds (2 Hz, same as expected_update_rate)

        self.publisher = self.create_publisher(PointCloud2, "/people_obstacles", 10)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            "/mqtt_converter/people_detection",
            self.mqtt_callback,
            10,
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Costmap clear service clients
        self.local_clear_client = self.create_client(
            ClearEntireCostmap,
            "/local_costmap/clear_entirely_local_costmap"
        )
        self.global_clear_client = self.create_client(
            ClearEntireCostmap,
            "/global_costmap/clear_entirely_global_costmap"
        )

        self.last_detection_stamp = self.get_clock().now()
        self.empty_timer = self.create_timer(
            self.empty_publish_period, self.publish_empty_if_needed
        )

    def clear_costmaps(self):
        req = ClearEntireCostmap.Request()  # Empty request for your nav2 version
        # Fire and forget; don't block main thread (async call)
        self.local_clear_client.call_async(req)
        self.global_clear_client.call_async(req)

    def mqtt_callback(self, msg: Float32MultiArray) -> None:
        data = msg.data
        if len(data) % 4 != 0:
            print(f"[WARN] Invalid data size: expected multiple of 4, got {len(data)}")
            return

        points = []
        for i in range(0, len(data), 4):
            try:
                _, x, y, z = data[i : i + 4]
                pt = PointStamped()
                pt.header.frame_id = "camera_link"
                pt.header.stamp = self.get_clock().now().to_msg()
                pt.point.x, pt.point.y, pt.point.z = x, y, z
                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.target_frame,
                        "camera_link",
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.3),
                    )
                    transformed = tf2_geometry_msgs.do_transform_point(pt, transform)
                    tx, ty = transformed.point.z, transformed.point.x
                except Exception as exc:
                    tx, ty = z, x
                # Instead of one point, publish a 3x3 grid (each offset by 0.05m)
                patch_size = 3  # 3x3 grid
                step = 0.05     # 5cm grid
                for dx in range(-1, 2):
                    for dy in range(-1, 2):
                        points.append([
                            tx + dx * step,
                            -ty + dy * step,
                            1.5
                        ])
            except Exception as e:
                print(f"[ERROR] Error processing person {i//4}: {e}")

        # **Clear costmaps BEFORE publishing new detections**
        self.clear_costmaps()

        # Only publish current detections
        self.publish_cloud(points)
        if points:
            self.last_detection_stamp = self.get_clock().now()

    def publish_empty_if_needed(self) -> None:
        now = self.get_clock().now()
        elapsed_sec = (now - self.last_detection_stamp).nanoseconds * 1e-9
        if elapsed_sec >= self.empty_publish_period:
            print("[EMPTY] No people detected in period, clearing costmaps and obstacles.")
            self.clear_costmaps()
            self.publish_cloud([])

    def publish_cloud(self, points) -> None:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.target_frame

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        if len(points) == 0:
            cloud = PointCloud2()
            cloud.header = header
            cloud.height = 1
            cloud.width = 0
            cloud.fields = fields
            cloud.is_bigendian = False
            cloud.point_step = 12
            cloud.row_step = 0
            cloud.data = []
            cloud.is_dense = False
        else:
            cloud = pc2.create_cloud(header, fields, points)

        self.publisher.publish(cloud)
        if points:
            print(f"[PUBLISH] Published {len(points)} person points to /people_obstacles.")
        else:
            print("[PUBLISH] Published EMPTY cloud (clearing obstacles).")

def main(args=None):
    rclpy.init(args=args)
    node = MqttPeopleToCostmap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

