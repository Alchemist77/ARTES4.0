import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import tf2_ros
import tf2_geometry_msgs

class TestFixedPersonPublisher(Node):
    def __init__(self):
        super().__init__('test_fixed_person_publisher')

        # Parameters
        self.declare_parameter('target_frame', 'base_link')
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value

        # TF Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publisher
        self.publisher = self.create_publisher(PointCloud2, '/people_obstacles', 10)

        # Timer to simulate publishing people detections
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Test input: from camera_link frame
        self.test_point = [0.327, 0.022, 1.910]
        self.source_frame = 'camera_link'

        self.get_logger().info(f"Publishing test point from '{self.source_frame}' to '{self.target_frame}'...")

    def timer_callback(self):
        try:
            # Look up the transform from camera to base_link/map
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )

            pt = PointStamped()
            pt.header.frame_id = self.source_frame
            pt.header.stamp = self.get_clock().now().to_msg()
            pt.point.x, pt.point.y, pt.point.z = self.test_point

            transformed_pt = tf2_geometry_msgs.do_transform_point(pt, transform)
            self.get_logger().info(f"Transformed point: {transformed_pt.point.x:.2f}, {transformed_pt.point.y:.2f}")

            # Flatten for costmap
            points = [[transformed_pt.point.x, transformed_pt.point.y, 0.0]]

            # Create PointCloud2 message
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.target_frame

            fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
            ]

            pc2_msg = pc2.create_cloud(header, fields, points)
            self.publisher.publish(pc2_msg)

        except Exception as e:
            self.get_logger().warn(f"Transform failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TestFixedPersonPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

