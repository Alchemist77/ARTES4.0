import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Pose
import numpy as np
import math

class GlobalCostmapMerger(Node):
    def __init__(self):
        super().__init__('global_costmap_merger')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.costmap1 = None
        self.costmap2 = None

        self.sub1 = self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', self.cb1, 10)
        self.sub2 = self.create_subscription(
            OccupancyGrid, '/elena/local_costmap/costmap', self.cb2, 10)

        self.pub = self.create_publisher(OccupancyGrid, '/merged_costmap', 10)
        self.timer = self.create_timer(1.0, self.merge_and_publish)  # Merge every second

    def cb1(self, msg):
        self.costmap1 = msg

    def cb2(self, msg):
        self.costmap2 = msg

    def fixed_odom_origin(self, origin_pose: Pose, from_frame):
        """If from_frame == 'odom', return fixed static pose rotated 90deg right. Else, original pose."""
        if from_frame.lstrip('/') == 'odom':
            x_fixed = -3.6 + origin_pose.position.x
            y_fixed = -5.56 + origin_pose.position.y

            # 1. Original orientation (as quaternion list)
            q_orig = [
                origin_pose.orientation.x,
                origin_pose.orientation.y,
                origin_pose.orientation.z,
                origin_pose.orientation.w,
            ]

            # 2. Quaternion for 90 deg yaw (counterclockwise, i.e., left)
            angle_rad = math.pi/2      # +90° left (CCW)
            # For -90° (right), use: angle_rad = -math.pi/2
            q_yaw = [0.0, 0.0, math.sin(angle_rad/2), math.cos(angle_rad/2)]  # [x, y, z, w]

            # 3. Multiply: q_new = q_yaw * q_orig
            q_new = quaternion_multiply(q_yaw, q_orig)

            orientation_fixed = Pose().orientation
            orientation_fixed.x = q_new[0]
            orientation_fixed.y = q_new[1]
            orientation_fixed.z = q_new[2]
            orientation_fixed.w = q_new[3]
            self.get_logger().info(
                f"[FIXED, 90deg right] odom -> map: pos=({x_fixed}, {y_fixed}), orientation=(0,0,-0.7071,0.7071)"
            )
            return x_fixed, y_fixed, orientation_fixed
        else:
            return origin_pose.position.x, origin_pose.position.y, origin_pose.orientation

    def blit_to_grid(self, dest, data, x_offset, y_offset, w, h):
        for y in range(h):
            for x in range(w):
                dx = x + x_offset
                dy = y + y_offset
                if 0 <= dx < dest.shape[1] and 0 <= dy < dest.shape[0]:
                    val = data[y * w + x]
                    if val >= 0:
                        dest[dy, dx] = max(dest[dy, dx], val)

    def merge_and_publish(self):
        self.get_logger().info("Merging costmaps...")

        if self.costmap1 is None or self.costmap2 is None:
            return

        cm1 = self.costmap1.info
        cm2 = self.costmap2.info

        # Use fixed pose for odom frame, otherwise as-is
        ox1, oy1, orient1 = self.fixed_odom_origin(cm1.origin, self.costmap1.header.frame_id)
        print("cm1.origin", cm1.origin)
        print("self.costmap1.header.frame_id", self.costmap1.header.frame_id)
        ox2, oy2, orient2 = self.fixed_odom_origin(cm2.origin, self.costmap2.header.frame_id)

        resolution = cm1.resolution
        w1, h1 = cm1.width, cm1.height
        w2, h2 = cm2.width, cm2.height

        min_x = min(ox1, ox2)
        min_y = min(oy1, oy2)
        max_x = max(ox1 + w1 * resolution, ox2 + w2 * resolution)
        max_y = max(oy1 + h1 * resolution, oy2 + h2 * resolution)

        merged_width = int(np.ceil((max_x - min_x) / resolution))
        merged_height = int(np.ceil((max_y - min_y) / resolution))

        x_off1 = int((ox1 - min_x) / resolution)
        y_off1 = int((oy1 - min_y) / resolution)
        x_off2 = int((ox2 - min_x) / resolution)
        y_off2 = int((oy2 - min_y) / resolution)

        self.get_logger().info(
            f"Merged origin: ({min_x}, {min_y}), size: {merged_width}x{merged_height}, "
            f"offsets: CM1({x_off1},{y_off1}), CM2({x_off2},{y_off2})"
        )

        merged_grid = np.full((merged_height, merged_width), -1, dtype=np.int8)

        self.blit_to_grid(merged_grid, self.costmap1.data, x_off1, y_off1, w1, h1)
        self.blit_to_grid(merged_grid, self.costmap2.data, x_off2, y_off2, w2, h2)

        merged = OccupancyGrid()
        merged.header.stamp = self.get_clock().now().to_msg()
        merged.header.frame_id = 'map'
        merged.info.resolution = resolution
        merged.info.width = merged_width
        merged.info.height = merged_height
        merged.info.origin.position.x = min_x
        merged.info.origin.position.y = min_y
        merged.info.origin.position.z = 0.0
        merged.info.origin.orientation = Pose().orientation
        merged.info.origin.orientation.w = 1.0
        merged.data = list(merged_grid.flatten())

        self.pub.publish(merged)

def main():
    rclpy.init()
    node = GlobalCostmapMerger()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

