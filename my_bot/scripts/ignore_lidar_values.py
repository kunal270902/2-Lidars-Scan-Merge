#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class IgnoreLidarValuesNode(Node):
    def __init__(self):
        super().__init__('ignore_lidar_values')

        # Create subscribers for the scan1 and scan2 topics
        self.scan1_subscriber = self.create_subscription(
            LaserScan,
            '/scan1',  # Listening to scan1 topic
            self.scan1_callback,
            10  # QoS profile depth
        )
        self.scan2_subscriber = self.create_subscription(
            LaserScan,
            '/scan2',  # Listening to scan2 topic
            self.scan_callback,
            10  # QoS profile depth
        )

        # Create publishers for the filtered scan1 and scan2 topics
        self.scan1_publisher = self.create_publisher(
            LaserScan,
            '/scan1',  # Republish on a different topic
            10
        )
        self.scan2_publisher = self.create_publisher(
            LaserScan,
            '/scan2',  # Republish on a different topic
            10
        )

        self.get_logger().info('Subscribed to scan1 and scan2 topics')

    def scan1_callback(self, msg):
        self.new_lidar = msg
        total_values = len(self.new_lidar.ranges)
        angle_increment = msg.angle_increment
        angle_min = msg.angle_min

        # Define the angle range to ignore for scan1 (in radians)
        ignore_angle_min1 = -1.5708  # -90 degrees
        ignore_angle_max1 = 1.5708  # 90 degrees

        for i in range(total_values):
            angle = angle_min + i * angle_increment
            if ignore_angle_min1 <= angle <= ignore_angle_max1:
                self.new_lidar.ranges[i] = 0.0

        self.scan1_publisher.publish(self.new_lidar)

    def scan_callback(self, msg):
        self.new_lidar = msg
        total_values = len(self.new_lidar.ranges)
        angle_increment = msg.angle_increment
        angle_min = msg.angle_min

        # Define the angle range to ignore for scan2 (in radians)
        ignore_angle_min1 = 1.5708  # 90 degrees
        ignore_angle_max1 = 3.14159  # 180 degrees
        ignore_angle_min2 = -3.14159  # -180 degrees
        ignore_angle_max2 = -1.5708  # -90 degrees

        for i in range(total_values):
            angle = angle_min + i * angle_increment
            if (ignore_angle_min1 <= angle <= ignore_angle_max1) or (ignore_angle_min2 <= angle <= ignore_angle_max2):
                self.new_lidar.ranges[i] = 0.0

        self.scan2_publisher.publish(self.new_lidar)

def main(args=None):
    rclpy.init(args=args)
    node = IgnoreLidarValuesNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

