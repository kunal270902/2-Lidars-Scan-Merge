#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class ScanMerger(Node):
    def __init__(self):
        super().__init__('scan_merger')
        
        # Parameters
        self.declare_parameter('scan_topic1', '/scan1')
        self.declare_parameter('scan_topic2', '/scan2')
        self.declare_parameter('merged_scan_topic', '/merged_scan')
        
        self.scan_topic1 = self.get_parameter('scan_topic1').get_parameter_value().string_value
        self.scan_topic2 = self.get_parameter('scan_topic2').get_parameter_value().string_value
        self.merged_scan_topic = self.get_parameter('merged_scan_topic').get_parameter_value().string_value
        
        # Subscribers
        self.scan1_sub = self.create_subscription(LaserScan, self.scan_topic1, self.scan1_callback, 10)
        self.scan2_sub = self.create_subscription(LaserScan, self.scan_topic2, self.scan2_callback, 10)
        
        # Publisher
        self.merged_scan_pub = self.create_publisher(LaserScan, self.merged_scan_topic, 10)
        
        self.scan1 = None
        self.scan2 = None
    
    def scan1_callback(self, msg):
        self.scan1 = msg
        self.merge_scans()
    
    def scan2_callback(self, msg):
        self.scan2 = msg
        self.merge_scans()
    
    def merge_scans(self):
        if self.scan1 and self.scan2:
            merged_scan = LaserScan()
            merged_scan.header = self.scan1.header
            merged_scan.angle_min = min(self.scan1.angle_min, self.scan2.angle_min)
            merged_scan.angle_max = max(self.scan1.angle_max, self.scan2.angle_max)
            merged_scan.angle_increment = self.scan1.angle_increment 
            merged_scan.time_increment = self.scan1.time_increment
            merged_scan.scan_time = self.scan1.scan_time
            merged_scan.range_min = self.scan1.range_min
            merged_scan.range_max = self.scan1.range_max
            
            ranges1 = np.array(self.scan1.ranges)
            ranges2 = np.array(self.scan2.ranges)
            
            # Convert ignored (zero or NaN) values to inf
            ranges1[ranges1 == 0.0] = float('inf')
            ranges2[ranges2 == 0.0] = float('inf')
            ranges1[np.isnan(ranges1)] = float('inf')
            ranges2[np.isnan(ranges2)] = float('inf')
            
            merged_ranges = np.minimum(ranges1, ranges2)
            
            # Replace inf back with 0 for the merged scan
            merged_ranges[merged_ranges == float('inf')] = 0.0
            
            merged_scan.ranges = merged_ranges.tolist()
            self.merged_scan_pub.publish(merged_scan)

def main(args=None):
    rclpy.init(args=args)
    scan_merger = ScanMerger()
    rclpy.spin(scan_merger)
    scan_merger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

