#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from gpiozero import DistanceSensor

class HCSR04Node(Node):
    def __init__(self):
        super().__init__('hcsr04_node')
        self.declare_parameter('trigger_pin', 27)
        self.declare_parameter('echo_pin', 22)
        
        trig = self.get_parameter('trigger_pin').value
        echo = self.get_parameter('echo_pin').value
        
        # Inizialize sensor as original firmware
        self.sensor = DistanceSensor(echo=echo, trigger=trig, max_distance=3.0)
        self.pub = self.create_publisher(LaserScan, '/ultrasonic/scan', 10)
        self.timer = self.create_timer(1.0 / 15.0, self.publish_scan) # 15 Hz

    def publish_scan(self):
        dist_m = float(self.sensor.distance)
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'ultrasonic_link'
        scan.angle_min = -0.15
        scan.angle_max = 0.15
        scan.angle_increment = 0.3
        scan.range_min = 0.02
        scan.range_max = 4.0
        scan.ranges = [dist_m]
        self.pub.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = HCSR04Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()