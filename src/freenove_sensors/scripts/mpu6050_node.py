#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from mpu6050 import mpu6050 # Library used in the original firmware

class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')
        self.sensor = mpu6050(address=0x68, bus=1)
        self.sensor.set_accel_range(mpu6050.ACCEL_RANGE_2G)
        self.sensor.set_gyro_range(mpu6050.GYRO_RANGE_250DEG)

        # Calibration offset zero-bias (100)
        self.offset_accel, self.offset_gyro = self.calibrate()
        self.pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.timer = self.create_timer(1.0 / 50.0, self.publish_imu) # 50 Hz

    def calibrate(self):
        ax, ay, az, gx, gy, gz = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        for _ in range(100):
            a = self.sensor.get_accel_data()
            g = self.sensor.get_gyro_data()
            ax += a['x']; ay += a['y']; az += a['z']
            gx += g['x']; gy += g['y']; gz += g['z']
        return {'x': ax/100.0, 'y': ay/100.0, 'z': (az/100.0) - 9.81}, {'x': gx/100.0, 'y': gy/100.0, 'z': gz/100.0}

    def publish_imu(self):
        a = self.sensor.get_accel_data()
        g = self.sensor.get_gyro_data()
        
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # Accel in m/s^2, Gyro in rad/s
        imu_msg.linear_acceleration.x = a['x'] - self.offset_accel['x']
        imu_msg.linear_acceleration.y = a['y'] - self.offset_accel['y']
        imu_msg.linear_acceleration.z = a['z'] - self.offset_accel['z']
        imu_msg.angular_velocity.x = (g['x'] - self.offset_gyro['x']) * (3.14159 / 180.0)
        imu_msg.angular_velocity.y = (g['y'] - self.offset_gyro['y']) * (3.14159 / 180.0)
        imu_msg.angular_velocity.z = (g['z'] - self.offset_gyro['z']) * (3.14159 / 180.0)

        self.pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()