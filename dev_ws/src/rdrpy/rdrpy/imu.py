from .submodules.mpu6050 import mpu6050

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from tf.transformations import *



class IMU(Node):

    def __init__(self):
        super().__init__('imu')
        self.publisher_ = self.create_publisher(Imu, 'topic', 10)
        timer_period = 1 / 1000  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.mpu = mpu6050(0x68)
        self.prevrot = quaternion_from_euler(0, 0, 0)

    def timer_callback(self):
        msg = Imu()
        msg.header.stamp = rclpy.Time.now()

        

        accel_data = self.mpu.get_accel_data()
        gyro_data = self.mpu.get_gyro_data()
        msg.linear_acceleration.x = accel_data['x']
        msg.linear_acceleration.y = accel_data['y']
        msg.linear_acceleration.z = accel_data['z']
        msg.angular_velocity.x = gyro_data['x']
        msg.angular_velocity.y = gyro_data['y']
        msg.angular_velocity.z = gyro_data['z']

        q_rot = quaternion_from_euler(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        msg.orientation = quaternion_multiply(q_rot, self.prevrot)

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = IMU()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()