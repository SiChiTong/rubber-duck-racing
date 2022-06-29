import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from adafruit_servokit import ServoKit

class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')
        self.kit = ServoKit(channels=16)
        self.sub_cmd_vel = self.create_subscription(Twist, 'cmd_vel', self.listener_callback_cmd_vel, 10)
    def listener_callback_cmd_vel(self, twist):
        throttle = twist.linear.y * 90 + 90
        steering = twist.angular.z * 90 + 90
        self.kit.servo[0].angle = throttle
        self.kit.servo[1].angle = steering

def main(args=None):
    rclpy.init(args=args)

    motor_driver = MotorDriver()

    rclpy.spin(motor_driver)

    motor_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
