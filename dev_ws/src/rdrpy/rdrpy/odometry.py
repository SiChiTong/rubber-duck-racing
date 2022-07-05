import rclpy
from rclpy.node import Node

from geometry.msg import Twist
from sensor_msgs.msg import Odometry

import math

import Jetson.GPIO as GPIO

class Encoder(Node):

    def __init__(self):
        super().__init__('encoder')
        self.publisher_ = self.create_publisher(Odometry, '/encoder', 10)
        self.subscriber = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        timer_period = 1 / 60  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(21, GPIO.IN)
        GPIO.add_event_detect(21, GPIO.RISING, callback=self.get_rpm)
        
        self.count = 0
        self.start_time = self.rclpy.Time.now().to_sec()
        self.sample_rate = 20
        self.circumference = 0.1

        self.wheelbase = 0.3
        self.speed = 0.0
        self.motor_angle = 0.0 
        self.angle = 0.0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    def get_rpm(self):
        self.count += 1
        if self.count == self.sample_rate:
            end_time = self.rclpy.Time.now().to_sec()
            delta = self.start_time - end_time
            rps = self.count / delta
            self.speed = rps * self.circumference

    def cmd_vel_callback(self, msg):
        self.motor_angle = msg.angular.z
        self.angle = self.motor_angle * 70

    def timer_callback(self):
        odom = Odometry()
        angular_velocity = self.speed * math.tan(self.angle) / self.wheelbase
        
        x_dot = self.speed * math.cos(self.yaw)
        y_dot = self.speed * math.sin(self.yaw)

        self.x += x_dot * self.timer.duration.to_sec()
        self.y += y_dot * self.timer.duration.to_sec()

        yaw += angular_velocity * self.timer.duration.to_sec()

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2)
        odom.pose.pose.orientation.w = math.cos(yaw / 2)

        odom.twist.twist.linear.x = self.speed
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = angular_velocity

        odom.header.stamp = self.timer.last_called
        self.publisher_.publish(odom)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = Encoder()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()