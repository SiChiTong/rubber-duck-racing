PWR_MGMT_1 = 0x6b

ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B
ACCEL_XOUT_L = 0x3C
ACCEL_YOUT_H = 0x3D
ACCEL_YOUT_L = 0x3E
ACCEL_ZOUT_H = 0x3F
ACCEL_ZOUT_L = 0x40

GYRO_CONFIG = 0x1B
GYRO_XOUT_H = 0x43
GYRO_XOUT_L = 0x44
GYRO_YOUT_H = 0x45
GYRO_YOUT_L = 0x46
GYRO_ZOUT_H = 0x47
GYRO_ZOUT_L = 0x48

TEMP_H = 0x41
TEMP_L = 0x42

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Temperature, Imu
import transforms3d


class Mpu6050Driver(Node):

    def __init__(self):
        super().__init__('mpu_6050_driver')

        self.declare_parameter('imu_frame', 'robocar')
        self.IMU_FRAME = self.get_parameter('imu_frame').get_parameter_value().string_value
        self.declare_parameter('bus', 0)
        self.bus = self.get_parameter('bus').get_parameter_value().integer_value
        self.declare_parameter('device_address', 0x68)
        self.ADDR = self.get_parameter('bus').get_parameter_value().integer_value

        self.bus.write_byte_data(self.ADDR, PWR_MGMT_1, 0)

        self.imu_pub = self.create_publisher(Imu, 'topic', 10)
        self.imu_timer = self.create_timer(0.02, self.imu_callback)
        self.temp_pub = self.create_publisher(Imu, 'topic', 10)
        self.temp_timer = self.create_timer(0.02, self.temp_callback)

    def read_word(self, adr):
        high = self.bus.read_byte_data(self.ADDR, adr)
        low = self.bus.read_byte_data(self.ADDR, adr+1)
        val = (high << 8) + low
        return val

    def read_word_i2c(self, adr):
        val = self.read_word(adr)
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val

    def imu_callback(self):
        imu_msg = Imu()
        imu_msg.header.frame_id = self.IMU_FRAME

        # Read the acceleration vals
        accel_x = self.read_word_i2c(ACCEL_XOUT_H) / 16384.0
        accel_y = self.read_word_i2c(ACCEL_YOUT_H) / 16384.0
        accel_z = self.read_word_i2c(ACCEL_ZOUT_H) / 16384.0
        
        # Calculate a quaternion representing the orientation
        accel = accel_x, accel_y, accel_z
        ref = np.array([0, 0, 1])
        acceln = accel / np.linalg.norm(accel)
        axis = np.cross(acceln, ref)
        angle = np.arccos(np.dot(acceln, ref))
        orientation = transforms3d.quaternions.axangle2quat(angle, axis)

        # Read the gyro vals
        gyro_x = self.read_word_i2c(GYRO_XOUT_H) / 131.0
        gyro_y = self.read_word_i2c(GYRO_YOUT_H) / 131.0
        gyro_z = self.read_word_i2c(GYRO_ZOUT_H) / 131.0
        
        # Load up the IMU message
        o = imu_msg.orientation
        o.x, o.y, o.z, o.w = orientation

        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z

        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z

        imu_msg.header.stamp = self.get_clock().now()

        self.imu_pub.publish(imu_msg)

    def temp_callback(self):
        temp_msg = Temperature()
        temp_msg.header.frame_id = self.IMU_FRAME
        temp_msg.temperature = self.read_word_i2c(TEMP_H)/340.0 + 36.53
        temp_msg.header.stamp = self.get_clock().now()
        self.temp_pub.publish(temp_msg)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = Mpu6050Driver()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()