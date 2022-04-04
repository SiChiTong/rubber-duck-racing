from cv2 import line
import rclpy
from rclpy.node import Node
import cv2

import sensor_msgs.msg
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node

from robocar.msg import CarControl


class LineFollower(Node):

    def __init__(self):
        super().__init__('line_follower')
        self.cam_feed_sub = self.create_subscription(
            sensor_msgs.msg.CompressedImage,
            'webcam_feed',
            self.cam_feed_callback,
            10)
        self.cam_feed_sub  # prevent unused variable warning

        self.pub_car_control = self.create_publisher(CarControl, 'cmd_car_control', 10)

        self.cvb = CvBridge()

    def cam_feed_callback(self, msg):
        frame = self.cvb.compressed_imgmsg_to_cv2(msg)
        control_comand = CarControl()
        contours, hierarchy = cv2.findContours(frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        if (len(contours) > 0):
            cv2.drawContours(frame, contours, -1, 125, 2)

            lineContour = max(contours, key = cv2.contourArea)
            lineMoments = cv2.moments(lineContour)
            try:
                lineXPos = int(lineMoments['m10']/lineMoments['m00']) 
            except ZeroDivisionError:
                resized = cv2.resize(frame, (frame.shape[1] * 4, frame.shape[0] * 4), cv2.INTER_NEAREST)
                cv2.imshow("recieve", resized)
                control_comand.throttle = 0.0
                self.pub_car_control.publish(control_comand)
                return

            steeringFloat = float((lineXPos/frame.shape[1] - 1) * 1)

            control_comand.steering = steeringFloat
            control_comand.throttle = 0.2

            self.pub_car_control.publish(control_comand)
        else:
            control_comand.throttle = 0.0
            self.pub_car_control.publish(control_comand)
        resized = cv2.resize(frame, (frame.shape[1] * 4, frame.shape[0] * 4), cv2.INTER_NEAREST)
        cv2.imshow("recieve", resized)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    line_follower = LineFollower()

    rclpy.spin(line_follower)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    line_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()