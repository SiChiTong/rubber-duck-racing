import rclpy
from rclpy.node import Node
import cv2

import sensor_msgs.msg
from cv_bridge import CvBridge

cvb = CvBridge()

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        self.subscription_blue = self.create_subscription(
            sensor_msgs.msg.CompressedImage,
            'blue_feed',
            self.listener_callback_blue,
            10)
        self.subscription_yellow = self.create_subscription(
            sensor_msgs.msg.CompressedImage,
            'yellow_feed',
            self.listener_callback_yellow,
            10)
        self.subscription_unfiltered = self.create_subscription(
            sensor_msgs.msg.Image,
            'unfiltered_feed',
            self.listener_callback_unfiltered,
            10)

    def listener_callback_blue(self, msg):
        frame = cvb.compressed_imgmsg_to_cv2(msg)
        #frame = cv2.resize(frame, (frame.shape[1] * 4, frame.shape[0] * 4), cv2.INTER_NEAREST)
        cv2.imshow("recieve_blue", frame)
        cv2.waitKey(1)
        #self.get_logger().info("recieved frame")

    def listener_callback_yellow(self, msg):
        frame = cvb.compressed_imgmsg_to_cv2(msg)
        #frame = cv2.resize(frame, (frame.shape[1] * 4, frame.shape[0] * 4), cv2.INTER_NEAREST)
        cv2.imshow("recieve_yellow", frame)
        cv2.waitKey(1)
        #self.get_logger().info("recieved frame")

    def listener_callback_unfiltered(self, msg):
        frame = cvb.imgmsg_to_cv2(msg)
        cv2.imshow("recieve_unfiltered", frame)
        cv2.waitKey(1)
        #self.get_logger().info("recieved frame")

def main(args=None):
    rclpy.init(args=args)

    camera_viewer = CameraViewer()

    rclpy.spin(camera_viewer)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_viewer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()