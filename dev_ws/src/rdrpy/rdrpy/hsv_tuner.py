import rclpy
from rclpy.node import Node
import cv2

import sensor_msgs.msg
from cv_bridge import CvBridge

cvb = CvBridge()

class HSVTuner(Node):
    @staticmethod
    def nothing(x):
        pass

    def __init__(self):
        super().__init__('hsv_tuner')
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
            sensor_msgs.msg.CompressedImage,
            'unfiltered_feed',
            self.listener_callback_unfiltered,
            10)
        cv2.namedWindow('recieve_unfiltered')
        cv2.createTrackbar('Hmin', 'recieve_unfiltered', 0, 255, self.nothing)
        cv2.createTrackbar('Hmax', 'recieve_unfiltered', 0, 255, self.nothing)
        cv2.createTrackbar('Smin', 'recieve_unfiltered', 0, 255, self.nothing)
        cv2.createTrackbar('Smax', 'recieve_unfiltered', 0, 255, self.nothing)
        cv2.createTrackbar('Vmin', 'recieve_unfiltered', 0, 255, self.nothing)
        cv2.createTrackbar('Vmax', 'recieve_unfiltered', 0, 255, self.nothing)

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
        Hmin = cv2.getTrackbarPos('Hmin', 'recieve_unfiltered')
        Hmax = cv2.getTrackbarPos('Hmax', 'recieve_unfiltered')
        Smin = cv2.getTrackbarPos('Smin', 'recieve_unfiltered')
        Smax = cv2.getTrackbarPos('Smax', 'recieve_unfiltered')
        Vmin = cv2.getTrackbarPos('Vmin', 'recieve_unfiltered')
        Vmax = cv2.getTrackbarPos('Vmax', 'recieve_unfiltered')

        frame = cvb.compressed_imgmsg_to_cv2(msg)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (Hmin, Smin, Vmin), (Hmax, Smax, Vmax))

        cv2.imshow("recieve_unfiltered", frame)
        cv2.imshow("mask", mask)
        cv2.waitKey(1)
        #self.get_logger().info("recieved frame")

def main(args=None):
    rclpy.init(args=args)

    camera_viewer = HSVTuner()

    rclpy.spin(camera_viewer)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_viewer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()