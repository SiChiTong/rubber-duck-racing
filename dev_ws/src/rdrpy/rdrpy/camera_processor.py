import rclpy
from rclpy.node import Node
from .submodules.gstream import camStream
from .submodules.yutils import YUtils

import sensor_msgs.msg
from cv_bridge import CvBridge

class CameraProcessor(Node):

    def __init__(self):
        super().__init__('camera_processor')
        
        self.pub_img_unfliltered = self.create_publisher(sensor_msgs.msg.CompressedImage, 'unfiltered_feed', 10)   

        timer_period = 1 / 30
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.yutils = YUtils()
        self.get_logger().info("YUtils initialized")
        self.cap = camStream()
        self.get_logger().info("camera stream initialised")
        ret, self.frame = self.cap.read()
        self.cvb = CvBridge()
        self.get_logger().info("CvBridge initialised")

    def timer_callback(self):
        try:
            ret, self.frame = self.cap.read()
            if (ret):
                sign_detect_value = self.yutils.detect(self.frame)
                self.pub_img_unfliltered.publish(self.cvb.cv2_to_compressed_imgmsg(self.frame))
                self.get_logger().info("sign_detect_value: " + str(sign_detect_value))

        except Exception as e:
            self.get_logger().info(str(e))

def main(args=None):
    rclpy.init(args=args)

    camera_processor = CameraProcessor()

    rclpy.spin(camera_processor)

    rclpy.shutdown()

if __name__ == '__main__':
    main()