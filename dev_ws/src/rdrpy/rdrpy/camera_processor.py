import rclpy
from rclpy.node import Node
from .submodules.gstream import camStream

import sensor_msgs.msg
from cv_bridge import CvBridge

class CameraProcessor(Node):

    def __init__(self):
        super().__init__('camera_processor')
        
        self.pub_img_unfliltered = self.create_publisher(sensor_msgs.msg.Image, 'unfiltered_feed', 10)   

        timer_period = 1 / 60
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.cap = camStream()
        self.get_logger().info("camera stream initialised")
        ret, self.frame = self.cap.read()
        self.cvb = CvBridge()
        self.get_logger().info("CvBridge initialised")

    def timer_callback(self):
        try:
            ret, self.frame = self.cap.read()
            
            if (ret):
                self.pub_img_unfliltered.publish(self.cvb.cv2_to_imgmsg(self.frame))

        except Exception as e:
            self.get_logger().info(str(e))

def main(args=None):
    rclpy.init(args=args)

    camera_processor = CameraProcessor()

    rclpy.spin(camera_processor)

    rclpy.shutdown()

if __name__ == '__main__':
    main()