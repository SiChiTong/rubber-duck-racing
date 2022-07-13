import math
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from .submodules.gstream import camStream
from .submodules.yutils import YUtils

import std_msgs.msg
import sensor_msgs.msg
from cv_bridge import CvBridge
from std_srvs.srv import Empty

class HSVCam(Node):
    def __init__(self):
        super().__init__('hsv_cam')
        
        self.calibrate_warp_srv = self.create_service(Empty, 'calibrate_warp', self.calibrate_warp_callback)
        self.refresh_params_srv = self.create_service(Empty, 'refresh_params', self.refresh_params_callback)

        self.pub_blue_img = self.create_publisher(sensor_msgs.msg.CompressedImage, 'blue_feed', 1)
        self.pub_yellow_img = self.create_publisher(sensor_msgs.msg.CompressedImage, 'yellow_feed', 1)
        self.pub_purple_img = self.create_publisher(sensor_msgs.msg.CompressedImage, 'purple_feed', 1)
        self.pub_red_img = self.create_publisher(sensor_msgs.msg.CompressedImage, 'red_feed', 1)
        self.pub_green_img = self.create_publisher(sensor_msgs.msg.CompressedImage, 'green_feed', 1)
        self.pub_sign_detection = self.create_publisher(std_msgs.msg.Int8, 'sign_detection', 10)
        self.sign_detect_value = 0
        self.yellow_hsv_vals = [0, 30, 30, 70, 255, 255]
        self.declare_parameter('yellow_hsv_vals', self.yellow_hsv_vals)
        self.blue_hsv_vals = [80, 60, 60, 150, 255, 255]
        self.declare_parameter('blue_hsv_vals', self.blue_hsv_vals)     
        self.purple_hsv_vals = [80, 60, 60, 150, 255, 255]
        self.declare_parameter('purple_hsv_vals', self.purple_hsv_vals)  
        self.red_hsv_vals = [0, 100, 20, 10, 255, 255]
        self.declare_parameter('red_hsv_vals', self.red_hsv_vals)  
        self.green_hsv_vals = [0, 100, 20, 10, 255, 255]
        self.declare_parameter('green_hsv_vals', self.green_hsv_vals) 

        self.declare_parameter('warp_calib_file', '/rubber-duck-racing/dev_ws/src/rdrpy/rdrpy/warp_calib.npz')
        self.declare_parameter('warp_calib_save', '/rubber-duck-racing/dev_ws/src/rdrpy/rdrpy/warp_calib')

        self.yellow_hsv_vals = self.get_parameter('yellow_hsv_vals').get_parameter_value().integer_array_value
        self.blue_hsv_vals = self.get_parameter('blue_hsv_vals').get_parameter_value().integer_array_value
        self.purple_hsv_vals = self.get_parameter('purple_hsv_vals').get_parameter_value().integer_array_value
        self.red_hsv_vals = self.get_parameter('red_hsv_vals').get_parameter_value().integer_array_value
        self.green_hsv_vals = self.get_parameter('green_hsv_vals').get_parameter_value().integer_array_value

        try:
            calib_file_path =  self.get_parameter('warp_calib_file').get_parameter_value().string_value
            data = np.load(calib_file_path)
            self.homography = data['homography']
            self.bwidth = int(data['width'])
            self.bheight = int(data['height'])
        except Exception as e:
            self.get_logger().info("failed to read warp calibration file, no warp will be applied")

        timer_period = 1 / 60
        self.timer = self.create_timer(timer_period, self.timer_callback)
        yolo_period = 1 / 3
        self.yolo_timer = self.create_timer(yolo_period, self.yolo_timer_callback)
        self.yutils = YUtils()
        self.get_logger().info("YUtils initialized")
        self.cap = camStream()
        self.get_logger().info("camera stream initialised")
        self.ret, self.frame = self.cap.read()
        self.cvb = CvBridge()
        self.get_logger().info("CvBridge initialised")     

    @staticmethod
    def generateFlatCorners():
        cornersFlat = np.zeros((40, 1, 2))

        for x in range (8):
            for y in range(5):
                i = y + x * 5
                cornersFlat[i][0][0] = x * 3
                cornersFlat[i][0][1] = y * 3
        return cornersFlat
    
    @staticmethod
    def getPointRotation(pt1, pt2):
        slope = (pt1[1] - pt2[1]) / (pt1[0] - pt2[0])
        return math.degrees(math.atan(slope))

    def calibrateWarp(self):
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        while (1):
            ret, frame = self.cap.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            retCorners, cornersReal = cv2.findChessboardCorners(gray, (5, 8))
            cornersFlat = self.generateFlatCorners()

            if(retCorners):
                corners2 = cv2.cornerSubPix(gray,cornersReal, (11,11), (-1,-1), criteria)
                H, _ = cv2.findHomography(corners2, cornersFlat)

                corners = np.array([
                    [0, 0],
                    [0, frame.shape[0] - 1],
                    [frame.shape[1] - 1, frame.shape[0] -1],
                    [frame.shape[1] - 1, 0]
                ])

                cornersFinal = cv2.perspectiveTransform(np.float32([corners]), H)[0]

                bx, by, bwidth, bheight = cv2.boundingRect(cornersFinal)

                angle = self.getPointRotation(cornersFinal[1], cornersFinal[2])
                print(angle)
                rotationMtx = cv2.getRotationMatrix2D((bwidth/2, bheight/2), angle, 1)

                cornersFlat = cv2.transform(cornersFlat, rotationMtx)
                H, _ = cv2.findHomography(corners2, cornersFlat)

                corners = np.array([
                    [0, 0],
                    [0, frame.shape[0] - 1],
                    [frame.shape[1] - 1, frame.shape[0] -1],
                    [frame.shape[1] - 1, 0]
                ])

                cornersFinal = cv2.perspectiveTransform(np.float32([corners]), H)[0]

                bx, by, bwidth, bheight = cv2.boundingRect(cornersFinal)

                th = np.array([
                    [ 1, 0, -bx ],
                    [ 0, 1, -by ],
                    [ 0, 0,   1 ]
                ])

                pth = th.dot(H)
                
                calib_file_path = self.get_parameter('warp_calib_save').get_parameter_value().string_value
                np.savez(calib_file_path, homography=pth, width=bwidth, height=bheight)

                return pth, bwidth, bheight

    def hsv_line_detect(self, image):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        blue_mask = cv2.inRange(hsv_image, 
            (
                self.blue_hsv_vals[0],
                self.blue_hsv_vals[1],
                self.blue_hsv_vals[2]
            ),
            (
                self.blue_hsv_vals[3],
                self.blue_hsv_vals[4],
                self.blue_hsv_vals[5]
                
            ))
        yellow_mask = cv2.inRange(hsv_image, 
            (
                self.yellow_hsv_vals[0],
                self.yellow_hsv_vals[1],
                self.yellow_hsv_vals[2]
            ),
            (
                self.yellow_hsv_vals[3],
                self.yellow_hsv_vals[4],
                self.yellow_hsv_vals[5]
                
            ))
        
        return blue_mask, yellow_mask

    def hsv_aux_detect(self, image):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        purple_mask = cv2.inRange(hsv_image, 
            (
                self.purple_hsv_vals[0],
                self.purple_hsv_vals[1],
                self.purple_hsv_vals[2]
            ),
            (
                self.purple_hsv_vals[3],
                self.purple_hsv_vals[4],
                self.purple_hsv_vals[5]
                
            ))
        red_mask = cv2.inRange(hsv_image, 
            (
                self.red_hsv_vals[0],
                self.red_hsv_vals[1],
                self.red_hsv_vals[2]
            ),
            (
                self.red_hsv_vals[3],
                self.red_hsv_vals[4],
                self.red_hsv_vals[5]
                
            ))
        green_mask = cv2.inRange(hsv_image, 
            (
                self.green_hsv_vals[0],
                self.green_hsv_vals[1],
                self.green_hsv_vals[2]
            ),
            (
                self.green_hsv_vals[3],
                self.green_hsv_vals[4],
                self.green_hsv_vals[5]
                
            ))
        return purple_mask, red_mask, green_mask

    def timer_callback(self):
        try:
            self.ret, self.frame = self.cap.read()
            image = self.frame

            if (self.ret):
                if (hasattr(self, 'homography')):
                    image = cv2.warpPerspective(image, self.homography, (self.bwidth, self.bheight), cv2.INTER_NEAREST)

                blue_mask, yellow_mask = self.hsv_line_detect(image)
                self.get_logger.info("Publishing frame")
                self.pub_blue_img.publish(self.cvb.cv2_to_compressed_imgmsg(blue_mask))
                self.pub_yellow_img.publish(self.cvb.cv2_to_compressed_imgmsg(yellow_mask))
                

        except Exception as e:
            self.get_logger().error(str(e))

    def yolo_timer_callback(self):
        if(self.ret):
            msg = std_msgs.msg.Int8()
            self.sign_detect_value = self.yutils.detect(self.frame, False, 0.7)
            msg.data = self.sign_detect_value
            # purple_mask, red_mask, green_mask = self.hsv_aux_detect(self.frame)
            self.get_logger().info("Sign detect value: " + str(self.sign_detect_value))
            self.pub_sign_detection.publish(msg)
            # self.pub_purple_img.publish(self.cvb.cv2_to_compressed_imgmsg(purple_mask))
            # self.pub_red_img.publish(self.cvb.cv2_to_compressed_imgmsg(red_mask))
            # self.pub_green_img.publish(self.cvb.cv2_to_compressed_imgmsg(green_mask))

    def calibrate_warp_callback(self, request, response):
        self.get_logger().info('Request to calibrate recieved')
        try:
            self.homography, self.bwidth, self.bheight = self.calibrateWarp()
        except Exception as e:
            self.get_logger().info('Calibration Failed')
            self.get_logger().info(str(e))
        else:
            self.get_logger().info('Calibration Succeeded')
            self.get_logger().info("Frame size: " + str(self.bwidth) + "x" + str(self.bheight) + "y")
        return response

    def refresh_params_callback(self, request, response):
        self.get_logger().info('Refreshing the parameters')
        self.yellow_hsv_vals = self.get_parameter('yellow_hsv_vals').get_parameter_value().integer_array_value
        self.blue_hsv_vals = self.get_parameter('blue_hsv_vals').get_parameter_value().integer_array_value
        return response

def main(args=None):
    rclpy.init(args=args)

    hsv_cam = HSVCam()

    rclpy.spin(hsv_cam)

    rclpy.shutdown()

if __name__ == '__main__':
    main()