import math
import rclpy
from rclpy.node import Node
import cv2
import numpy as np

import sensor_msgs.msg
from cv_bridge import CvBridge
from std_srvs.srv import Empty

imsizeX = 128
imsizeY = 96
predictionHeight = imsizeY//8

class CameraProcessor(Node):

    def __init__(self):
        super().__init__('camera_processor')
        
        self.calibrate_warp_srv = self.create_service(Empty, 'calibrate_warp', self.calibrate_warp_callback)
        
        self.topic = topic = "webcam_feed"
        
        self.pub_img = self.create_publisher(sensor_msgs.msg.CompressedImage, topic, 10)
        self.pub_img_unfliltered = self.create_publisher(sensor_msgs.msg.Image, topic + '/unfiltered', 10)

        self.declare_parameter('transmit_unfiltered', False)
        self.transmit_unfiltered = self.get_parameter('transmit_unfiltered').get_parameter_value().bool_value

        self.declare_parameter('calibration_file', 'calibration.npz')
        self.declare_parameter('calibration_save', 'calibration')

        try:
            calib_file_path =  self.get_parameter('calibration_file').get_parameter_value().string_value
            data = np.load(calib_file_path)
            self.homography = data['homography']
            self.bwidth = int(data['width'])
            self.bheight = int(data['height'])
        except Exception as e:
            self.get_logger().info("failed to read calibration file: " + str(e))

        timer_period = 0.001
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.cap = cv2.VideoCapture(0)
        ret, self.frame = self.cap.read()
        self.cvb = CvBridge()

    @staticmethod
    def generateFlatCorners():
        cornersFlat = np.zeros((70, 1, 2))
        offsetX = 320 - 100
        offsetY = 240 - 70

        for x in range (10):
            for y in range(7):
                i = y + x * 7
                cornersFlat[i][0][0] = x * 5 + offsetX
                cornersFlat[i][0][1] = y * 5 + offsetY

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
            retCorners, cornersReal = cv2.findChessboardCorners(gray, (7, 10))
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
                
                calib_file_path = self.get_parameter('calibration_save').get_parameter_value().string_value
                np.savez(calib_file_path, homography=pth, width=bwidth, height=bheight)

                return pth, bwidth, bheight

    def timer_callback(self):
        try:
            ret, self.frame = self.cap.read()
            image = self.frame
            if (ret):
                #image = cv2.resize(self.frame, (imsizeX, imsizeY))

                if(hasattr(self, 'homography')):
                    image = cv2.warpPerspective(image, self.homography, (self.bwidth, self.bheight))
                                
                self.pub_img.publish(self.cvb.cv2_to_compressed_imgmsg(image))

                self.transmit_unfiltered = self.transmit_unfiltered = self.get_parameter('transmit_unfiltered').get_parameter_value().bool_value
                if (self.transmit_unfiltered):
                    self.pub_img_unfliltered.publish(self.cvb.cv2_to_imgmsg(self.frame))

                cv2.waitKey(1)
        except Exception as e:
            self.get_logger().info(str(e))

    def calibrate_warp_callback(self, request, response):
        self.get_logger().info('Request to calibrate recieved')
        try:
            self.homography, self.bwidth, self.bheight = self.calibrateWarp()
        except Exception as e:
            self.get_logger().info('Calibration Failed')
            self.get_logger().info(str(e))
        else:
            self.get_logger().info('Calibration Succeeded')
        return response

def main(args=None):
    rclpy.init(args=args)

    camera_processor = CameraProcessor()

    rclpy.spin(camera_processor)

    rclpy.shutdown()

if __name__ == '__main__':
    main()