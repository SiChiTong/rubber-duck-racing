import cv2
import sys
import numpy as np
import rclpy
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
from cv_bridge import CvBridge
import std_msgs.msg as std_msgs



waitTime = 33



class PointCloud(Node):
    def __init__(self):
        super().__init__('point_cloud')
        
        self.pub_blue_img = self.create_publisher(sensor_msgs.PointCloud2, 'pcd', 10)


        timer_period = 1 / 60
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.cap = cv2.VideoCapture(0)
        
        self.get_logger().info("camera stream initialised")
        ret, self.frame = self.cap.read()
        self.cvb = CvBridge()
        self.get_logger().info("CvBridge initialised")     

    def hsv_line_detect(self, image):
        hMin = 25
        sMin = 54
        vMin = 60

        hMax = 32
        sMax = 255
        vMax = 255
        lower = np.array([hMin, sMin, vMin])
        upper = np.array([hMax, sMax, vMax])
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        resized_image = cv2.resize(hsv_image, (320, 240), cv2.INTER_NEAREST)
        mask = cv2.inRange(resized_image, lower, upper)

        
        return self.mask_to_point_cloud(mask)
         

        
    def timer_callback(self):
        try:
            ret, self.frame = self.cap.read()
            image = self.frame
            
            if (ret):
                pcd = self.hsv_line_detect(image)

                self.pub_blue_img.publish(pcd)
                

        except Exception as e:
            self.get_logger().info(str(e)) 

    @staticmethod
    def mask_to_point_cloud(mask):
        points = np.empty((2,3))
        for i in range(240): 
           for j in range(320):
               if(mask[i][j] != 0):
                    points = np.append(points, [[i/10,j/10,0]], axis=0)
        ros_dtype = sensor_msgs.PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize
        data = points.astype(dtype).tobytes() 
        fields = [sensor_msgs.PointField(
        name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate('xyz')]
        
        header = std_msgs.Header(frame_id="map")
        

        return sensor_msgs.PointCloud2(
            header=header,
            height=1, 
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=(itemsize * 3), # Every point consists of 3 float32s.
            row_step=(itemsize * 3 * points.shape[0]),
            data=data
        )

def main(args=None):
    rclpy.init(args=args)

    point_cloud = PointCloud()

    rclpy.spin(point_cloud)

    rclpy.shutdown()

if __name__ == '__main__':
    main()