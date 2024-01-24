import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import cv_bridge
from image_geometry.cameramodels import PinholeCameraModel
from sensor_msgs.msg import Image, CameraInfo
import pyrealsense2

class DetectObject(Node):
    def __init__(self):
        super().__init__("detect_object")
        # self.image_rect_raw = self.create_subscription(Image, "/camera/d435i/color/image_rect", self.image_rect_raw_callback, 10)
        self.image_raw = self.create_subscription(Image, "/camera/d435i/color/image_raw", self.image_raw_callback, 10)
        self.depth_image = None
        self.camera_info = self.create_subscription(CameraInfo, "/camera/d435i/color/camera_info", self.camera_callback, 10)
        self.intrinsics = None
        # published the masked depth image
        self.depth_publisher = self.create_publisher(
            Image, "/depth_mask", qos_profile=10
        )

        self.cv_bridge = cv_bridge.CvBridge()
        self.camera_model = PinholeCameraModel()
        # parameters for thresholding
        self.hue = [100, 114]
        self.saturation = [75, 255]
        self.value = [104, 255]

        self.window_detection_name = 'Object Detection'
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.cap = cv2.VideoCapture(0)
        self.cv_image = None

    
    def timer_callback(self):
        ret, frame = self.cap.read()     
        if ret == True and self.cv_image != None:
            self.depth_publisher.publish(self.cv_bridge.cv2_to_imgmsg(self.cv_image))
            cv2.namedWindow(self.window_detection_name, cv2.WINDOW_NORMAL)
            cv2.imshow(self.window_detection_name, self.cv_image)

    # def image_raw_callback(self, image):    
    #     """
    #     Callback function for depth image.

    #     Args:
    #         sensor_msgs.msg.Image: The depth image received from the camera.

    #     Returns:
    #         None

    #     """
    #     self.depth_image = self.cv_bridge.imgmsg_to_cv2(
    #         image, desired_encoding="passthrough"
    #     )

    def camera_callback(self, camera_info):
        self.intrinsics = pyrealsense2.intrinsics()
        self.intrinsics.width = camera_info.width
        self.intrinsics.height = camera_info.height
        self.intrinsics.ppx = camera_info.k[2]
        self.intrinsics.ppy = camera_info.k[5]
        self.intrinsics.fx = camera_info.k[0]
        self.intrinsics.fy = camera_info.k[4]
        self.intrinsics.model = pyrealsense2.distortion.none
        self.intrinsics.coeffs = [i for i in camera_info.d]

    def image_raw_callback(self, image):
        if self.depth_image is None or self.intrinsics is None:
            return
        # convert ros image msg to opencv mat
        self.cv_image = self.cv_bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")

        hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        thresh = cv2.inRange(hsv,(self.hue[0], self.saturation[0], self.value[0]), 
                             (self.hue[1], self.saturation[1], self.value[1]))
        
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours is not None:
            cx=0
            cy=0
            areas = []
            centroids = []
            locations = []
            if len(contours) > 0:
                for cnt in contours:
                    M = cv2.moments(cnt)
                    #print(M)
                    area = M['m00']
                    try:
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])     
                        centroid = (cx,cy)
                        centroids.append(centroid)
                        areas.append(area)                
                        rect = cv2.minAreaRect(cnt)
                        print(rect)
                        box = cv2.boxPoints(rect)
                        box = np.int0(box)
                        cv2.drawContours(self.cv_image,[box],0,(0,255,0),2) 
                    except:
                        pass  
                    
                if areas:
                    max = np.argmax(areas)
                    centroid_max = centroids[max]  
                    cv2.circle(self.cv_image, centroid_max, 5, [0,0,255], 5)    
                    # self.depth_publisher.publish(self.cv_bridge.cv2_to_imgmsg(self.cv_image))
                    # cv2.namedWindow(self.window_detection_name, cv2.WINDOW_NORMAL)
                    # cv2.imshow(self.window_detection_name, self.cv_image)

def main(args=None):
    rclpy.init(args=args)
    res = DetectObject()
    rclpy.spin(res)
    rclpy.shutdown()


if __name__ == "__main__":
    main()