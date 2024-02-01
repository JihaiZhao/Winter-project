import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import cv_bridge
from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion,
    Vector3,
    TransformStamped,
    Transform,
)
import tf2_geometry_msgs
from image_geometry.cameramodels import PinholeCameraModel
from sensor_msgs.msg import Image, CameraInfo
import pyrealsense2
import scipy.stats as ss
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.transform_listener import TransformListener, Buffer
from std_msgs.msg import Header
from rclpy.time import Time

class DetectObject(Node):
    def __init__(self):
        super().__init__("detect_object")
        self.image_rect_raw = self.create_subscription(
            Image,
            "/camera/d435i/color/image_raw",
            self.image_rect_raw_callback,
            qos_profile=10,
        )
        self.image_raw = self.create_subscription(
            Image,
            "/camera/d435i/aligned_depth_to_color/image_raw",
            self.image_raw_callback,
            qos_profile=10,
        )
        self.depth_image = None
        self.camera_info = self.create_subscription(
            CameraInfo,
            "/camera/d435i/aligned_depth_to_color/camera_info",
            self.camera_callback,
            qos_profile=10,
        )
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

        # tf2 broadcasters and listeners
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, qos=10)
        self.tf_broadcaster = TransformBroadcaster(self, qos=10)
        self.transform_broadcaster = TransformBroadcaster(self, qos=10)   

        # timer to publish transforms
        self.timer = self.create_timer(0.1, self.timer_callback)     

    def image_raw_callback(self, image):
        """
        Callback function for depth image.

        Args:
            sensor_msgs.msg.Image: The depth image received from the camera.

        Returns:
            None

        """
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(
            image, desired_encoding="passthrough"
        )
        self.get_logger().debug(self.depth_image.__str__())

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

        self.get_logger().debug(self.intrinsics.__str__())

    def image_rect_raw_callback(self, image):
        if self.depth_image is None or self.intrinsics is None:
            return
        # convert ros image msg to opencv mat
        self.cv_image = self.cv_bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        self.get_logger().debug(self.cv_image.__str__())

        hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        thresh = cv2.inRange(
            hsv,
            (self.hue[0], self.saturation[0], self.value[0]),
            (self.hue[1], self.saturation[1], self.value[1]),
        )

        contours, hierarchy = cv2.findContours(
            thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )
        contours = list(contours)
        # self.get_logger().info(len(contours).__str__())
        if len(contours) != 0:
            object_contour = max(contours, key=lambda cnt: cv2.contourArea(cnt))
            if object_contour is not None:
                M = cv2.moments(object_contour)
                try:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    centroid = (cx, cy)
                    rect = cv2.minAreaRect(object_contour)
                    # self.get_logger().info(rect[1].__str__())
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    # self.get_logger().info(box.__str__())
                    contour_img = cv2.drawContours(self.cv_image, [box], 0, (0, 255, 0), 3)
                    # perimeter = cv2.arcLength(object_contour,True)/10
                    # self.get_logger().info(perimeter.__str__())
                    contour_img = cv2.circle(contour_img, centroid, 5, [0, 0, 255], 1)
                    length = rect[1][0]
                    width = rect[1][1]
                    # cv2.putText(
                    #     contour_img,
                    #     f"{int(2*cx)}",
                    #     (2*cx, cy),
                    #     cv2.FONT_HERSHEY_SIMPLEX,
                    #     1,
                    #     (0, 0, 255),
                    #     2,
                    # )

                    # cv2.putText(
                    #     contour_img,
                    #     f"{int(width)}",
                    #     (cx, cy+int(width)/2),
                    #     cv2.FONT_HERSHEY_SIMPLEX,
                    #     1,
                    #     (0, 0, 255),
                    #     2,
                    # )

                    self.depth_publisher.publish(self.cv_bridge.cv2_to_imgmsg(contour_img))

                    if not self.depth_image.any():
                        return None

                    # use all non-zero values as the depth, zero values are invalid
                    depth_mask_filtered = self.depth_image[self.depth_image != 0]

                    depth = ss.tmean(depth_mask_filtered)

                    # get the 3d point
                    point = pyrealsense2.rs2_deproject_pixel_to_point(
                        self.intrinsics, [cx, cx], depth
                    )
                    point = np.array(point)
                    point /= 1000.0  # mm to m
                    pose = Pose(
                        position=Point(x=point[0], y=point[1], z=point[2]),
                        orientation=Quaternion(x=1, y=0, z=0, w=0),
                    )
                    if pose is None:
                        return
                    
                    try:
                        tf = self.tf_buffer.lookup_transform(
                            "panda_link0", "d435i_color_optical_frame", time=Time(seconds=0.0)
                        )
                        pose_panda0 = tf2_geometry_msgs.do_transform_pose(pose, tf)
                        self.transform_broadcaster.sendTransform(
                            TransformStamped(
                                header=Header(
                                    stamp=self.get_clock().now().to_msg(),
                                    frame_id="panda_link0",
                                ),
                                child_frame_id="object",
                                transform=Transform(
                                    translation=Vector3(
                                        x=pose_panda0.position.x,
                                        y=pose_panda0.position.y,
                                        z=pose_panda0.position.z,
                                    ),
                                    rotation=pose_panda0.orientation,
                                ),
                            )
                        )
                    except Exception as e:
                        self.get_logger().warn(f"Exception: {e}")
                        pass                        

                except:
                    pass

    def timer_callback(self):
        """Publish the transform for the d435i"""
        # publish d435i transform to the franka hand
        self.transform_broadcaster.sendTransform(
            TransformStamped(
                header=Header(
                    stamp=self.get_clock().now().to_msg(), frame_id="panda_hand"
                ),
                child_frame_id="d435i_link",
                transform=Transform(
                    translation=Vector3(x=0.04, y=0.0, z=0.05),
                    rotation=Quaternion(
                        x=0.706825, y=-0.0005629, z=0.707388, w=0.0005633
                    ),
                ),
            )
        )

def main(args=None):
    rclpy.init(args=args)
    res = DetectObject()
    rclpy.spin(res)
    rclpy.shutdown()


if __name__ == "__main__":
    main()