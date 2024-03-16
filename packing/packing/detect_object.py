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
from scipy.ndimage import median_filter
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.transform_listener import TransformListener, Buffer
from std_msgs.msg import Header
from rclpy.time import Time
from std_msgs.msg import Empty

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

        # publish the size of the object
        self.size_publish = self.create_publisher(
            Point, "/dimension", qos_profile=10
        )

        self.cv_bridge = cv_bridge.CvBridge()
        self.camera_model = PinholeCameraModel()
        # parameters for thresholding
        self.hue = [90, 114]
        self.saturation = [75, 255]
        self.value = [104, 255]

        # tf2 broadcasters and listeners
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, qos=10)
        self.tf_broadcaster = TransformBroadcaster(self, qos=10)
        self.transform_broadcaster = TransformBroadcaster(self, qos=10)     
        
        self.object_publisher = self.create_publisher(
            Empty, "program_start", qos_profile=10
        )

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

        # thresh = cv2.inRange(
        #     hsv,
        #     (self.hue[0], self.saturation[0], self.value[0]),
        #     (self.hue[1], self.saturation[1], self.value[1]),
        # )

        mask1 = cv2.inRange(hsv, (0, 70, 50), (10, 255, 255))
        mask2 = cv2.inRange(hsv, (160, 70, 50), (180, 255, 255))

        thresh = mask1 | mask2

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
                    topmost = tuple(object_contour[object_contour[:, :, 1].argmin()][0])
                    bottommost = tuple(object_contour[object_contour[:, :, 1].argmax()][0])
                    leftmost = tuple(object_contour[object_contour[:, :, 0].argmin()][0])
                    rightmost = tuple(object_contour[object_contour[:, :, 0].argmax()][0])
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    centroid = (cx, cy)
                    rect = cv2.minAreaRect(object_contour)
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    # self.get_logger().info(box.__str__())
                    contour_img = cv2.drawContours(self.cv_image, [box], -1, (0, 255, 0), 2)
                    contour_img = cv2.circle(contour_img, centroid, 2, [255, 0, 0], 2)
                    contour_img = cv2.circle(contour_img, topmost, 2, [255, 0, 0], 2)
                    contour_img = cv2.circle(contour_img, bottommost, 2, [255, 0, 0], 2)
                    contour_img = cv2.circle(contour_img, leftmost, 2, [255, 0, 0], 2)
                    contour_img = cv2.circle(contour_img, rightmost, 2, [255, 0, 0], 2)

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

                    # self.get_logger().info(contour_img.__str__())
                    self.depth_publisher.publish(self.cv_bridge.cv2_to_imgmsg(contour_img))
                    
                    if not self.depth_image.any():
                        return None

                    # use all non-zero values as the depth, zero values are invalid
                    depth_mask_filtered = self.depth_image[self.depth_image != 0]

                    depth_mask_filtered = median_filter(depth_mask_filtered, 1)

                    depth = ss.tmean(depth_mask_filtered)

                    # get the 3d point of the center and 4 conner
                    point = pyrealsense2.rs2_deproject_pixel_to_point(
                        self.intrinsics, [cx, cy], depth
                    )
                    point = np.array(point)
                    point /= 1000.0  # mm to m

                    point_right = pyrealsense2.rs2_deproject_pixel_to_point(
                        self.intrinsics, rightmost, depth
                    )
                    point_right = np.array(point_right)
                    point_right /= 10.0  # mm to cm

                    point_left = pyrealsense2.rs2_deproject_pixel_to_point(
                        self.intrinsics, leftmost, depth
                    )
                    point_left = np.array(point_left)
                    point_left /= 10.0  # mm to cm

                    point_top = pyrealsense2.rs2_deproject_pixel_to_point(
                        self.intrinsics, topmost, depth
                    )
                    point_top = np.array(point_top)
                    point_top /= 10.0  # mm to cm

                    point_bottom = pyrealsense2.rs2_deproject_pixel_to_point(
                        self.intrinsics, bottommost, depth
                    )
                    point_bottom = np.array(point_bottom)
                    point_bottom /= 10.0  # mm to cm

                    # find the length and width
                    length = np.sqrt((point_top[0]-point_left[0])**2 + (point_top[1]-point_left[1])**2)
                    width = np.sqrt((point_left[0]-point_bottom[0])**2 + (point_left[1]-point_bottom[1])**2)

                    angle = np.arctan2(point_top[1] - point_left[1], point_top[0] - point_left[0])
                    if width - length > 0.5:
                        temp = length
                        length = width
                        width = temp
                        angle = -angle

                    # self.get_logger().info(angle.__str__())
                    
                    # self.get_logger().info("point_top")
                    # self.get_logger().info(point_top.__str__())
                    # self.get_logger().info(point_top[0].__str__())
                    # self.get_logger().info(point_top[1].__str__())
                    # self.get_logger().info(point_left[0].__str__())
                    # self.get_logger().info(point_left[1].__str__())

                    msg = Point()
                    msg.x = (length)
                    msg.y = (width)
                    self.size_publish.publish(msg)
                    # self.get_logger().info("width")
                    # self.get_logger().info(msg.y.__str__())
                    # self.get_logger().info("length")
                    # self.get_logger().info(msg.x.__str__())

                    # find average depth of the five points
                    # self.avg_z = (point_top[2]+point_left[2]+point_bottom[2]+point_right[2]+point[2])/5
                    # self.get_logger().info("avg")
                    # self.get_logger().info(self.avg_z.__str__())
                    # self.get_logger().info("depth")
                    # self.test = point[2]-0.05
                    # self.get_logger().info(self.test.__str__())

                    pose = Pose(
                        position=Point(x=point[0]-0.005, y=point[1], z=point[2]-0.029),
                        # orientation=euler_to_quaternion(-np.pi/2, 0, 0),
                        orientation=euler_to_quaternion(angle, 0, 0),
                    )
                    # self.get_logger().info("center")
                    # self.get_logger().info(point.__str__())
                    
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
                        self.object_publisher.publish(Empty())
                    except Exception as e:
                        self.get_logger().warn(f"Exception: {e}")
                        pass                        

                except:
                    pass
  
def euler_to_quaternion(yaw, pitch, roll):
    """
    Convert Euler angles to a quaternion.

    Args:
        yaw (float): Yaw angle in radians.
        pitch (float): Pitch angle in radians.
        roll (float): Roll angle in radians.

    Returns:
        Quaternion: A quaternion representing the rotation.

    """
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(
        roll / 2
    ) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(
        roll / 2
    ) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(
        roll / 2
    ) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(
        roll / 2
    ) * np.sin(pitch / 2) * np.sin(yaw / 2)

    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main(args=None):
    rclpy.init(args=args)
    res = DetectObject()
    rclpy.spin(res)
    rclpy.shutdown()