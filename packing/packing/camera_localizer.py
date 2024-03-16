""" 
A node that handles camera localization and tag filtering.

Publishes Transforms:
    + d435i_link (geometry_msgs/TransformStamped): The tf from the d435i camera to the robot.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from tf2_ros import (
    TransformListener,
    Buffer,
    TransformBroadcaster,
    StaticTransformBroadcaster,
)
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion


class CameraLocalizer(Node):
    """A node that handles camera localization and tag filtering."""

    def __init__(self):
        super().__init__("camera_localizer")

        # create transform listener and buffer
        self.buffer = Buffer()
        self.transform_listener = TransformListener(self.buffer, self)
        self.transform_broadcaster = TransformBroadcaster(self)
        self.static_transform_broadcaster = StaticTransformBroadcaster(self)

        # timer to publish transforms
        self.timer = self.create_timer(0.1, self.timer_callback)

    async def timer_callback(self):
        """Publish the transform for the d435i"""
        # publish d435i transform to the franka hand
        self.transform_broadcaster.sendTransform(
            TransformStamped(
                header=Header(
                    stamp=self.get_clock().now().to_msg(), frame_id="panda_hand"
                ),
                child_frame_id="d435i_link",
                transform=Transform(
                    translation=Vector3(x=0.05, y=0.0, z=0.065),
                    rotation=Quaternion(
                        x=0.706825, y=-0.0005629, z=0.707388, w=0.0005633
                    ),
                ),
            ),
        )

        self.static_transform_broadcaster.sendTransform(
            TransformStamped(
                header=Header(
                    stamp=self.get_clock().now().to_msg(), frame_id="panda_link0"
                ),
                child_frame_id="box",
                transform=Transform(
                    translation=Vector3(x=0.40, y=0.0, z=0.0),
                    rotation=Quaternion(
                        # x=0.0, y=0.0, z=0.9999997, w=0.0007963
                        # x = 1.000, y = -0.001, z = -0.006, w = -0.005
                        x = 1.000, y = 0.0, z = 0.0, w = 0.0
                        # x=0.706825, y=-0.0005629, z=0.707388, w=0.0005633
                    ),
                ),
            )
        )

def main(args=None):
    rclpy.init(args=args)
    camera_localizer = CameraLocalizer()
    rclpy.spin(camera_localizer)
    rclpy.shutdown()