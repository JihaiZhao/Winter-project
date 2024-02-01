import rclpy
from rclpy.node import Node
import tf2_geometry_msgs
from tf2_ros import Buffer, TransformListener
from moveit_wrapper.moveitapi import MoveItApi
from moveit_wrapper.grasp_planner import GraspPlan, GraspPlanner
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped, Transform, Vector3
from std_msgs.msg import Header
from rclpy.callback_groups import ReentrantCallbackGroup
from franka_msgs.action import Grasp
from rclpy.time import Time
from franka_msgs.msg import GraspEpsilon
from rclpy.callback_groups import ReentrantCallbackGroup
from botrista_interfaces.action import EmptyAction, GraspProcess, PourAction
from rclpy.action import ActionServer, ActionClient
from botrista_interfaces.srv import DelayTime
import numpy as np

class Pick_Place(Node):
    def __init__(self):
        super().__init__("pick_place")

        self.kettle_actual_place = TransformStamped()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.moveit_api = MoveItApi(
            self, "panda_link0", "panda_hand_tcp", "panda_manipulator", "/franka/joint_states")
        self.grasp_planner = GraspPlanner(
            self.moveit_api, "panda_gripper/grasp")
        self.declare_parameter(
            "x",
            -0.5,
        )
        self.declare_parameter(
            "y",
            -0.5,
        )
        self.x_position = self.get_parameter("x").get_parameter_value().double_value
        self.y_position = self.get_parameter("y").get_parameter_value().double_value

        self.point = Point(x=0.5, y=-0.5, z=0.10)
        self.q = Quaternion(x=0.999636, y=0.00625, z=-0.0262182, w=-0.0128503)

        self.timer = self.create_timer(0.1, self.timer_callback) 

    def timer_callback(self):
        try:
            tf = self.buffer.lookup_transform("panda_link0", "object", Time())
        except Exception as e:
            self.get_logger().error("No transform found")
            return
        
        self.moveit_api.plan(point=tf.transform.translation, orientation=tf.transform.rotation, execute=True)

def main(args=None):
    rclpy.init(args=args)
    res = Pick_Place()
    rclpy.spin(res)
    rclpy.shutdown()

if __name__ == "__main__":
    main()