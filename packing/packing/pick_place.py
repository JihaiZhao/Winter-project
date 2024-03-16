import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import (
    Point,
    Quaternion,
    Pose,
)
import tf2_geometry_msgs
from rclpy.time import Time
import numpy as np
from moveit_wrapper.moveitapi import MoveItApi
from moveit_wrapper.grasp_planner import GraspPlan, GraspPlanner
from franka_msgs.action import Grasp
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from packing_interface.action import EmptyAction, GraspProcess
from tf2_ros.transform_listener import TransformListener
from franka_msgs.action import (
    Grasp,
)
from packing.pack import Packer, SimplePacker, Rect

class pick_place(Node):
    def __init__(self):
        super().__init__("pick_place")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.container_size = (0.1, 0.1, 0.3)
        # self.container_size = (0.05, 0.09, 0.3)

        self.p = SimplePacker(*self.container_size)

        self.moveit_api = MoveItApi(
            self,
            "panda_link0",
            "panda_hand_tcp",
            "panda_manipulator",
            "/franka/joint_states",
        )
        self.grasp_planner = GraspPlanner(self.moveit_api, "panda_gripper/grasp")

        # self.grasp_action_client = ActionClient(self, Grasp, "panda_gripper/grasp")

        self.grasp_process = ActionClient(
            self, GraspProcess, "grasp_process", callback_group=ReentrantCallbackGroup()
        )

        self.approach_pose = Pose(
            position=Point(x=0.0, y=0.0, z=-0.16), orientation=Quaternion()
        )
        self.grasp_pose = Pose(
            position=Point(x=0.0, y=0.0, z=-0.06), orientation=Quaternion()
        )
        self.retreat_pose = Pose(
            position=Point(x=0.0, y=0.0, z=-0.16), orientation=Quaternion()
        )

        self.q = Quaternion(x=0, y=0, z=0, w=1)

        self.pick_server = ActionServer(self,
                                        EmptyAction,
                                        "pick",
                                        self.pick_callback,
                                        callback_group=ReentrantCallbackGroup())
        
        self.place_serve = ActionServer(self,
                                        EmptyAction,
                                        "place",
                                        self.place_callback,
                                        callback_group=ReentrantCallbackGroup())

        self.adjust_serve = ActionServer(self,
                                        EmptyAction,
                                        "adjust",
                                        self.adjust_callback,
                                        callback_group=ReentrantCallbackGroup())


        self.size_subscriber = self.create_subscription(
            Point,
            "/dimension",
            self.size_callback,
            10,
        )

        self.dims = []
    
    # return the dimension of the object
    def size_callback(self, msg):
        self.size_x = msg.x
        self.size_y = msg.y
        # self.get_logger().info("size_x") 
        # self.get_logger().info(self.size_x.__str__()) 
        # self.get_logger().info("size_y") 
        # self.get_logger().info(self.size_y.__str__()) 

    async def pick_callback(self, goal_handle):
        # tf_total = []
        # while len(tf_total) < 10000:
    
        try:
            tf = self.buffer.lookup_transform("panda_link0", "object", Time())
            self.get_logger().info(tf.__str__()) 

        except Exception as e:
            self.get_logger().error("no transform")
            return

        # self.dims.append((round(self.size_x/100,2), round(self.size_y/100,2), 0.05))
        dim_x = round(self.size_x)/100
        dim_y = round(self.size_y)/100
        self.dims.append((dim_x+0.0039, dim_y+0.00345, 0.045))
        self.get_logger().warn(dim_x.__str__()) 
        self.get_logger().warn(dim_y.__str__()) 
        self.get_logger().warn("rect_info")
        # self.get_logger().warn((self.dims).__str__()) 
        rects = [Rect(d) for d in self.dims]
        self.rect = self.p.fit(rects)

        # self.get_logger().warn(self.rect[0].fit.position.__str__()) 

        approach_pose = tf2_geometry_msgs.do_transform_pose(
            self.approach_pose, tf)
        
        grasp_pose = tf2_geometry_msgs.do_transform_pose(
            self.grasp_pose, tf)
        
        self.get_logger().warn(grasp_pose.__str__()) 

        grasp_plan = GraspPlan(
            approach_pose=approach_pose,
            grasp_pose=grasp_pose,
            # place_pose = None,
            grasp_command=Grasp.Goal(
                width=0.02,  # open the gripper wider to release the kettle
                force=50.0,
                speed=0.05,
            ),
            retreat_pose=approach_pose
        )

        await self.grasp_planner.execute_grasp_plan(grasp_plan)
        goal_handle.succeed()
        return EmptyAction.Result()   

    async def place_callback(self, goal_handle):
        try:
            tf = self.buffer.lookup_transform("panda_link0", "box", Time())
            self.get_logger().info(tf.__str__()) 

        except Exception as e:
            self.get_logger().error("no transform")
            return
        i = len(self.dims)
        approach_pose = Pose(
            position=Point(x=self.rect[i-1].fit.position[0], y=self.rect[i-1].fit.position[1], z=-(float(self.rect[i-1].fit.position[2])+0.180)), orientation=self.q
        )

        place_pose = Pose(
            position=Point(x=self.rect[i-1].fit.position[0], y=self.rect[i-1].fit.position[1], z=-(float(self.rect[i-1].fit.position[2])+0.088)), orientation=self.q
        )
        
        approach_pose = Pose(
            position=Point(x=self.rect[i-1].fit.position[0], y=self.rect[i-1].fit.position[1], z=-0.30), orientation=self.q
        )

        self.get_logger().info("rect") 
        # self.get_logger().info((float(self.rect[i-1].fit.position[2])+0.08).__str__()) 
        self.get_logger().info(self.rect[i-1].fit.position.__str__()) 
        self.rect.clear()
        self.p = SimplePacker(*self.container_size)

        place_pose = tf2_geometry_msgs.do_transform_pose(
                    place_pose, tf)
        approach_pose = tf2_geometry_msgs.do_transform_pose(
                    approach_pose, tf)
        
        self.get_logger().info(approach_pose.__str__()) 
        
        grasp_plan = GraspPlan(
            approach_pose=approach_pose,
            grasp_pose=place_pose,
            grasp_command=Grasp.Goal(
                width=0.02,  # open the gripper wider to release the kettle
                force=50.0,
                speed=0.2,
            ),
            retreat_pose=approach_pose
        )

        await self.grasp_planner.execute_grasp_plan(grasp_plan)

        goal_handle.succeed()
        return EmptyAction.Result() 

    async def adjust_callback(self, goal_handle):
        try:
            tf = self.buffer.lookup_transform("panda_link0", "object", Time())
            self.get_logger().info(tf.__str__()) 

            adjust_tf = self.buffer.lookup_transform("object", "d435i_color_optical_frame", Time())

        except Exception as e:
            self.get_logger().error("no transform")
            return
        
        relate_pose = Pose(
                    position=Point(x=adjust_tf._transform.translation.x, y=adjust_tf._transform.translation.y, z=adjust_tf.transform.translation.z), orientation=Quaternion()
                )
        
        grasp_pose = tf2_geometry_msgs.do_transform_pose(
            relate_pose, tf)
        self.get_logger().info("pose_adas") 
        self.get_logger().info(grasp_pose.position.__str__()) 

        await self.moveit_api.plan_async(            
            point=grasp_pose.position,
            orientation=Quaternion(x=1.0,y=0.0,z=0.0,w=0.0),
            execute=True
            )
        
        goal_handle.succeed()
        return EmptyAction.Result()   

def main(args=None):
    rclpy.init(args=args)
    res = pick_place()
    rclpy.spin(res)
    rclpy.shutdown()