#!/usr/bin/env python

import os
from re import L
import rospy
from nav_msgs.srv import GetPlan, GetPlanRequest
from navfn.srv import MakeNavPlan, MakeNavPlanRequest
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist
import rvo2
from tf.transformations import euler_from_quaternion
import math
import copy
import tf2_ros
from rospkg import RosPack
import yaml


class SimulationHandler:
    num_hum = 1

    def __init__(self, num_humans = None, config = None, debug = False):
        self.num_hum = num_humans if num_humans is not None else self.num_hum

        self.config = config
        self.debug = debug

        humans = [Human(i, self.config, self.debug) for i in range(self.num_hum)]

class Human:
    def __init__(self, id = 0, config = None, debug = False):
        self.id = id
        self.orca_id = None
        self.current_pose = PoseStamped()
        self.current_twist = Twist()
        self.current_yaw = 0.0

        self.debug = debug

        if config is not None:

            config = os.path.abspath(config)
            assert os.path.exists(config)

            with open(config, "r") as file:
                self.configs = yaml.safe_load(file)
                self.configs = self.configs["human_{}".format(self.id + 1)] # Select respective config
        else:
            raise Exception("Expected a valid configurations. Received {}".format(config))

        # Load configs
        self.get_configs()
    
        # ID is started from 1. There is a default Inhus Human with id 1
        topic_header = "/morse_agents/human{}".format(self.id + 2)

        self.sub = rospy.Subscriber(topic_header + "/odom", Odometry, self._pose_callback)


        # Setup publishers
        self.path_pub = rospy.Publisher(topic_header + "/path_viz", Path, queue_size=600)
        self.vel_publisher = rospy.Publisher(topic_header + "/cmd_vel", Twist, queue_size=10, latch=False)


        # Planner Server
        # TODO: Acquire planner name before creating the service
        self.plan_service = MakeNavPlan()
        self.plan = None        

        # Setup path message
        self.path = Path()
        self.path.header.frame_id = "map"

        # Setup goal message
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = "map"

        self.goal_twist = Twist()



    def _pose_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.current_twist = msg.twist.twist

        (_, _, self.current_yaw) = euler_from_quaternion(
            [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
            ]
        )

        if self.debug:
            print("Human {}: Current Pose - {}\n".format(self.id + 2, self.current_pose.position))
            print("Human {}: Current Twist - {}\n".format(self.id + 2, self.current_twist))
            print("Human {}: Current Yaw - {}\n".format(self.id + 2, self.current_yaw))

    
    def get_pose(self):
        return self.current_pose
    
    def set_goal(self, x, y):
        self.goal_pose.header.stamp = rospy.Time(0)
        self.goal_pose.pose.pose.position.x = x
        self.goal_pose.pose.pose.position.y = y

    def get_configs(self):

        self.loop = self.configs["loop_path"]
        self.planner_name = self.configs["planner_name"]
        print("Selected Planner: {}".format(self.planner_name))

        
        # Get goal poses
        self.scenarios = self.configs["scenarios"]
        scene_id = self.scenarios["selected_id"]

        print("Selected scenario: {}".format(scene_id))

        self.goals = self.scenarios["goals"][scene_id]
        print("Selected goals: {}".format(self.goals))


        L
        
    def make_plan(self, goal = None):
        
        pass

    def _compute_vel_and_orientation(self):
        """
        Computes velocity and orientation
        """
        pass

    def update_step(self):

        # Send path to rviz
        self.path.header.stamp = rospy.Time(0)
        self.path.poses = self.plan.plan.poses
        self.path_pub.publish(self.path)

        # Update cmd_vel
        self.vel_pub.publish(self.goal_twist)



def main():
    rospy.init_node('local_collision_avoidance')
    
    pkg_path = RosPack().get_path("morse_ros")
    supervisor = SimulationHandler(config=os.path.join(pkg_path, "./configs", "laas_adream.yaml"), debug = False)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():

        rate.sleep()


if __name__ == "__main__":
    main()

    
