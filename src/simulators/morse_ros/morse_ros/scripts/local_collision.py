#!/usr/bin/env python

import os
import rospy
from navfn.srv import MakeNavPlan
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist
import rvo2
from tf.transformations import euler_from_quaternion
import math
from copy import deepcopy
import numpy as np
from rospkg import RosPack
import yaml

SIM = rvo2.PyRVOSimulator(1/60.0, 1.5, 3, 2.5, 4.5, 0.4, 1.2) #TODO:better way to give these parameters
RATE_HZ = 20.0

class SimulationHandler:

    def __init__(self, config = None, rate = 1, debug = False):

        self.config = config
        self.debug = debug

        if config is not None:

            config = os.path.abspath(config)
            assert os.path.exists(config)

            with open(config, "r") as file:
                configs = yaml.safe_load(file)
                self.num_hum = configs["num_humans"]
        else:
            raise Exception("Expected a valid configurations. Received {}".format(config))

        self.humans = [Human(i, self.config, self.debug) for i in range(self.num_hum)]

        rospy.sleep(2.0)
        self.rate = rospy.Rate(rate)

    def run(self):
        while not rospy.is_shutdown():
            for human in self.humans:
                human.update_step()

            self.rate.sleep()

class Human:

    _MAKE_NEW_PLAN = True

    def __init__(self, id = 0, config = None, debug = False):
        self.id = id
        self.orca_id = None
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

        # Setup subscribers
        self.sub = rospy.Subscriber(topic_header + "/odom", Odometry, self._pose_callback)

        # Setup publishers
        self.path_pub = rospy.Publisher(topic_header + "/path_viz", Path, queue_size=600)
        self.vel_pub = rospy.Publisher(topic_header + "/cmd_vel", Twist, queue_size=10, latch=False)

        # Setup path message
        self.path = Path()
        self.path.header.frame_id = "map"

        # Setup current pose message
        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = "map"

        # Setup goal messages
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = "map"
        self.goal_twist = Twist()
        self.goal_twist_world = Twist()

        #CHECK:Additional parameters for world frame to human frame 
        self.plan_twist_world = Twist()
        self.yaw_orig_world = None
        self.omega = None

        # Planner Server
        rospy.logdebug("Human {}: Waiting for `{}` service".format(self.id + 1, self.planner_name))
        rospy.wait_for_service(self.planner_name)
        rospy.logdebug("Human {}: Planner service `{}` acquired".format(self.id + 1, self.planner_name))

        self.planner_service = rospy.ServiceProxy(self.planner_name, MakeNavPlan)
        self.plan = None
        self.path_list = []

        # Finally, setup ORCA
        self.add_to_orca()

    def add_to_orca(self):
        self.orca_id = SIM.addAgent((self.current_pose.pose.position.x, self.current_pose.pose.position.y))

    def _pose_callback(self, msg):
        self.current_pose.pose = msg.pose.pose
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
        self.goal_pose.pose.position.x = x
        self.goal_pose.pose.position.y = y

    def get_configs(self):

        self.loop = self.configs["loop_path"]
        self.planner_name = self.configs["planner_name"]
        # Get goal poses
        self.scenarios = self.configs["scenarios"]
        scene_id = self.scenarios["selected_id"]

        self.goals = self.scenarios["goals"][scene_id]
        self.goals_bkup = deepcopy(self.goals)

        if self.debug:
            print("Selected Planner: {}".format(self.planner_name))
            print("Selected scenario: {}".format(scene_id))
            print("Selected goals: {}".format(self.goals))
        
    def make_plan(self):
        start = PoseStamped()
        start.header.seq = 0
        start.header.frame_id = "map"
        start.header.stamp = rospy.Time(0)
        start.pose.position.x = self.current_pose.pose.position.x #  database.start_human2_pose_x   
        start.pose.position.y = self.current_pose.pose.position.y #database.start_human2_pose_y

        goal = PoseStamped()
        goal.header.seq = 0
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time(0)
        goal.pose.position.x = self.goal_pose.pose.position.x #database.goal_human2_pose_x 
        goal.pose.position.y = self.goal_pose.pose.position.y #database.goal_human2_pose_y
    

        r = MakeNavPlan()
        r.start = start
        r.goal = goal
        
        self.plan = self.planner_service(r.start, r.goal)

    def _compute_vel_and_orientation(self, desired_pose):
        """
        Computes velocity and orientation
        """
        # data_back = copy.deepcopy(self) #TODO: can just copy current pose and yaw

        self.plan_twist_world.linear.x = (desired_pose.pose.position.x - self.current_pose.pose.position.x) / (1/RATE_HZ)
        self.plan_twist_world.linear.y = (desired_pose.pose.position.y - self.current_pose.pose.position.y) / (1/RATE_HZ)
        
        (_, _, self.yaw_orig_world) = euler_from_quaternion(
            [
            desired_pose.pose.orientation.x,
            desired_pose.pose.orientation.y,
            desired_pose.pose.orientation.z,
            desired_pose.pose.orientation.w,
            ]
        )

        self.omega = self.normalize_theta(self.yaw_orig_world - self.current_yaw)/ (1/RATE_HZ)
        
    @staticmethod
    def normalize_theta(theta):
        PI = math.pi
        result = math.fmod(theta + PI, 2.0 * PI)
        if result <= 0:
            return result + PI
        return result - PI
    
    @staticmethod
    def goal_checker(current_pose, goal_pose, threshold):
        if np.linalg.norm([
            goal_pose.pose.position.x - current_pose.pose.position.x,
            goal_pose.pose.position.y - current_pose.pose.position.y
        ]) < threshold:
            return True
        else:
            return False

    def update_step(self):
        
        if self.goal_checker(self.current_pose, self.goal_pose, 0.2): # and len(self.path_list) < 2:
            self._MAKE_NEW_PLAN = True
            self.goal_twist.linear.x = 0.0
            self.goal_twist.linear.y = 0.0
            self.goal_twist.angular.z = 0.0

            # If looping is enabled, reverse
            if self.loop:
                self.goals_bkup.reverse()
                self.goals = deepcopy(self.goals_bkup)
                self.set_goal(self.goals[0][0], self.goals[0][1])

        elif self.current_pose != self.goal_pose:
            # Make a new plan for the next goal position
            if self.goals != [] and self._MAKE_NEW_PLAN:
                goal = self.goals[0]
                self.set_goal(goal[0], goal[1])
                self.make_plan()
            
                # Process the existing plan
                self.path_list = self.plan.path
                self._MAKE_NEW_PLAN = False
                
                # Remove the goal that has been already planned
                self.goals.remove(goal)

            try:
                desired_pose = self.path_list[0]
                self.path_list.remove(desired_pose)
                self._compute_vel_and_orientation(desired_pose)

                # Set position and veloctiy
                SIM.setAgentPosition(self.orca_id, (self.current_pose.pose.position.x, self.current_pose.pose.position.y))
                SIM.setAgentPrefVelocity(self.orca_id, (self.plan_twist_world.linear.x, self.plan_twist_world.linear.y))

                SIM.doStep()

                # Get twist
                self.goal_twist_world.linear.x, self.goal_twist_world.linear.y = SIM.getAgentVelocity(self.orca_id) #Doubt send number or structure
                
                # Convert the velocity to human frame
                self.goal_twist.linear.x = self.goal_twist_world.linear.x*math.cos(self.yaw_orig_world)+ self.goal_twist_world.linear.y*math.sin(self.yaw_orig_world) #vel*math.cos(self.yaw_orig_world) 
                self.goal_twist.linear.y = -self.goal_twist_world.linear.x*math.sin(self.yaw_orig_world)+ self.goal_twist_world.linear.y*math.cos(self.yaw_orig_world) #vel*math.sin(yaw)

                self.goal_twist.angular.z = self.omega
            except IndexError as e:
                pass

            # Send path to rviz
            self.path.header.stamp = rospy.Time(0)
            self.path.poses = self.plan.path
            self.path_pub.publish(self.path)

        # Update cmd_vel
        self.vel_pub.publish(self.goal_twist)


def main():
    rospy.init_node('local_collision_avoidance')
    
    pkg_path = RosPack().get_path("morse_ros")
    simulator = SimulationHandler(config=os.path.join(pkg_path, "./configs", "laas_adream.yaml"), rate=RATE_HZ, debug = False)

    try:
        simulator.run()
    finally:
        pass


if __name__ == "__main__":
    main()

    
