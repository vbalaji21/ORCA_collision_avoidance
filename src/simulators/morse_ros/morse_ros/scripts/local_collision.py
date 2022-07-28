#!/usr/bin/env python

import os
import rospy
from navfn.srv import MakeNavPlan
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist
import rvo2
from transformations import euler_from_quaternion
import math
from copy import deepcopy
import numpy as np
from rospkg import RosPack
import yaml
import random

timestep = 1 / 60.0
neighborDist = 2.5  # 1.5
maxNeighbors = 5.0
timeHorizon = 1.5
timeHorizonObst = 1.5  # 1.0
radius = 0.4
maxSpeed = 1.8

RATE_HZ = 20.0

SIM = rvo2.PyRVOSimulator(
    timestep, neighborDist, maxNeighbors, timeHorizon, timeHorizonObst, radius, maxSpeed
)  # TODO:better way to give these parameters


class SimulationHandler:
    def __init__(self, config=None, rate=1, debug=False):

        self.config = config
        self.debug = debug

        # TODO:change place
        self.orca_robot_id = None
        self.inhus_robot_id = None
        self.current_robot_pose = PoseStamped()
        self.current_inhus_pose = PoseStamped()

        if config is not None:

            config = os.path.abspath(config)
            assert os.path.exists(config)

            with open(config, "r") as file:
                configs = yaml.safe_load(file)
                self.num_hum = configs["num_humans"]
        else:
            raise Exception(
                "Expected a valid configurations. Received {}".format(config)
            )

        self.total_generated_humans = [
            Human(i, self.config, self.debug) for i in range(self.num_hum)
        ]

        self.humans = self.total_generated_humans

        # TODO: chenge this subscriber place and write better
        rospy.Subscriber(
            "/morse_agents/human1/odom",
            Odometry,
            self.update_inhus_pose,
            self.current_inhus_pose,
        )
        rospy.Subscriber(
            "/odom", Odometry, self.update_robot_pose, self.current_robot_pose
        )

        self.add_inhus_and_cohan()
        self.add_sim_obstacles()

        rospy.sleep(2.0)
        self.rate = rospy.Rate(rate)

    def reset_humans(self, num_hum):  # Swapping the required human vectors in the list
        # TODO:write an assert statement to check number of humans selected is not greater than total number of humans
        assert num_hum <= self.num_hum
        self.humans = self.total_generated_humans[:num_hum]

    def update_inhus_pose(self, data, args):

        args.pose.position.x = data.pose.pose.position.x
        args.pose.position.y = data.pose.pose.position.y

    def update_robot_pose(seolf, data, args):

        args.pose.position.x = data.pose.pose.position.x
        args.pose.position.y = data.pose.pose.position.y

    def add_inhus_and_cohan(self):
        self.inhus_robot_id = SIM.addAgent(
            (
                self.current_inhus_pose.pose.position.x,
                self.current_inhus_pose.pose.position.y,
            )
        )
        self.orca_robot_id = SIM.addAgent(
            (
                self.current_robot_pose.pose.position.x,
                self.current_robot_pose.pose.position.y,
            )
        )

    def update_robot_and_inhus_position(self):

        SIM.setAgentPosition(
            self.inhus_robot_id,
            (
                self.current_inhus_pose.pose.position.x,
                self.current_inhus_pose.pose.position.y,
            ),
        )
        SIM.setAgentPosition(
            self.orca_robot_id,
            (
                self.current_robot_pose.pose.position.x,
                self.current_robot_pose.pose.position.y,
            ),
        )

    def add_sim_obstacles(self):
        o1 = SIM.addObstacle(
            [(1.5, -0.117), (5.5, -0.177), (5.5, -0.0143), (1.5, -0.0143)]
        )  # Room 1 sidewall
        o2 = SIM.addObstacle(
            [(5.77, 18.3), (11.0, 18.3), (11.0, 18.5), (5.77, 18.5)]
        )  # Room 2 sidewall
        o3 = SIM.addObstacle(
            [(7.14, 12.5), (8.34, 12.5), (8.34, 12.7), (7.14, 12.7)]
        )  # Inter room entrance near room 2
        o9 = SIM.addObstacle(
            [(6.21, 3.32), (9.52, 3.32), (9.52, 4.25), (6.21, 4.25)]
        )  # Interroom enrance near room 1
        o12 = SIM.addObstacle(
            [(1.3, 4.64), (2.3, 4.64), (2.3, 12.5), (1.3, 12.5)]
        )  # ideblocked space near interrrom
        o14 = SIM.addObstacle([(6.4, 7.42), (6.54, 7.42)])
        o20 = SIM.addObstacle(
            [(6.54, 8.47), (6.4, 8.47)]
        )  # ideblocked space near interrrom
        o15 = SIM.addObstacle(
            [(6.43, 10.9), (6.43, 9.59)]
        )  # , (2.3, 12.5), (1.3, 12.5)]) #ideblocked space near interrrom
        o16 = SIM.addObstacle([(7.73, 8.33), (8.36, 8.33)])
        o17 = SIM.addObstacle([(7.73, 8.47), (8.36, 8.47)])
        o18 = SIM.addObstacle([(4.22, 8.33), (5.45, 8.33)])
        o19 = SIM.addObstacle([(4.22, 8.47), (5.45, 8.47)])
        o21 = SIM.addObstacle([(4.00, 4.84), (5.45, 4.84)])

        k1 = SIM.processObstacles()

    def run(self, scenario):
        # while not rospy.is_shutdown(): #CHECKIT: removed to run randomisation while for GUI
        # TODO:Add a start button to give goals and take the goal giving logic out of it
        if scenario.reset_status == True:
            i = 0
            for human in self.humans:
                human.vel_pub.publish(human.reset_twist)

                human.goals = [
                    scenario.goals[i]
                ]  # TODO: definitely change this dirty logic
                i = i + 1  # TODO: definitely change this dirty logic
                human._MAKE_NEW_PLAN = True  # Ask every human to make a new plan

                # clear previous visualisation path
                human.reset_path_viz.header.frame_id = "map"
                human.reset_path_viz.header.stamp = rospy.Time(0)
                human.path_pub.publish(human.reset_path_viz)

            self.reset_humans(scenario.current_num_orca_hum)
            scenario.reset_status = False
        for human in self.humans:
            human.update_step()
            self.update_robot_and_inhus_position()

        self.rate.sleep()


class Human:

    _MAKE_NEW_PLAN = True

    def __init__(self, id=0, config=None, debug=False):
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
                self.configs = self.configs[
                    "human_{}".format(self.id + 1)
                ]  # Select respective config
        else:
            raise Exception(
                "Expected a valid configurations. Received {}".format(config)
            )

        # Load configs
        self.get_configs()

        # ID is started from 1. There is a default Inhus Human with id 1
        topic_header = "/morse_agents/human{}".format(self.id + 2)

        # Setup subscribers
        self.sub = rospy.Subscriber(
            topic_header + "/odom", Odometry, self._pose_callback
        )

        # Setup publishers
        self.path_pub = rospy.Publisher(
            topic_header + "/path_viz", Path, queue_size=600
        )
        self.vel_pub = rospy.Publisher(
            topic_header + "/cmd_vel", Twist, queue_size=10, latch=False
        )

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

        # reset twist (zero) and path viz before human reset
        self.reset_twist = Twist()  # Defualt value is zero, good
        self.reset_twist = Twist()  # Defualt value is zero, good
        self.reset_path_viz = Path()

        # CHECK:Additional parameters for world frame to human frame
        self.plan_twist_world = Twist()
        self.yaw_orig_world = None
        self.omega = None

        # Planner Server
        rospy.logdebug(
            "Human {}: Waiting for `{}` service".format(self.id + 1, self.planner_name)
        )
        rospy.wait_for_service(self.planner_name)
        rospy.logdebug(
            "Human {}: Planner service `{}` acquired".format(
                self.id + 1, self.planner_name
            )
        )

        self.planner_service = rospy.ServiceProxy(self.planner_name, MakeNavPlan)
        self.plan = None
        self.path_list = []

        # Finally, setup ORCA
        self.add_to_orca()

    def add_to_orca(self):
        self.orca_id = SIM.addAgent(
            (self.current_pose.pose.position.x, self.current_pose.pose.position.y)
        )

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
            print(
                "Human {}: Current Pose - {}\n".format(
                    self.id + 2, self.current_pose.position
                )
            )
            print(
                "Human {}: Current Twist - {}\n".format(self.id + 2, self.current_twist)
            )
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
        start.pose.position.x = (
            self.current_pose.pose.position.x
        )  #  database.start_human2_pose_x
        start.pose.position.y = (
            self.current_pose.pose.position.y
        )  # database.start_human2_pose_y

        goal = PoseStamped()
        goal.header.seq = 0
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time(0)
        goal.pose.position.x = (
            self.goal_pose.pose.position.x
        )  # database.goal_human2_pose_x
        goal.pose.position.y = (
            self.goal_pose.pose.position.y
        )  # database.goal_human2_pose_y

        r = MakeNavPlan()
        r.start = start
        r.goal = goal

        self.plan = self.planner_service(r.start, r.goal)

        # while not (self.plan.plan_found == 1):
        #     print("Im in polanning loop", self.orca_id)
        #     self.plan = self.planner_service(r.start, r.goal)

        if self.plan.plan_found == 0:
            self._MAKE_NEW_PLAN = True
        elif self.plan.plan_found == 1:
            self._MAKE_NEW_PLAN = False

    def _compute_vel_and_orientation(self, desired_pose):
        """
        Computes velocity and orientation
        """
        # data_back = copy.deepcopy(self) #TODO: can just copy current pose and yaw
        self.plan_twist_world.linear.x = (
            desired_pose.pose.position.x - self.current_pose.pose.position.x
        ) / (1 / RATE_HZ)
        self.plan_twist_world.linear.y = (
            desired_pose.pose.position.y - self.current_pose.pose.position.y
        ) / (1 / RATE_HZ)

        (_, _, self.yaw_orig_world) = euler_from_quaternion(
            [
                desired_pose.pose.orientation.x,
                desired_pose.pose.orientation.y,
                desired_pose.pose.orientation.z,
                desired_pose.pose.orientation.w,
            ]
        )

        omega = self.normalize_theta(self.yaw_orig_world - self.current_yaw) / (
            1 / RATE_HZ
        )
        self.omega = self.clamp(omega, -12.5, 12.5)

    @staticmethod
    def normalize_theta(theta):
        PI = math.pi
        result = math.fmod(theta + PI, 2.0 * PI)
        if result <= 0:
            return result + PI
        return result - PI

    @staticmethod
    def goal_checker(current_pose, goal_pose, threshold):
        if (
            np.linalg.norm(
                [
                    goal_pose.pose.position.x - current_pose.pose.position.x,
                    goal_pose.pose.position.y - current_pose.pose.position.y,
                ]
            )
            < threshold
        ):
            return True
        else:
            return False

    @staticmethod
    def clamp(x, minn, maxx):
        return x if x > minn and x < maxx else (minn if x < minn else maxx)

    def update_step(self):

        if self.goal_checker(
            self.current_pose, self.goal_pose, 0.2
        ):  # or (self._MAKE_NEW_PLAN == False and len(self.path_list) < 2)):
            self._MAKE_NEW_PLAN = True
            self.goal_twist.linear.x = 0.0
            self.goal_twist.linear.y = 0.0
            self.goal_twist.angular.z = 0.0

            # If looping is enabled, reverse
            if self.loop:
                # self.goals_bkup.reverse()
                # self.goals = deepcopy(self.goals_bkup)
                # self.set_goal(self.goals[0][0], self.goals[0][1])

                self.goals_bkup = deepcopy(self.goals)
                finished_goal = self.goals_bkup.pop(0)
                self.goals_bkup.append(finished_goal)
                self.goals = deepcopy(self.goals_bkup)

        else:  # self.current_pose != self.goal_pose:
            # Make a new plan for the next goal position
            if self.goals != [] and self._MAKE_NEW_PLAN:
                # print("hereeeee flag and goal", self._MAKE_NEW_PLAN, self.goals, self.orca_id)
                goal = self.goals[0]
                self.set_goal(goal[0], goal[1])
                self.make_plan()

                # Process the existing plan
                if (self.plan.plan_found == 1) and (len(self.plan.path) != 0):
                    self.path_list = self.plan.path
                # self._MAKE_NEW_PLAN = False

                # Remove the goal that has been already planned
                self.goals.remove(goal)

            try:
                # print("diffffff here", self._MAKE_NEW_PLAN)
                if len(self.path_list) != 0:
                    desired_pose = self.path_list[0]
                    self.path_list.remove(desired_pose)
                    self._compute_vel_and_orientation(desired_pose)

                    # Replan if it moves 1 m away from desired position in trajectory
                    if False == self.goal_checker(self.current_pose, desired_pose, 0.3):
                        self.make_plan()
                        # copy new plan

                        if (self.plan.plan_found == 1) and (len(self.plan.path) != 0):
                            self.path_list = self.plan.path

                    # Set position and veloctiy
                    SIM.setAgentPosition(
                        self.orca_id,
                        (
                            self.current_pose.pose.position.x,
                            self.current_pose.pose.position.y,
                        ),
                    )

                    SIM.setAgentPrefVelocity(
                        self.orca_id,
                        (
                            self.plan_twist_world.linear.x,
                            self.plan_twist_world.linear.y,
                        ),
                    )

                    SIM.doStep()

                    # Get twist
                    (
                        self.goal_twist_world.linear.x,
                        self.goal_twist_world.linear.y,
                    ) = SIM.getAgentVelocity(
                        self.orca_id
                    )  # Doubt send number or structure

                    # Convert the velocity to human frame
                    self.goal_twist.linear.x = (
                        self.goal_twist_world.linear.x * math.cos(self.yaw_orig_world)
                        + self.goal_twist_world.linear.y * math.sin(self.yaw_orig_world)
                    )  # vel*math.cos(self.yaw_orig_world)
                    self.goal_twist.linear.y = (
                        -self.goal_twist_world.linear.x * math.sin(self.yaw_orig_world)
                        + self.goal_twist_world.linear.y * math.cos(self.yaw_orig_world)
                    )  # vel*math.sin(yaw)

                    self.goal_twist.angular.z = self.omega

                else:
                    # self.vel_pub.publish(self.reset_twist)
                    self.goal_twist.linear.x = 0.0
                    self.goal_twist.linear.y = 0.0
                    self.goal_twist.angular.z = 0.0
                    # print("reset zero cmd vel and id, length of path list", self.orca_id, len(self.path_list))

            except IndexError as e:
                pass

        # Send path to rviz
        self.path.header.stamp = rospy.Time(0)
        self.path.poses = self.path_list  # self.plan.path
        self.path_pub.publish(self.path)
        # Update cmd_vel
        self.vel_pub.publish(self.goal_twist)


# #Pertubation put it inside try or just before goal_twist calculation
# q = random.randrange(1.0, 10.0, 10.0)
# print("q", q)
# angle = (q) *0.1  # std::rand() * 2.0f * M_PI / RAND_MAX;
# dist = (q) * 0.0001 #  std::rand() * 0.0001f / RAND_MAX;
# self.plan_twist_world.linear.x = self.plan_twist_world.linear.x + dist*math.cos(angle)
# self.plan_twist_world.linear.y = self.plan_twist_world.linear.y + dist*math.sin(angle)

# def main():
#     rospy.init_node('local_collision_avoidance')

#     pkg_path = RosPack().get_path("morse_ros")
#     simulator = SimulationHandler(config=os.path.join(pkg_path, "./configs", "laas_adream.yaml"), rate=RATE_HZ, debug = False)

#     try:
#         simulator.run()
#     finally:
#         pass


# if __name__ == "__main__":
#     main()
