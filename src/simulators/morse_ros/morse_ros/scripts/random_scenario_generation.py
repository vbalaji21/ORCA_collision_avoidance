import math
import os
import random
import tkinter as tk

import numpy as np
import pymorse
import rospy
import yaml
from geometry_msgs.msg import PoseStamped, Twist
from inhus.msg import Goal  # check whether it got imported
from rospkg import RosPack
from transformations import quaternion_from_euler

from local_collision import SimulationHandler

RATE_HZ = 20.0  # TODO:put this in a proper place, eg:- Inside simulator class

# Note, make this change in code - Simultor refers to Morse, ORCA or rvo2 simulator has to be mentioned explicitly
class ScenarioCreator(SimulationHandler):  # This is rvo2 simulator

    gui = tk.Tk()
    reset_status = False  # CHANGE: Had to put these variables outside to share it with other class in another file
    total_orca_hum = None  # Added for conveinece and read at startup, did instead of passing simulator at unwanted places
    current_num_orca_hum = None

    def __init__(self, SimulationHandler):
        sim_instance = None
        self.sim_human_id = [
            "Human.001",
            "Human.002",
            "Human.003",
            "Human.004",
        ]  # TODO:can read it from simulator itself
        self.total_orca_hum = SimulationHandler.num_hum
        self.goals = [[0.0, 0.0]] * (self.total_orca_hum + 2)
        self.NUM_GOALS = 10

        # Robot and Inhus publishers
        self.robot_goal_pub = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=10
        )
        self.inhus_goal_pub = rospy.Publisher(
            "/boss/human/new_goal", Goal, queue_size=10
        )

        self._simulator_instance()

    def _simulator_instance(self):
        self.sim_instance = pymorse.Morse()

    def _robot_velocity_zero(self):
        robot_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10, latch=False)
        reset_twist = Twist()
        robot_vel.publish(reset_twist)

    def _inhus_velocity_zero(self):
        inhus_vel = rospy.Publisher(
            "/morse_agents/human1/cmd_vel", Twist, queue_size=10, latch=False
        )
        reset_twist = Twist()
        inhus_vel.publish(reset_twist)

    def _orca_velocity_zero(self):  # TODO
        reset_twist = Twist()

        for i in range(self.total_orca_hum):
            replacement = str(2 + i)
            topic_header = "/morse_agents/human{}".format(replacement)
            # print("replacement", i)
            # # topic_header.replace(topic_header[size - 2:], replacement)
            # print("topic header", topic_header)
            orca_vel_pub = rospy.Publisher(
                topic_header + "/cmd_vel", Twist, queue_size=10, latch=False
            )
            orca_vel_pub.publish(reset_twist)
        pass

    def _simulator_reset_robot_pose(self, x, y, yaw):
        self._robot_velocity_zero()
        self.sim_instance.rpc(
            "simulation", "set_object_position", "pr2", [x, y, 0], [0.0, 0.0, yaw]
        )

    def _simulator_reset_inhus_pose(self, x, y, yaw):
        self._inhus_velocity_zero()
        self.sim_instance.rpc(
            "simulation", "set_object_position", "Human", [x, y, 0], [0.0, 0.0, yaw]
        )

    def _simulator_reset_orca_humans_pose(
        self, poses, yaw
    ):  # TODO:write as for loop and string match
        self._orca_velocity_zero()

        # ORCA human
        for i in range(self.current_num_orca_hum):
            # print("i", i)
            self.sim_instance.rpc(
                "simulation",
                "set_object_position",
                self.sim_human_id[i],
                [(poses[(int(2 + i))][0]), (poses[(int(2 + i))][1]), 0],
                [0.0, 0.0, yaw[(int(2 + i))][0]],
            )

        # logic to put remaining orca human out of the map
        if self.current_num_orca_hum != self.total_orca_hum:
            diff = self.total_orca_hum - self.current_num_orca_hum
            for i in range(diff):
                # print("i, curr, total", i, self.current_num_orca_hum, self.total_orca_hum)
                yaw = random.uniform(math.pi, -math.pi)
                self.sim_instance.rpc(
                    "simulation",
                    "set_object_position",
                    self.sim_human_id[(self.current_num_orca_hum + i)],
                    [200.0, (200.0 + i), 0],
                    [0.0, 0.0, yaw],
                )

    def _set_robot_goal_pose(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.seq = 0
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time(0)

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0

        q = quaternion_from_euler(0.0, 0.0, yaw)

        # print("orn quat", q)

        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        self.robot_goal_pub.publish(msg)

    def _set_inhus_goal_pose(self, x, y, yaw):
        msg = Goal()

        msg.type = "pose_goal"
        msg.pose_goal.pose.x = x
        msg.pose_goal.pose.y = y
        msg.pose_goal.pose.theta = yaw

        msg.pose_goal.radius = 0.0

        self.inhus_goal_pub.publish(msg)
        pass

    def _set_orca_humans_goal_pose(self, poses, yaw):
        # First two sequences are already reserved for robot and inhus
        self.goals = poses[2:]
        pass

    def _extract_circles_info_in_map(self):
        pkg_path = RosPack().get_path("morse_ros")
        circle = os.path.join(pkg_path, "./configs", "laas_adream.yaml")
        circle = os.path.abspath(circle)

        with open(circle, "r") as file:
            read_circles = yaml.safe_load(file)
            num_circles = len(read_circles["radii"])
            diameter_list = read_circles["radii"]

            centre_list = read_circles["centers"]

        return num_circles, diameter_list, centre_list

    def _find_random_unique_list(self, max_num, list_no):
        unique_random_list = random.sample(range(0, (max_num)), (list_no))
        return unique_random_list

    def _generate_poses_from_unique_circles(
        self, unique_circles_list, diameter_list, centre_list
    ):
        poses = [0.0, 0.0] * len(
            unique_circles_list
        )  # CHANGE: (self.total_orca_hum+2) instead of unique_circles_list
        yaw = [0] * len(unique_circles_list)

        for i in range(len(unique_circles_list)):
            selected_circle_radius = diameter_list[(unique_circles_list[i])] / 2
            selected_circle_centre = centre_list[(unique_circles_list[i])]
            (
                x,
                y,
                yaw_current,
            ) = self._generate_random_cartesian_cordinatines_inside_circle(
                selected_circle_centre, selected_circle_radius
            )
            poses[i] = [x, y]
            yaw[i] = [yaw_current]

        return poses, yaw

    def _reset_agent_start_poses(self, poses, yaw):

        # TODO:make all of their velocities too zero before resetting their start positions
        self._simulator_reset_robot_pose(poses[0][0], poses[0][1], yaw[0][0])
        self._simulator_reset_inhus_pose(poses[1][0], poses[1][1], yaw[1][0])
        self._simulator_reset_orca_humans_pose(poses, yaw)

    def _reset_agent_goal_poses(self, poses, yaw):

        self._set_robot_goal_pose(poses[0][0], poses[0][1], yaw[0][0])
        self._set_inhus_goal_pose(poses[1][0], poses[1][1], yaw[1][0])
        self._set_orca_humans_goal_pose(poses, yaw)

    @staticmethod
    def _max_distance_checker(init_pose, next_pose, threshold):
        # print("type 2 poses and generated_pose", type(init_pose), type(next_pose))
        # print("values 2", init_pose, next_pose)
        # print("convergence 2 x next, init", next_pose[0], init_pose[0])
        # print("convergence 2 y next, init", next_pose[1], init_pose[1])
        return bool(
            np.linalg.norm([next_pose[0] - init_pose[0], next_pose[1] - init_pose[1]])
            < threshold
        )

    @staticmethod
    def _min_distance_checker(init_pose, next_pose, threshold):
        return bool(
            np.linalg.norm([next_pose[0] - init_pose[0], next_pose[1] - init_pose[1]])
            > threshold
        )

    def _check_distance_from_remaning_coordinates(
        self, poses, current_index, generated_pose, min_dist, max_dist
    ):
        total_index = len(poses)
        # print("total index length", total_index)
        modified_poses = poses[:current_index]
        len_modified_poses = len(modified_poses)
        # print("len modified list", len_modified_poses)

        for i in range(len_modified_poses):
            if True == (
                self._max_distance_checker(
                    modified_poses[i], generated_pose[0], max_dist
                )
                and self._min_distance_checker(
                    modified_poses[i], generated_pose[0], min_dist
                )
            ):
                status = True
                # print("status true", status)
                # print("poses", poses)
            else:
                status = False
                return status

        return status

    def _generate_random_far_coordinates(
        self,
        start_poses,
        start_range,
        agents_range,
        num_circles,
        diameter_list,
        centre_list,
    ):
        new_poses = []
        new_yaw = []

        # print("poses", start_poses)
        for pose in start_poses:
            x, y, yaw = 0.0, 0.0, 0.0
            i = 0
            while True:
                # print("x, y, yaw and iteration ", x, y, yaw, i)
                # i = i +1
                # generate a random point inside a random circle
                initial_circle = random.randrange(0.0, num_circles, 1.0)
                initial_circle_radius = diameter_list[(initial_circle)] / 2
                initial_circle_centre = centre_list[(initial_circle)]
                (
                    x,
                    y,
                    yaw,
                ) = self._generate_random_cartesian_cordinatines_inside_circle(
                    initial_circle_centre, initial_circle_radius
                )

                # print("check type", pose, type(pose), type([x,y]))
                if self._min_distance_checker(
                    pose, [x, y], start_range[0]
                ) and self._max_distance_checker(pose, [x, y], start_range[1]):
                    if len(new_poses) == 0:
                        break  # while break

                    # Check for other agents new pose
                    satisfy_agent = []
                    for new_pose in new_poses:
                        if self._min_distance_checker(
                            [x, y], new_pose, agents_range[0]
                        ) and self._max_distance_checker(
                            [x, y], new_pose, agents_range[1]
                        ):
                            satisfy_agent.append(True)
                        else:
                            satisfy_agent.append(False)
                    # print("satisfy agents", satisfy_agent, new_poses, x,y)
                    if all(satisfy_agent):
                        break
                    else:
                        continue

            new_poses.append([x, y])
            new_yaw.append([yaw])

        print("new goal poses", new_poses)
        return new_poses, new_yaw

    def _generate_random_closer_coordinates(
        self, min_dist, max_dist, num_circles, diameter_list, centre_list
    ):
        poses = [0] * (self.total_orca_hum + 2)
        # print("length")
        yaw = [0] * (self.total_orca_hum + 2)

        # select a random point inside a random circle
        initial_circle = random.randrange(0.0, num_circles, 1.0)
        initial_circle_radius = diameter_list[(initial_circle)] / 2
        initial_circle_centre = centre_list[(initial_circle)]
        x, y, yaw_current = self._generate_random_cartesian_cordinatines_inside_circle(
            initial_circle_centre, initial_circle_radius
        )
        poses[0] = [x, y]
        yaw[0] = [yaw_current]

        # Logic to 1. randomly 2.select next point >= min_dist and <= max_dist
        for i in range(self.total_orca_hum + 1):  # except robot
            next_circle = random.randrange(0.0, num_circles, 1.0)
            next_circle_radius = diameter_list[(next_circle)] / 2
            next_circle_centre = centre_list[(next_circle)]
            (
                x,
                y,
                yaw_current,
            ) = self._generate_random_cartesian_cordinatines_inside_circle(
                next_circle_centre, next_circle_radius
            )

            generated_pose = [0.0, 0.0] * (1)  # change to number later
            generated_pose[(int(0))] = [x, y]
            # generated_pose[1] = [ 0.0, 0.0]
            current_index = int(i + 1)
            yaw[int(current_index)] = [yaw_current]

            # check distance from ramining circles
            while not (
                True
                == self._check_distance_from_remaning_coordinates(
                    poses, current_index, generated_pose, min_dist, max_dist
                )
            ):
                # repeat point generation logic
                next_circle = random.randrange(
                    0.0, num_circles, 1.0
                )  # TODO: write this repeated logic as function
                next_circle_radius = diameter_list[(next_circle)] / 2
                next_circle_centre = centre_list[(next_circle)]
                (
                    x,
                    y,
                    yaw_current,
                ) = self._generate_random_cartesian_cordinatines_inside_circle(
                    next_circle_centre, next_circle_radius
                )
                generated_pose[int(0)] = [x, y]
                ("print", x, y)

            # print("generate list", generated_pose[0][0], generated_pose[0][1])
            poses[current_index] = [x, y]
        #     ("outside print", x, y)

        # print("All generated poses", poses, yaw)

        return poses, yaw

    def _select_closer_points(
        self, restricted_circle, num_circles, diameter_list, centre_list
    ):
        poses = [0.0, 0.0] * (self.total_orca_hum + 2)
        yaw = [0] * (self.total_orca_hum + 2)

        # #internal variable
        # selected_circles = [0] * (self.total_orca_hum+2)

        initial_circle = random.randrange(0.0, num_circles, 1.0)
        # print("selected first circle and restricted one", initial_circle, restricted_circle)

        while initial_circle == restricted_circle:
            initial_circle = random.randrange(0.0, num_circles, 1.0)
            # print("In loop to select unrestricted circle")

        # generate first point
        selected_circle_radius = diameter_list[(initial_circle)] / 2
        selected_circle_centre = centre_list[(initial_circle)]
        x, y, yaw_current = self._generate_random_cartesian_cordinatines_inside_circle(
            selected_circle_centre, selected_circle_radius
        )
        poses[0] = [x, y]
        yaw[0] = [yaw_current]

        # select list of circles in from generated point
        selected_goal_circles = []

        for i in range(num_circles):
            # find norm is less than 5 metres
            # if so append the list
            if True == self._max_distance_checker(
                poses[0], centre_list[i], 5
            ):  # TODO: make it as macro, Less than 5 meters for check
                selected_goal_circles.append(i)
                # print("selected list", selected_goal_circles)

        # randomly select circles in this list
        for i in range(self.total_orca_hum + 2 - 1):
            random_circle = random.choice(selected_goal_circles)

            # generate point
            selected_circle_radius = diameter_list[(random_circle)] / 2
            selected_circle_centre = centre_list[(random_circle)]
            (
                x,
                y,
                yaw_current,
            ) = self._generate_random_cartesian_cordinatines_inside_circle(
                selected_circle_centre, selected_circle_radius
            )
            poses[(int(1 + i))] = [x, y]
            yaw[(int(1 + i))] = [yaw_current]

        # #dirty logic change this place
        # self.goals
        # print("goal poses and yaw", poses, yaw) #SEE HERE: START FROM here
        # print("old poses and yaw", poses, yaw)
        return poses, yaw

    def _reset_agents_pose_in_simulator(self):
        num_circles, diameter_list, centre_list = self._extract_circles_info_in_map()

        poses, yaw = self._generate_random_closer_coordinates(
            1.25, 5, num_circles, diameter_list, centre_list
        )  # keep it as 0.7 meters as agents human diamenter is 0.65m and robot is 0.4m

        self._reset_agent_start_poses(poses, yaw)

        goals = []
        yaws = []
        for i in range(self.NUM_OF_GOALS):
            goal, yaw = self._generate_random_far_coordinates(
                start_poses=poses,
                start_range=(1, 5),
                agents_range=(1, 5),
                num_circles=num_circles,
                diameter_list=diameter_list,
                centre_list=centre_list,
            )
            goals.append(goal)
            yaws.append(yaw)
            goals = np.array(goals).T
            yaws = np.array(yaws).T

            # current goal is next start
            poses = goal

        self._reset_agent_goal_poses(goals.tolist(), yaws.tolist())

    def _generate_random_cartesian_cordinatines_inside_circle(self, centre, radius):
        r = radius * math.sqrt(random.random())
        theta = random.random() * 2 * math.pi

        x = centre[0] + r * math.cos(theta)
        y = centre[1] + r * math.sin(theta)

        yaw = random.uniform(math.pi, -math.pi)
        return x, y, yaw

    def gui_create(self, simulator):
        self.gui.title("Datalogger")
        self.gui.geometry("250x150")
        reset_button = tk.Button(self.gui, text="Reset", command=self.reset)
        stop_button = tk.Button(self.gui, text="Stop")
        save_button = tk.Button(self.gui, text="Save")
        discard_button = tk.Button(self.gui, text="Discard")
        reset_button.pack(fill="both", expand=0.5)
        stop_button.pack(fill="both", expand=0.5)
        save_button.pack(fill="both", expand=0.5)
        discard_button.pack(fill="both", expand=0.5)

    def gui_update(self):
        self.gui.update_idletasks()
        self.gui.update()

    def reset(self):
        self.reset_status = True
        self.current_num_orca_hum = random.randrange(
            1.0, 5.0, 1.0
        )  # TODO: put these numbers as macros or as variables at top to configure

        # TODO: Do a sanitary run before or after setting poses
        # TODO: write logic for random reset position
        self._reset_agents_pose_in_simulator()


def main():
    rospy.init_node("randomisation_node")

    pkg_path = RosPack().get_path("morse_ros")
    simulator = SimulationHandler(
        config=os.path.join(pkg_path, "./configs", "laas_adream_init.yaml"),
        rate=RATE_HZ,
        debug=False,
    )

    scenario = ScenarioCreator(simulator)
    scenario.gui_create(simulator)

    # try:
    while True:
        scenario.gui_update()
        simulator.run(scenario)
    # finally:
    #     pass


if __name__ == "__main__":
    main()
