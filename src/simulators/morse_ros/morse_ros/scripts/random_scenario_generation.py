import rospy
from local_collision import SimulationHandler
from rospkg import RosPack
import os
import pymorse
import random
import sys
import tkinter as tk
import math
import yaml
import numpy as np
from geometry_msgs.msg import PoseStamped
from inhus.msg import Goal   #check whether it got imported
from transformations import quaternion_from_euler

RATE_HZ = 20.0 #TODO:put this in a proper place, eg:- Inside simulator class

#Note, make this change in code - Simultor refers to Morse, ORCA or rvo2 simulator has to be mentioned explicitly
class ScenarioCreator(SimulationHandler): #This is rvo2 simulator

    gui = tk.Tk()
    reset_status = False    #CHANGE: Had to put these variables outside to share it with other class in another file
    total_orca_hum = None   #Added for conveinece and read at startup, did instead of passing simulator at unwanted places
    current_num_orca_hum = None

    def __init__(self, SimulationHandler):
        sim_instance = None
        self.sim_human_id = ['Human.001', 'Human.002', 'Human.003', 'Human.004'] #TODO:can read it from simulator itself
        self.total_orca_hum = SimulationHandler.num_hum
        self.goals = [[0.0, 0.0]] * self.total_orca_hum   #TODO: this is fine
        self.start_poses = [0.0, 0.0] * (self.total_orca_hum+2)

        #Robot and Inhus publishers
        self.robot_goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.inhus_goal_pub = rospy.Publisher("/boss/human/new_goal", Goal, queue_size=10)

        for i in range(self.total_orca_hum):     # generate goals from circles
            self.goals[i] = [(1.0 + i), 17.0]
            # print("remove this soon human",i, self.goals[i]) #TODO:remove this

        # print("rvo2 sim no", self.total_orca_hum)
        self._simulator_instance()

    def _simulator_instance(self):
        self.sim_instance = pymorse.Morse()

    def _simulator_reset_robot_pose(self, x, y, yaw):
        self.sim_instance.rpc('simulation', 'set_object_position','pr2', [x,y,0], [0.0, 0.0, yaw])

    def _simulator_reset_inhus_pose(self, x, y, yaw):
        self.sim_instance.rpc('simulation', 'set_object_position','Human', [x,y,0], [0.0, 0.0, yaw])

    def _simulator_reset_orca_humans_pose(self, poses, yaw): #TODO:write as for loop and string match

        print("poses", poses[(2+1)][0], yaw[(2+1)][0])
        #ORCA human
        for i in range(self.current_num_orca_hum):
            print("i", i)
            self.sim_instance.rpc('simulation', 'set_object_position',self.sim_human_id[i], [(poses[(int(2+i))][0]),(poses[(int(2+i))][1]),0], [0.0, 0.0, yaw[(int(2+i))][0]])

        #logic to put remaining orca human out of the map
        if(self.current_num_orca_hum != self.total_orca_hum):
            diff = self.total_orca_hum - self.current_num_orca_hum
            for i in range(diff):
                # print("i, curr, total", i, self.current_num_orca_hum, self.total_orca_hum)
                yaw = random.uniform(math.pi, -math.pi)
                self.sim_instance.rpc('simulation', 'set_object_position',self.sim_human_id[(self.current_num_orca_hum+i)], [200.0, (200.0+i),0], [0.0, 0.0, yaw])

    def _set_robot_goal_pose(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.seq = 0
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time(0)

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0

        q = quaternion_from_euler(0.0, 0.0, yaw)

        print("orn quat", q)

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
        print("except first 2 poses", poses[2:])
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
        unique_random_list = random.sample(range(0,(max_num)), (list_no)) 
        return unique_random_list

    def _generate_poses_from_unique_circles(self, unique_circles_list, diameter_list, centre_list):
        poses = [0.0, 0.0] * len(unique_circles_list)  #CHANGE: (self.total_orca_hum+2) instead of unique_circles_list
        yaw = [0] * len(unique_circles_list)

        for i in range(len(unique_circles_list)):
            selected_circle_radius = diameter_list[(unique_circles_list[i])]/2
            selected_circle_centre = centre_list[(unique_circles_list[i])]
            x, y, yaw_current = self._generate_random_cartesian_cordinatines_inside_circle(selected_circle_centre, selected_circle_radius)
            poses[i] = [x, y]
            yaw[i] = [yaw_current]

        print("poses and yaw", poses, yaw)
        return poses, yaw

    def _reset_agent_start_poses(self, poses, yaw):

        #TODO:make all of their velocities too zero before resetting their start positions
        self._simulator_reset_robot_pose(poses[0][0], poses[0][1], yaw[0][0])
        self._simulator_reset_inhus_pose(poses[1][0], poses[1][1], yaw[1][0])
        self._simulator_reset_orca_humans_pose(poses, yaw)

    def _reset_agent_goal_poses(self, poses, yaw):

        self._set_robot_goal_pose(poses[0][0], poses[0][1], yaw[0][0])
        self._set_inhus_goal_pose(poses[1][0], poses[1][1], yaw[1][0])
        self._set_orca_humans_goal_pose(poses, yaw)

    @staticmethod
    def _distance_checker(init_pose, next_pose, threshold):
        # print("check init pose datatype", init_pose)
        # print("check init pose datatype ii", init_pose[0], init_pose[1])
        # print("check next pose datatype", next_pose)
        # print("check next pose datatype ii", next_pose[0], next_pose[1])
        if np.linalg.norm([
            next_pose[0] - init_pose[0],
            next_pose[1] - init_pose[1]
        ]) < threshold:
            return True
        else:
            return False

    def _select_closer_points(self, restricted_circle, num_circles, diameter_list, centre_list):
        poses = [0.0, 0.0] * (self.total_orca_hum+2)
        yaw = [0] * (self.total_orca_hum+2)

        # #internal variable
        # selected_circles = [0] * (self.total_orca_hum+2)

        initial_circle = random.randrange(0.0, num_circles, 1.0)
        print("selected first circle and restricted one", initial_circle, restricted_circle)

        while (initial_circle == restricted_circle):
            initial_circle = random.randrange(0.0, num_circles, 1.0)
            print("In loop to select unrestricted circle")

        #generate first point
        selected_circle_radius = diameter_list[(initial_circle)]/2
        selected_circle_centre = centre_list[(initial_circle)]
        x, y, yaw_current = self._generate_random_cartesian_cordinatines_inside_circle(selected_circle_centre, selected_circle_radius)
        poses[0] = [x, y]
        yaw[0] = [yaw_current]

        #select list of circles in from generated point
        selected_goal_circles = []

        for i in range(num_circles):
            #find norm is less than 5 metres
            #if so append the list
            if(True == self._distance_checker(poses[0], centre_list[i], 5)):   #TODO: make it as macro, Less than 5 meters for check
                selected_goal_circles.append(i)
                print("selected list", selected_goal_circles)

        #randomly select circles in this list
        for i in range(self.total_orca_hum+2 - 1): 
            random_circle = random.choice(selected_goal_circles)

            #generate point
            selected_circle_radius = diameter_list[(random_circle)]/2
            selected_circle_centre = centre_list[(random_circle)]
            x, y, yaw_current = self._generate_random_cartesian_cordinatines_inside_circle(selected_circle_centre, selected_circle_radius)
            poses[(int(1+i))] = [x, y]
            yaw[(int(1+i))] = [yaw_current]

        # #dirty logic change this place
        # self.goals
        print("goal poses and yaw", poses, yaw) #SEE HERE: START FROM here
        return poses, yaw

    def _reset_agents_pose_in_simulator(self):
        num_circles, diameter_list, centre_list = self._extract_circles_info_in_map()

        #logic to select unique circles
        unique_circles_list = self._find_random_unique_list(num_circles, (self.total_orca_hum+2)) # 2 added for Robot + Inhus
        poses, yaw = self._generate_poses_from_unique_circles(unique_circles_list, diameter_list, centre_list)

        self._reset_agent_start_poses(poses, yaw)

        #logic to select a point and select other points near the first point
        poses, yaw = self._select_closer_points(unique_circles_list[0], num_circles, diameter_list, centre_list)

        self._reset_agent_goal_poses(poses, yaw)

        #select robot circle
        # selected_circle = random.randrange(0.0, (num_circles), 1.0)  #starting from zero for indexing
        # selected_circle_radius = diameter_list[(selected_circle)]/2
        # selected_circle_centre = centre_list[(selected_circle)]
        # print("selected circle num, radius, centre", selected_circle, selected_circle_radius, selected_circle_centre)
        # print("centre", selected_circle_centre[0], selected_circle_centre[1])
        # x, y, yaw = self._generate_random_cartesian_cordinatines_inside_circle(selected_circle_centre, selected_circle_radius)
        # self._simulator_reset_robot_pose(x, y, yaw)
        # self._simulator_reset_inhus_pose(self, x, y, yaw)
        # self._simulator_reset_orca_humans_pose()


    def _generate_random_cartesian_cordinatines_inside_circle(self, centre, radius):
        # print("random no", random.random())
        r = radius * math.sqrt(random.random())
        theta = random.random() * 2 * math.pi

        x = centre[0] + r * math.cos(theta)
        y = centre[1] + r * math.sin(theta)

        yaw = random.uniform(math.pi, -math.pi)
        return x, y, yaw

    def gui_create(self, simulator):
        self.gui.title("Datalogger")
        self.gui.geometry("250x150")
        reset_button = tk.Button(self.gui, text = "Reset", command = self.reset)
        stop_button = tk.Button(self.gui, text = "Stop")
        save_button = tk.Button(self.gui, text = "Save")
        discard_button = tk.Button(self.gui, text = "Discard")
        reset_button.pack(fill= "both",expand=0.5)
        stop_button.pack(fill= "both",expand=0.5)
        save_button.pack(fill= "both",expand=0.5)
        discard_button.pack(fill= "both",expand=0.5)

    def gui_update(self):
        self.gui.update_idletasks()
        self.gui.update()

    def reset(self):
        self.reset_status = True
        self.current_num_orca_hum = random.randrange(1.0, 5.0, 1.0) #TODO: put these numbers as macros or as variables at top to configure


        #TODO: Do a sanitary run before or after setting poses
        #TODO: write logic for random reset position
        self._reset_agents_pose_in_simulator()
        print("reset called")

def main():
    rospy.init_node("randomisation_node")

    pkg_path = RosPack().get_path("morse_ros")
    simulator = SimulationHandler(config=os.path.join(pkg_path, "./configs", "laas_adream_init.yaml"), rate=RATE_HZ, debug = False)

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