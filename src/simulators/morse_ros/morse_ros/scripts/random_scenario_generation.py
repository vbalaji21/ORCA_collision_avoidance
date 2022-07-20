import rospy
from local_collision import SimulationHandler
from rospkg import RosPack
import os
import pymorse
import random
import sys
import tkinter as tk
import math

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

        for i in range(self.total_orca_hum):     # generate goals from circles
            self.goals[i] = [(1.0 + i), 17.0]
            print("human",i, self.goals[i])

        print("rvo2 sim no", self.total_orca_hum)
        self._simulator_instance()

    def _simulator_instance(self):
        self.sim_instance = pymorse.Morse()

    def _simulator_reset_robot_pose(self):

        x,y = self._generate_random_cartesian_cordinatines_inside_circle(0, 0 , 1)
        yaw = random.uniform(math.pi, -math.pi)
        self.sim_instance.rpc('simulation', 'set_object_position','pr2', [9.0,2.0,0], [0.0, 0.0, yaw])

    def _simulator_reset_humans_pose(self): #TODO:write as for loop and string match
        #Inhus human
        yaw = random.uniform(math.pi, -math.pi)
        self.sim_instance.rpc('simulation', 'set_object_position','Human', [10.0,0.0,0], [0.0, 0.0, yaw])

        #ORCA human
        for i in range(self.current_num_orca_hum):
            yaw = random.uniform(math.pi, -math.pi)
            self.sim_instance.rpc('simulation', 'set_object_position',self.sim_human_id[i], [10.0,(-5.0+i),0], [0.0, 0.0, yaw])

        if(self.current_num_orca_hum != self.total_orca_hum):
            diff = self.total_orca_hum - self.current_num_orca_hum
            for i in range(diff):
                print("i, curr, total", i, self.current_num_orca_hum, self.total_orca_hum)
                yaw = random.uniform(math.pi, -math.pi)
                self.sim_instance.rpc('simulation', 'set_object_position',self.sim_human_id[(self.current_num_orca_hum+i)], [200.0, (200.0+i),0], [0.0, 0.0, yaw])


    def _generate_random_cartesian_cordinatines_inside_circle(self, centre_x, centre_y , radius):
        # print("random no", random.random())
        r = radius * math.sqrt(random.random())
        theta = random.random() * 2 * math.pi

        x = centre_x + r * math.cos(theta)
        y = centre_y + r * math.sin(theta)
        # print("random x and y inside circle", x, y)
        return x, y

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
        self._simulator_reset_robot_pose()
        self._simulator_reset_humans_pose()
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
