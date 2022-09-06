# import pymorse
# import random

# print("random no", random.random())
# # sim = pymorse.Morse()

# # sim.rpc('simulation', 'set_object_position','pr2', [12,4,0])

# # with pymorse.Morse() as sim:
# #     print(sim.rpc('simulation', 'list_robots') )
# #     print()

# #     sim.rpc('simulation', 'set_object_position','pr2', [2,4,0])
# #     sim.rpc('simulation', 'set_object_position','Human.001', [200.0,200.0,0])

# #     # sim.rpc('simulation', 'reset_objects')
# #     # print(sim.rpc('simulation', 'get_scene_objects')) #Not useful

# #     print("DETAILS", sim.rpc('simulation', 'details'))
# #     # sim.rpc('simulation', 'deactivate', 'HumanSkeleton')



# #     #sim.rpc('simulation', 'reset_objects')
# #     #sim.rpc('simulation', 'set_camarafp_far_clip', 1000)

import tkinter as tk
import time
import rospy


def rep_func(rate):
    a = 1
    b = a + 1
    print("b", b)
    time.sleep(1)
    rate.sleep()


def main():
    rospy.init_node("new_node")
    gui = tk.Tk()

    gui.title("Datalogger")
    gui.geometry("250x150")
    reset_button = tk.Button(gui, text = "Reset")
    stop_button = tk.Button(gui, text = "Stop")
    save_button = tk.Button(gui, text = "Save")
    discard_button = tk.Button(gui, text = "Discard")
    exit_button = tk.Button(gui, text="Exit", command= gui.destroy)

    reset_button.pack(fill= "both",expand=0.5)
    stop_button.pack(fill= "both",expand=0.5)
    save_button.pack(fill= "both",expand=0.5)
    discard_button.pack(fill= "both",expand=0.5)
    # Button for closing

    exit_button.pack(fill= "both",expand=0.5)

    rate = rospy.Rate(1)

    while True:
        gui.update_idletasks()
        gui.update()
        rep_func(rate)
        # scenario.gui.protocol('WM_DELETE_WINDOW', scenario.doSomething)  # root is your root window


if __name__ == "__main__":
    main()
