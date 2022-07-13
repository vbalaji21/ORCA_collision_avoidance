#Launching Morse
1) Activate the virtual env -  source my_project_venv/bin/activate 
2) cd ros_ws/InHus.....
3) source devel/setup.bash
4)roslaunch inhus_navigation inhus_cohan_nav.launch map_name:=....

#Launch InHuS
1) cd ros_ws/InHus.....
2) source devel/setup.bash
2) roslaunch inhus_navigation morse_simulator.launch map_name:=...

#Launching CoHAN
1) cd ros_ws/cohan_planner_multi
2) source devel/setup.bash 
3) roslaunch cohan_navigation morse_pr2_only.launch map_name:=laas_adream
