#!/usr/bin/env python

# from tracemalloc import start
import rospy
from nav_msgs.srv import GetPlan, GetPlanRequest
from navfn.srv import MakeNavPlan, MakeNavPlanRequest
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import rvo2
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import math
import copy
import tf2_ros
import numpy as np
import random

#TODO: code restructuring
#1) Have an id for each humans and don't create a new class
#2) Map the id with the index of plan (not new_plan_1 , 2 etc), have an array of plan
#3) Many publishers is fine but string match and chage the name of publishers etc, don't rewrite stuff
#4)  


vel_msg = Twist()
orca_human2_pub = rospy.Publisher('/morse_agents/human2/cmd_vel', Twist, queue_size=10, latch=False)
orca_human3_pub = rospy.Publisher('/morse_agents/human3/cmd_vel', Twist, queue_size=10, latch=False)

orca_human2_plan_pub = rospy.Publisher('/human_2_global_plan', Path, queue_size=600)
orca_human3_plan_pub = rospy.Publisher('/human_3_global_plan', Path, queue_size=600)

def path_message_conversion(new_plan):
    gplan = Path()
    q = PoseStamped()
    gplan.poses = new_plan.path
    gplan.header.frame_id = "map"


    return gplan

class orca_instance:
    def __init__(self, goal):
        self.current_pose = PoseStamped()
        self.current_twist = Twist()
        self.current_yaw = 0  

        self.goal_pose = PoseStamped()
        self.goal_pose.pose.position.x = goal[0]
        self.goal_pose.pose.position.y = goal[1]

        self.next_step = Twist()

class data_instance:
  def __init__(self):      


    #ORCA agent numbers
    self.human3_orca = None
    self.human2_orca = None
    self.human1_orca = None
    self.robot_orca = None

    # Current poses human and robot
    self.current_robot_pose_x = 0
    self.current_robot_pose_y = 0

    self.current_human1_pose_x = 0
    self.current_human1_pose_y = 0

def update_human1_pose(data, args):
    args.current_human1_pose_x = data.pose.pose.position.x
    args.current_human1_pose_y = data.pose.pose.position.y

def update_orca_human_pose_callback(data, args):
    args.current_pose.pose = data.pose.pose
    args.current_twist = data.twist.twist

    orientation_list = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

    args.current_yaw = yaw

def update_human2_pose(data, args):
    args.start_human2_pose_x = data.pose.pose.position.x
    args.start_human2_pose_y = data.pose.pose.position.y

    args.human2_twist = data.twist.twist

    args.start_human2_quat_orientation_x = data.pose.pose.orientation.x
    args.start_human2_quat_orientation_y = data.pose.pose.orientation.y
    args.start_human2_quat_orientation_z = data.pose.pose.orientation.z
    args.start_human2_quat_orientation_w = data.pose.pose.orientation.w

    orientation_list = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

    args.start_human2_yaw = yaw

def update_robot_pose(data, args):
    args.current_robot_pose_x = data.pose.pose.position.x
    args.current_robot_pose_y = data.pose.pose.position.y

def clamp(x, minn, maxx):
   return x if x > minn and x < maxx else (minn if x < minn else maxx)

def get_plan(orca_agent):

    start = PoseStamped()
    start.header.seq = 0
    start.header.frame_id = "map"
    start.header.stamp = rospy.Time(0)
    start.pose.position.x = orca_agent.current_pose.pose.position.x #  database.start_human2_pose_x   
    start.pose.position.y = orca_agent.current_pose.pose.position.y #database.start_human2_pose_y

    Goal = PoseStamped()
    Goal.header.seq = 0
    Goal.header.frame_id = "map"
    Goal.header.stamp = rospy.Time(0)
    Goal.pose.position.x = orca_agent.goal_pose.pose.position.x #database.goal_human2_pose_x 
    Goal.pose.position.y = orca_agent.goal_pose.pose.position.y #database.goal_human2_pose_y

    get_plan_service = rospy.ServiceProxy('gbl_planner/make_plan', MakeNavPlan)
    r = MakeNavPlan()
    r.start = start
    r.goal = Goal 
    plan = get_plan_service(r.start, r.goal)

    return plan

def normalize_theta(theta):
  PI = math.pi
  result = math.fmod(theta + PI, 2.0 * PI)
  if result <= 0:
    return result + PI
  return result - PI

def velocity(plan, orca_agent, i, tfBuffer):

    x_vel_world = (plan.path[i].pose.position.x - orca_agent.current_pose.pose.position.x) / (0.05)  #can be i-1
    y_vel_world = (plan.path[i].pose.position.y - orca_agent.current_pose.pose.position.y) / (0.05) #(plan.plan.poses[1].pose.position.y - plan.plan.poses[0].pose.position.y) / (0.05)

    orientation_list = [plan.path[i].pose.orientation.x, plan.path[i].pose.orientation.y, plan.path[i].pose.orientation.z, plan.path[i].pose.orientation.w]
    (roll, pitch, yaw_orig) = euler_from_quaternion (orientation_list)

    omega = normalize_theta(yaw_orig - orca_agent.current_yaw)/0.05
    
    return x_vel_world, y_vel_world, yaw_orig, omega

#extension tip 3, have one function and string match publishers inside   
def publish_human2_vel(orca_agent):
    orca_human2_pub.publish(orca_agent.next_step)

def publish_human3_vel(orca_agent):
    orca_human3_pub.publish(orca_agent.next_step)

def add_orca_agents(sim, general_database, orca_agent_1, orca_agent_2):

#4) Put for loop and add agents, don't rewrite each line and don't pass each class
    general_database.human2_orca = sim.addAgent((orca_agent_1.current_pose.pose.position.x, orca_agent_1.current_pose.pose.position.y))
    general_database.human3_orca = sim.addAgent((orca_agent_2.current_pose.pose.position.x, orca_agent_2.current_pose.pose.position.y))


    o1 = sim.addObstacle([(8.0, -1.78), (9.0, -1.78), (9.0, -1.18), (8.0, -1.18)])

    k1 = sim.processObstacles()

def plan(orca_agent, i):
#5) don't pass number i, the orca_agent class id should help in doing this 
    if(i == 1):
        new_plan = get_plan(orca_agent)
        gplan = path_message_conversion(new_plan)
        orca_human2_plan_pub.publish(gplan) #(new_plan.path)

    if(i == 2):
        new_plan = get_plan(orca_agent)
        gplan = path_message_conversion(new_plan)
        orca_human3_plan_pub.publish(gplan) #(new_plan.path)

    return new_plan

def move_logic(sim, general_database, new_plan, tfBuffer, i, orca_human1, orca_human2):
#6) put for loop and don't rewrite things

    if(i == 1):

        if np.linalg.norm([orca_human1.goal_pose.pose.position.x - orca_human1.current_pose.pose.position.x, orca_human1.goal_pose.pose.position.y -  orca_human1.current_pose.pose.position.y]) < 0.2 or len(new_plan.path) <2:
            orca_human1.next_step.linear.x = 0
            orca_human1.next_step.linear.y = 0
            orca_human1.next_step.angular.z = 0

        else:

            data_back = copy.deepcopy(orca_human1)
            x_vel, y_vel, yaw, omega = velocity(new_plan, data_back, 1, tfBuffer)

            sim.setAgentPrefVelocity(general_database.human2_orca, (x_vel, y_vel))  #check this sequence
            sim.doStep()
            sim.setAgentPosition(general_database.human3_orca, (orca_human2.current_pose.pose.position.x, orca_human2.current_pose.pose.position.y))
            sim.setAgentPosition(general_database.human2_orca, (orca_human1.current_pose.pose.position.x, orca_human1.current_pose.pose.position.y))
            orcavel_x, orcavel_y = sim.getAgentVelocity(general_database.human2_orca)

            vel = np.linalg.norm([orcavel_x, orcavel_y])

            new_x_vel = orcavel_x*math.cos(yaw)+ orcavel_y*math.sin(yaw) #vel*math.cos(yaw) 
            new_y_vel = -orcavel_x*math.sin(yaw)+ orcavel_y*math.cos(yaw) #vel*math.sin(yaw)
    
            orca_human1.next_step.linear.x = new_x_vel
            orca_human1.next_step.linear.y = new_y_vel
            orca_human1.next_step.linear.z = 0

            orca_human1.next_step.angular.x = 0
            orca_human1.next_step.angular.y = 0
            orca_human1.next_step.angular.z = omega

    if(i == 2):

        if np.linalg.norm([orca_human2.goal_pose.pose.position.x - orca_human2.current_pose.pose.position.x, orca_human2.goal_pose.pose.position.y -  orca_human2.current_pose.pose.position.y]) < 0.2 or len(new_plan.path) <2:
            orca_human2.next_step.linear.x = 0
            orca_human2.next_step.linear.y = 0
            orca_human2.next_step.angular.z = 0

        else:

            data_back = copy.deepcopy(orca_human2)
            x_vel, y_vel, yaw, omega = velocity(new_plan, data_back, 1, tfBuffer)
                    

            sim.setAgentPrefVelocity(general_database.human3_orca, (x_vel, y_vel))  #check this sequence
            sim.doStep()
            sim.setAgentPosition(general_database.human2_orca, (orca_human1.current_pose.pose.position.x, orca_human1.current_pose.pose.position.y))
            sim.setAgentPosition(general_database.human3_orca, (orca_human2.current_pose.pose.position.x, orca_human2.current_pose.pose.position.y))
            orcavel_x, orcavel_y = sim.getAgentVelocity(general_database.human3_orca)
	    #7) check whether this get velocity should be done should be done right after set or Can be done by setting and getting velocity in a loop --> check this sequence 

            vel = np.linalg.norm([orcavel_x, orcavel_y])

            new_x_vel = orcavel_x*math.cos(yaw)+ orcavel_y*math.sin(yaw) #vel*math.cos(yaw) 
            new_y_vel = -orcavel_x*math.sin(yaw)+ orcavel_y*math.cos(yaw) #vel*math.sin(yaw)


            orca_human2.next_step.linear.x = new_x_vel
            orca_human2.next_step.linear.y = new_y_vel
            orca_human2.next_step.linear.z = 0

            orca_human2.next_step.angular.x = 0
            orca_human2.next_step.angular.y = 0
            orca_human2.next_step.angular.z = omega


def main():
    global k
    rospy.init_node('get_plan_client')
    general_database = data_instance()

    orca_human1 = orca_instance([8.41, 2.31])    

    orca_human2 = orca_instance([8.45, -4.85])

    rospy.Subscriber('/morse_agents/human1/odom', Odometry, update_human1_pose, general_database)

#extension tip 3, have one function and string match subscribers 
    rospy.Subscriber('/morse_agents/human2/odom', Odometry, update_orca_human_pose_callback, orca_human1)
    rospy.Subscriber('/morse_agents/human3/odom', Odometry, update_orca_human_pose_callback, orca_human2)    
    rospy.Subscriber('/odom', Odometry, update_robot_pose, general_database)
    rospy.wait_for_service('gbl_planner/make_plan')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    

    sim = rvo2.PyRVOSimulator(1/60.0, 1.5, 3, 2.5, 4.5, 0.4, new_plan_hum_2human2) 


    new_plan_hum_1 = plan(orca_human1, 1)
    new_plan_hum_2 = plan(orca_human2, 2)

    rate = rospy.Rate(1) # 24hz maximum (41 ms)
    control_rate = rospy.Rate(20)
    while not rospy.is_shutdown(): # try normal while loop also for making it faster

        new_plan_hum_1 = plan(orca_human1, 1)
        new_plan_hum_2 = plan(orca_human2, 2)

        move_logic(sim, general_database, new_plan_hum_1, tfBuffer, 1, orca_human1, orca_human2)
        move_logic(sim, general_database, new_plan_hum_2, tfBuffer, 2, orca_human1, orca_human2)

              
        publish_human2_vel(orca_human1)
        publish_human3_vel(orca_human2)
        control_rate.sleep()
            
            
# extra tip check ROS message filters
if __name__ == "__main__":
    main()
