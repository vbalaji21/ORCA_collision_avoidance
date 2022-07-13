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



# Obstacles are also supported as vertices
# o1 = sim.addObstacle([(0.1, 0.1), (-0.1, 0.1), (-0.1, -0.1)])
# sim.processObstacles()

vel_msg = Twist()
orca_human2_pub = rospy.Publisher('/morse_agents/human2/cmd_vel', Twist, queue_size=10, latch=False)
orca_human3_pub = rospy.Publisher('/morse_agents/human3/cmd_vel', Twist, queue_size=10, latch=False)

orca_human2_plan_pub = rospy.Publisher('/human_2_global_plan', Path, queue_size=600)
orca_human3_plan_pub = rospy.Publisher('/human_3_global_plan', Path, queue_size=600)

def path_message_conversion(new_plan):
    gplan = Path()
    q = PoseStamped()
    # q.pose = new_plan.path[0].pose
    # msg.poses[len(new_plan.path)] = [0] * len(new_plan.path) 
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
        # self.goal_twist = Twist()

        self.next_step = Twist()

class data_instance:
  def __init__(self):
    # self.start_human2_pose_x = 0
    # self.start_human2_pose_y = 0

    # self.human2_twist = Twist()
    # self.start_human2_quat_orientation_x = 0            #start orientation  
    # self.start_human2_quat_orientation_y = 0            #start orientation 
    # self.start_human2_quat_orientation_z = 0            #start orientation
    # self.start_human2_quat_orientation_w = 1            #start orientation     


    # # self.start_human2_pose_z = 0    
    # self.goal_human2_pose_x =  goal[0] #8.47 #1.0 #10.1 #7.5 #5.0 #10.8 #5.0 #9.4
    # self.goal_human2_pose_y =  goal[1] #10.1 #11.0 #11.1 #1.5 #2.5 #9.17 # 2.5 #6.4
    # self.goal_human2_quat_orientationPoseS_x = 0            #goal orientation  
    # self.goal_human2_quat_orientation_y = 0            #goal orientation 
    # self.goal_human2_quat_orientation_z = 0            #goal orientation
    # self.goal_human2_quat_orientation_w = 1            #goal orientation         


    #ORCA agent numbers
    self.human3_orca = None
    self.human2_orca = None
    self.human1_orca = None
    self.robot_orca = None

    # self.next_human2_state = Twist()
    # self.plan = 0

    # Current poses human and robot
    self.current_robot_pose_x = 0
    self.current_robot_pose_y = 0
    # self.current_robot_pose_z = 0
    self.current_human1_pose_x = 0
    self.current_human1_pose_y = 0
    # self.current_human1_pose_z = 0

def update_human1_pose(data, args):
    args.current_human1_pose_x = data.pose.pose.position.x
    args.current_human1_pose_y = data.pose.pose.position.y
    # args.current_human1_pose_z = data.pose.pose.position.z

def update_orca_human_pose_callback(data, args):
    args.current_pose.pose = data.pose.pose
    args.current_twist = data.twist.twist

    # print("data", data.pose.pose)
    orientation_list = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

    args.current_yaw = yaw

    # args.start_human2_pose_z = data.pose.pose.position.z

def update_human2_pose(data, args):
    args.start_human2_pose_x = data.pose.pose.position.x
    args.start_human2_pose_y = data.pose.pose.position.y
    # args.start_human2_pose_z = data.pose.pose.position.z
    args.human2_twist = data.twist.twist

    args.start_human2_quat_orientation_x = data.pose.pose.orientation.x
    args.start_human2_quat_orientation_y = data.pose.pose.orientation.y
    args.start_human2_quat_orientation_z = data.pose.pose.orientation.z
    args.start_human2_quat_orientation_w = data.pose.pose.orientation.w

    orientation_list = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

    args.start_human2_yaw = yaw
    # args.start_human2_pose_z = data.pose.pose.position.z

def update_robot_pose(data, args):
    args.current_robot_pose_x = data.pose.pose.position.x
    args.current_robot_pose_y = data.pose.pose.position.y
    # args.current_robot_pose_z = data.pose.pose.position.z

def clamp(x, minn, maxx):
   return x if x > minn and x < maxx else (minn if x < minn else maxx)

def get_plan(orca_agent):

    start = PoseStamped()
    start.header.seq = 0
    start.header.frame_id = "map"
    start.header.stamp = rospy.Time(0)
    start.pose.position.x = orca_agent.current_pose.pose.position.x #  database.start_human2_pose_x   
    start.pose.position.y = orca_agent.current_pose.pose.position.y #database.start_human2_pose_y
    # start.pose.position.z = database.start_human2_pose_z

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
    # print("plan", plan.plan.poses[0].pose.position.x)
    return plan

def normalize_theta(theta):
  PI = math.pi
  result = math.fmod(theta + PI, 2.0 * PI)
  if result <= 0:
    return result + PI
  return result - PI

def velocity(plan, orca_agent, i, tfBuffer):
    # a_max = 1.5

    x_vel_world = (plan.path[i].pose.position.x - orca_agent.current_pose.pose.position.x) / (0.05)  #can be i-1
    y_vel_world = (plan.path[i].pose.position.y - orca_agent.current_pose.pose.position.y) / (0.05) #(plan.plan.poses[1].pose.position.y - plan.plan.poses[0].pose.position.y) / (0.05)
    # print("x start and goal",  database.start_human2_pose_y, plan.plan.poses[1].pose.position.y)
    # print("y vel",  y_vel)
    orientation_list = [plan.path[i].pose.orientation.x, plan.path[i].pose.orientation.y, plan.path[i].pose.orientation.z, plan.path[i].pose.orientation.w]
    (roll, pitch, yaw_orig) = euler_from_quaternion (orientation_list)

    omega = normalize_theta(yaw_orig - orca_agent.current_yaw)/0.05

    # prev_vel = np.linalg.norm([orca_agent.current_twist.linear.x, orca_agent.current_twist.linear.y])
    # vel = clamp(np.linalg.norm([x_vel_world,y_vel_world]),-1.2, 1.2)

    # a_current = (vel- prev_vel)/0.05
    # a = min(a_max, a_current)
    # vel = prev_vel + a*0.05

    # x_vel = vel*math.cos(yaw_orig) #x_vel_world*math.cos(yaw_orig) - y_vel_world*math.sin(yaw_orig) 
    # y_vel = vel*math.sin(yaw_orig) #x_vel_world*math.sin(yaw_orig) + y_vel_world*math.cos(yaw_orig)
    
    return x_vel_world, y_vel_world, yaw_orig, omega


# def velocity(plan, orca_agent, i, tfBuffer):
#     # a_max = 1.5

#     orcavel_x, orcavel_y
#     # print("x start and goal",  database.start_human2_pose_y, plan.plan.poses[1].pose.position.y)
#     # print("y vel",  y_vel)
#     # orientation_list = [plan.path[i].pose.orientation.x, plan.path[i].pose.orientation.y, plan.path[i].pose.orientation.z, plan.path[i].pose.orientation.w]
#     # (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

#     orca_agent.yaw #= normalize_theta(yaw - orca_agent.current_yaw)

#     # prev_vel = np.linalg.norm([orca_agent.current_twist.linear.x, orca_agent.current_twist.linear.y])
#     # vel = clamp(np.linalg.norm([x_vel_world,y_vel_world])+prev_vel,-1.2, 1.2)

#     # a_current = (vel- prev_vel)/0.05
#     # a = min(a_max, a_current)
#     # vel = prev_vel + a*0.05

#     vel = np.linalg.norm([orcavel_x, orcavel_y])

#     x_vel = vel*math.cos(yaw) 
#     y_vel = vel*math.sin(yaw)
    
#     return x_vel, y_vel, yaw
    
def publish_human2_vel(orca_agent):
    orca_human2_pub.publish(orca_agent.next_step)

def publish_human3_vel(orca_agent):
    orca_human3_pub.publish(orca_agent.next_step)

def add_orca_agents(sim, general_database, orca_agent_1, orca_agent_2):
    # Pass either just the position (the other parameters then use the default values passed to the PyRVOSimulator constructor), or pass all available parameters.
    # print("orca added",database.start_human2_pose_x, database.start_human2_pose_y)
    general_database.human2_orca = sim.addAgent((orca_agent_1.current_pose.pose.position.x, orca_agent_1.current_pose.pose.position.y))
    general_database.human3_orca = sim.addAgent((orca_agent_2.current_pose.pose.position.x, orca_agent_2.current_pose.pose.position.y))
    # database.human1_orca = sim.addAgent((database.current_human1_pose_x, database.current_human1_pose_y))  #read from odom topic
    # database.robot_orca  = sim.addAgent((database.current_robot_pose_x, database.current_robot_pose_y))  #read from odom topic

    # o1 = sim.addObstacle([(5.66, 7.35), (5.68, 6.17), (6.77, 6.17), (5.59, 6.95), (5.68, 6.35)])
    # o1 = sim.addObstacle([(6.36, 7.57), (6.36, 5.25), (7.76, 5.25), (7.76,7.57)])
    # o1 = sim.addObstacle([(0.604, 0.0022), (0.604, -0.345), (1.72, -0.345)]) #(1.72, 0.0022),
    # o1 = sim.addObstacle([(-0.0437, 0.262), (-0.0144, -0.178), (1.75, -0.266), (1.81, 0.204)]) #(1.72, 0.0022),
    # o1 = sim.addObstacle([(0.162, 1.67), (0.162, 2.87), (2.1, 2.87), (2.1, 1.67)])      #([(0.103, 0.0571), (0.103, 1.44), (2.25, 1.44), (2.25, 0.0571)])    
    o1 = sim.addObstacle([(8.0, -1.78), (9.0, -1.78), (9.0, -1.18), (8.0, -1.18)])

    # print("obstacle no", o1)
    k1 = sim.processObstacles()
    # # rospy.sleep(10.0)
    # print("process obstacle", k1)
# sim.processObstacles()

#    a3 = sim.addAtimer_callbackefVelocity(human2_orca, (x_vel, y_vel))
    # sim.setAgentPrefVelocity(a1, (-1, 1))
    # sim.setAgentPrefVelocity(a2, (-1, -1))
    # sim.setAgentPrefVelocity(a3, (1, -1))

def plan(orca_agent, i):

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
    global l
    # l = l + 0.0001
    # random.seed(1.0)
    # print(random.random()) 

    # q = random.randrange(1.0, 10.0, 10.0) 
    # print("q", q)
    # angle = (q) *0.1  # std::rand() * 2.0f * M_PI / RAND_MAX;
    # dist = (q) * 0.0001 #  std::rand() * 0.0001f / RAND_MAX;



    if(i == 1):

        if np.linalg.norm([orca_human1.goal_pose.pose.position.x - orca_human1.current_pose.pose.position.x, orca_human1.goal_pose.pose.position.y -  orca_human1.current_pose.pose.position.y]) < 0.2 or len(new_plan.path) <2:
            orca_human1.next_step.linear.x = 0
            orca_human1.next_step.linear.y = 0
            orca_human1.next_step.angular.z = 0

        else:
            # sim.setAgentPosition(database.robot_orca, (database.current_robot_pose_x, database.current_robot_pose_y))
            # sim.setAgentPosition(general_database.human3_orca, (orca_human2.current_pose.pose.position.x, orca_human2.current_pose.pose.position.y))
            # sim.setAgentPosition(general_database.human2_orca, (orca_human1.current_pose.pose.position.x, orca_human1.current_pose.pose.position.y))
            # sim.setAgentPosition(database.human1_orca, (database.current_human1_pose_x, database.current_human1_pose_y))

            # rospy.sleep(0.02)                             # sleep of 100 Hz needed
            data_back = copy.deepcopy(orca_human1)
            x_vel, y_vel, yaw, omega = velocity(new_plan, data_back, 1, tfBuffer)
            # print("human 1 calculated velocity", x_vel, y_vel, i)

            sim.setAgentPrefVelocity(general_database.human2_orca, (x_vel, y_vel))  #check this sequence
            sim.doStep()
            sim.setAgentPosition(general_database.human3_orca, (orca_human2.current_pose.pose.position.x, orca_human2.current_pose.pose.position.y))
            sim.setAgentPosition(general_database.human2_orca, (orca_human1.current_pose.pose.position.x, orca_human1.current_pose.pose.position.y))
            orcavel_x, orcavel_y = sim.getAgentVelocity(general_database.human2_orca)
            # print("human 1 orca calculated velocity", orcavel_x, orcavel_y, i)

    # yaw = normalize_theta(yaw - orca_agent1.current_yaw)

    # prev_vel = np.linalg.norm([orca_agent.current_twist.linear.x, orca_agent.current_twist.linear.y])
    # vel = clamp(np.linalg.norm([x_vel_world,y_vel_world])+prev_vel,-1.2, 1.2)

    # # a_current = (vel- prev_vel)/0.05
    # # a = min(a_max, a_current)
    # # vel = prev_vel + a*0.05

    # x_vel = vel*math.cos(yaw) 
    # y_vel = vel*math.sin(yaw)

            # x_vel = orcavel_x + (dist * math.cos(angle))   
            # y_vel = orcavel_y + (dist * math.sin(angle))  
            # print("human 2 orca calculated velocity", orcavel_x, orcavel_y, i)

            # sim.setAgentPrefVelocity(general_database.human3_orca, (x_vel, y_vel)) 

            # orcavel_x, orcavel_y = sim.getAgentVelocity(general_database.human2_orca)

            vel = np.linalg.norm([orcavel_x, orcavel_y])

            new_x_vel = orcavel_x*math.cos(yaw)+ orcavel_y*math.sin(yaw) #vel*math.cos(yaw) 
            new_y_vel = -orcavel_x*math.sin(yaw)+ orcavel_y*math.cos(yaw) #vel*math.sin(yaw)
    
            # print("human 1 orca calculated velocity", new_x_vel, new_y_vel, i)

            # print("orca velocity", orcavel_x, orcavel_y, i)
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
            # sim.setAgentPosition(database.robot_orca, (database.current_robot_pose_x, database.current_robot_pose_y))
            # sim.setAgentPosition(general_database.human2_orca, (orca_human1.current_pose.pose.position.x, orca_human1.current_pose.pose.position.y))
            # sim.setAgentPosition(general_database.human3_orca, (orca_human2.current_pose.pose.position.x, orca_human2.current_pose.pose.position.y))
            # sim.setAgentPosition(database.human1_orca, (database.current_human1_pose_x, database.current_human1_pose_y))

            # rospy.sleep(0.02)                             # sleep of 100 Hz needed
            data_back = copy.deepcopy(orca_human2)
            x_vel, y_vel, yaw, omega = velocity(new_plan, data_back, 1, tfBuffer)
            # print("human 2 caleculated velocity", x_vel, y_vel, i)

            # print("calculated velocity", x_vel, y_vel, i)
	        # angle = 0.1 # std::rand() * 2.0f * M_PI / RAND_MAX;
		    # dist = 0.1 # std::rand() * 0.0001f / RAND_MAX;

		    # sim->setAgentPrefVelocity(i, sim->getAgentPrefVelocity(i) +
		    #                       dist * RVO::Vector2(std::cos(angle), std::sin(angle)));

                    

            sim.setAgentPrefVelocity(general_database.human3_orca, (x_vel, y_vel))  #check this sequence
            sim.doStep()
            sim.setAgentPosition(general_database.human2_orca, (orca_human1.current_pose.pose.position.x, orca_human1.current_pose.pose.position.y))
            sim.setAgentPosition(general_database.human3_orca, (orca_human2.current_pose.pose.position.x, orca_human2.current_pose.pose.position.y))
            orcavel_x, orcavel_y = sim.getAgentVelocity(general_database.human3_orca)

            # x_vel = orcavel_x + (dist * math.cos(angle))   
            # y_vel = orcavel_y + (dist * math.sin(angle))  
            # print("human 2 orca calculated velocity", orcavel_x, orcavel_y, i)

            # sim.setAgentPrefVelocity(general_database.human3_orca, (x_vel, y_vel)) 

            # orcavel_x, orcavel_y = sim.getAgentVelocity(general_database.human3_orca)

            vel = np.linalg.norm([orcavel_x, orcavel_y])

            new_x_vel = orcavel_x*math.cos(yaw)+ orcavel_y*math.sin(yaw) #vel*math.cos(yaw) 
            new_y_vel = -orcavel_x*math.sin(yaw)+ orcavel_y*math.cos(yaw) #vel*math.sin(yaw)

            # print("human 2 orca calculated velocity", new_x_vel, new_y_vel, i)

            # print("orca velocity", orcavel_x, orcavel_y, i)
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

    # orca_human1 = orca_instance([1.66, 1.7])
    # orca_human1 = orca_instance([8.5, 10.0])
    # orca_human1 = orca_instance([10.2, 12.9])
    # orca_human1 = orca_instance([7.4, 2.31])
    # orca_human1 = orca_instance([8.37, -4.85])
    # orca_human1 = orca_instance([1.0, 3.75])
    # orca_human1 = orca_instance([2.74, -2.2])
    orca_human1 = orca_instance([8.41, 2.31])    

    # orca_human2 = orca_instance([8.5, 10])
    # orca_human2 = orca_instance([1.66, 1.7])
    # orca_human2 = orca_instance([10.4, 2.23])

    # orca_human2 = orca_instance([1.4, 15.0])
    orca_human2 = orca_instance([8.45, -4.85])
    # orca_human2 = orca_instance([8.37, -4.85])
    # orca_human2 = orca_instance([8.20, 2.31])
    # orca_human1 = orca_instance([1.2, -2.1])

    # rospy.Subscriber('/morse_agents/human1/odom', Odometry, update_human1_pose, general_database)
    rospy.Subscriber('/morse_agents/human2/odom', Odometry, update_orca_human_pose_callback, orca_human1)
    rospy.Subscriber('/morse_agents/human3/odom', Odometry, update_orca_human_pose_callback, orca_human2)    
    rospy.Subscriber('/odom', Odometry, update_robot_pose, general_database)
    rospy.wait_for_service('gbl_planner/make_plan')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    

    sim = rvo2.PyRVOSimulator(1/60.0, 1.5, 3, 2.5, 4.5, 0.4, 1.2) #timestep, neighborDist, #maxNeighbors, timeHorizon, #timeHorizonObst, float radius, float maxSpeed

    rospy.sleep(0.001) # 1 ms wait for data to get updated
    add_orca_agents(sim, general_database, orca_human1, orca_human2) 


    # new_plan = get_plan(orca_human1)
    # path_message_conversion(new_plan)

    new_plan_hum_1 = plan(orca_human1, 1)
    new_plan_hum_2 = plan(orca_human2, 2)

    rate = rospy.Rate(1) # 24hz maximum (41 ms)
    control_rate = rospy.Rate(20)
    while not rospy.is_shutdown(): # try normal while loop also for making it faster

        # k = k + 1
        # if(k%10 == 0):
        new_plan_hum_1 = plan(orca_human1, 1)
        new_plan_hum_2 = plan(orca_human2, 2)
            # if(k == 100):
            #     k = 0
        # sim.processObstacles()
        # new_plan = get_plan(orca_human1)
        # gplan = path_message_conversion(new_plan)
        # orca_human2_plan_pub.publish(gplan) #(new_plan.path)
        # new_plan_hum_1 = plan(orca_human1, 1)
        # new_plan_hum_2 = plan(orca_human2, 2)

        move_logic(sim, general_database, new_plan_hum_1, tfBuffer, 1, orca_human1, orca_human2)
        move_logic(sim, general_database, new_plan_hum_2, tfBuffer, 2, orca_human1, orca_human2)

              
        publish_human2_vel(orca_human1)
        publish_human3_vel(orca_human2)
        control_rate.sleep()
            
            

if __name__ == "__main__":
    main()
