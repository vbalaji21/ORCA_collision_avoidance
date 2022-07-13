#!/usr/bin/env python

# from tracemalloc import start
import rospy
from nav_msgs.srv import GetPlan, GetPlanRequest
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import InHuS_Social_Navigation.src.simulators.morse_ros.morse_ros.scenarios.rvo21 as rvo21
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import numpy as np
import math

# Obstacles are also supported as vertices
# o1 = sim.addObstacle([(0.1, 0.1), (-0.1, 0.1), (-0.1, -0.1)])
# sim.processObstacles()

vel_msg = Twist()
pub = rospy.Publisher('/morse_agents/human2/cmd_vel', Twist, queue_size=10, latch=False)


class data_instance:
  def __init__(self):
    self.start_human2_pose_x = 0
    self.start_human2_pose_y = 0
    self.start_human2_quat_orientation_x = 0            #start orientation  
    self.start_human2_quat_orientation_y = 0            #start orientation 
    self.start_human2_quat_orientation_z = 0            #start orientation
    self.start_human2_quat_orientation_w = 0            #start orientation     

    self.start_human2_yaw = 0
      
    self.goal_human2_pose_x =  7.5 #9.4
    self.goal_human2_pose_y =  1.0 #6.4
    self.goal_human2_quat_orientation_x = 0            #goal orientation  
    self.goal_human2_quat_orientation_y = 0            #goal orientation 
    self.goal_human2_quat_orientation_z = 0            #goal orientation
    self.goal_human2_quat_orientation_w = 1            #goal orientation         

    #ORCA agent numbers
    self.human2_orca = None
    self.human1_orca = None
    self.robot_orca = None

    self.next_human2_state = Twist()
    self.plan = 0

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

def update_human2_pose(data, args):
    args.start_human2_pose_x = data.pose.pose.position.x
    args.start_human2_pose_y = data.pose.pose.position.y

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

def get_plan(database):

    start = PoseStamped()
    start.header.seq = 0
    start.header.frame_id = "map"
    start.header.stamp = rospy.Time(0)
    start.pose.position.x = database.start_human2_pose_x   
    start.pose.position.y = database.start_human2_pose_y
    # start.pose.position.z = database.start_human2_pose_z

    Goal = PoseStamped()
    Goal.header.seq = 0
    Goal.header.frame_id = "map"
    Goal.header.stamp = rospy.Time(0)
    Goal.pose.position.x = database.goal_human2_pose_x 
    Goal.pose.position.y = database.goal_human2_pose_y

    Goal.pose.orientation.x = database.goal_human2_quat_orientation_x
    Goal.pose.orientation.y = database.goal_human2_quat_orientation_y    
    Goal.pose.orientation.z = database.goal_human2_quat_orientation_z
    Goal.pose.orientation.w = database.goal_human2_quat_orientation_w


    get_plan_service = rospy.ServiceProxy('/human1/move_base/GlobalPlanner/make_plan', GetPlan)   # Adding human1 global planner to identify robot as well    move_base/GlobalPlanner/make_plan
    r = GetPlan()
    r.start = start
    r.goal = Goal 
    r.tolerance = 0.5
    plan = get_plan_service(r.start, r.goal, r.tolerance)
    # print("plan", plan.plan.poses[0].pose.position.x)
    return plan

def normalize_theta(theta):
  PI = math.pi
  result = math.fmod(theta + PI, 2.0 * PI)
  if result <= 0:
    return result + PI
  return result - PI

def velocity(plan, database, i):

    x_vel = clamp((plan.plan.poses[i].pose.position.x - database.start_human2_pose_x) / (0.05) , -1.2, 1.2) #can be i-1
    y_vel = clamp((plan.plan.poses[i].pose.position.y - database.start_human2_pose_y) / (0.05) , -1.2, 1.2) #(plan.plan.poses[1].pose.position.y - plan.plan.poses[0].pose.position.y) / (0.05)

    orientation_list = [plan.plan.poses[i].pose.orientation.x, plan.plan.poses[i].pose.orientation.y, plan.plan.poses[i].pose.orientation.z, plan.plan.poses[i].pose.orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    print("yaw", yaw)

    yaw_change = normalize_theta(yaw - database.start_human2_yaw)

    print("yaw change", yaw_change)

    # #type(pose) = geometry_msgs.msg.Pose
    # quaternion = (
    #     plan.plan.poses[i].orientation.x,
    #     plan.plan.poses[i].orientation.y,
    #     plan.plan.poses[i].orientation.z,
    #     plan.plan.poses[i].orientation.w)

    # euler = tf.transformations.euler_from_quaternion(quaternion)
    # roll = euler[0]
    # pitch = euler[1]
    # yaw = euler[2]

    return x_vel, y_vel, yaw_change

def publish_human2_vel(database):
    # print("published velocity", database.next_human2_state)
    pub.publish(database.next_human2_state)

def add_orca_agents(sim, database):
    # Pass either just the position (the other parameters then use the default values passed to the PyRVOSimulator constructor), or pass all available parameters.
    database.human2_orca = sim.addAgent((database.start_human2_pose_x, database.start_human2_pose_y))
    database.human1_orca = sim.addAgent((database.current_human1_pose_x, database.current_human1_pose_y))  #read from odom topic
    database.robot_orca  = sim.addAgent((database.current_robot_pose_x, database.current_robot_pose_y))  #read from odom topic

    # o1 = sim.addObstacle([(5.66, 7.35), (5.68, 6.17), (6.77, 6.17), (5.59, 6.95), (5.68, 6.35)])
    o1 = sim.addObstacle([(-0.0144, -0.178), (1.75, -0.266), (1.81, 0.204), (-0.0437, 0.262)]) #

    # print("obstacle no", o1)
    # k1 = sim.processObstacles()
    # print("process obstacle", k1)
    sim.processObstacles()

#    a3 = sim.addAtimer_callbackefVelocity(human2_orca, (x_vel, y_vel))
    # sim.setAgentPrefVelocity(a1, (-1, 1))
    # sim.setAgentPrefVelocity(a2, (-1, -1))
    # sim.setAgentPrefVelocity(a3, (1, -1))


def main():
    rospy.init_node('get_plan_client')
    database = data_instance()
    # global vel_msg

    rospy.Subscriber('/morse_agents/human1/odom', Odometry, update_human1_pose, database)
    rospy.Subscriber('/morse_agents/human2/odom', Odometry, update_human2_pose, database)
    rospy.Subscriber('/odom', Odometry, update_robot_pose, database)
    rospy.wait_for_service('move_base/GlobalPlanner/make_plan')
    

    sim = rvo21.PyRVOSimulator(1/20.0, 1.5, 5, 1.5, 2, 0.4, 2) #timestep, neighborDist, #maxNeighbors, timeHorizon, #timeHorizonObst, float radius, float maxSpeed
    # #, const Vector2 &velocity) : defaultAgent_(NULL)

    rospy.sleep(0.001) # 1 ms wait for data to get updated
    new_plan = get_plan(database)
    print("new plan", new_plan)
    add_orca_agents(sim, database)


    rate = rospy.Rate(20) # 24hz maximum (41 ms)
    while not rospy.is_shutdown(): # try normal while loop also for making it faster
        

        if(database.plan == 0):
            # print("length of the data", len(new_plan.plan.poses))
            for i in range(len(new_plan.plan.poses)):

                sim.setAgentPosition(database.robot_orca, (database.current_robot_pose_x, database.current_robot_pose_y))
                sim.setAgentPosition(database.human2_orca, (database.start_human2_pose_x, database.start_human2_pose_y))
                sim.setAgentPosition(database.human1_orca, (database.current_human1_pose_x, database.current_human1_pose_y))

                # sim.processObstacles()
                rospy.sleep(0.05)
                x_vel, y_vel, yaw = velocity(new_plan, database, i)
                # print("calculated velocity", x_vel, y_vel, i)
                sim.setAgentPrefVelocity(database.human2_orca, (x_vel, y_vel))  #check this sequence
                sim.doStep()
                orcavel_x, orcavel_y = sim.getAgentVelocity(database.human2_orca)

                # print("orca velocity", orcavel_x, orcavel_y, i)
                database.next_human2_state.linear.x = orcavel_x
                database.next_human2_state.linear.y = orcavel_y
                database.next_human2_state.linear.z = 0
                database.next_human2_state.angular.x = 0
                database.next_human2_state.angular.y = 0
                database.next_human2_state.angular.z = 0

                if(i == len(new_plan.plan.poses) or i == len(new_plan.plan.poses)-1):
                    database.next_human2_state.linear.x = 0
                    database.next_human2_state.linear.y = 0
                    database.plan = 1                

                publish_human2_vel(database)
            
        rate.sleep()


if __name__ == "__main__":
    main()
