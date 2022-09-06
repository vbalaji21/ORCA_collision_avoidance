#include "ros/ros.h"
#include "inhus/PoseVel.h"
#include "cohan_msgs/TrackedAgents.h"
#include "cohan_msgs/TrackedSegmentType.h"
#include "cohan_msgs/AgentType.h"
#include <tf2/LinearMath/Quaternion.h>
#include "nav_msgs/Odometry.h"
#include <tf/tf.h>

ros::Publisher pub_tracked_agents;

cohan_msgs::TrackedAgent robot_tracked_agent;
cohan_msgs::TrackedAgent human1_orca_tracked_agent;
cohan_msgs::TrackedAgent human2_orca_tracked_agent;

tf2::Quaternion myQuaternion;
// robot_tracked_agent.type = cohan_msgs::AgentType::HUMAN;
// robot_tracked_agent.name = "robot";
// robot_tracked_agent.segments.push_back(agent_segment);
// robot_tracked_agent.track_id = 1;

void agentCB(const inhus::PoseVel::ConstPtr& msg)
{
    cohan_msgs::TrackedSegment agent_segment;
    agent_segment.type = cohan_msgs::TrackedSegmentType::TORSO;
    geometry_msgs::Pose pose;
    pose.position.x = msg->pose.x;
    pose.position.y = msg->pose.y;
    tf2::Quaternion q;
    q.setRPY(0,0,msg->pose.theta);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    agent_segment.pose.pose = pose;
    agent_segment.twist.twist = msg->vel;
    
    // cohan_msgs::TrackedAgent tracked_agent;
    robot_tracked_agent.type = cohan_msgs::AgentType::HUMAN;
    robot_tracked_agent.name = "robot";
    robot_tracked_agent.segments[0] = agent_segment;
    //robot_tracked_agent.segments.push_back(agent_segment);
    robot_tracked_agent.track_id = 1;

    cohan_msgs::TrackedAgents tracked_agents;
    tracked_agents.agents.push_back(robot_tracked_agent);
    tracked_agents.agents.push_back(human1_orca_tracked_agent);
    tracked_agents.agents.push_back(human2_orca_tracked_agent);
    tracked_agents.header.stamp = ros::Time::now();
    tracked_agents.header.frame_id = "map";

    pub_tracked_agents.publish(tracked_agents);
}

void agentCB_ORCA1(const nav_msgs::Odometry::ConstPtr& msg)
{
    cohan_msgs::TrackedSegment agent_segment;
    agent_segment.type = cohan_msgs::TrackedSegmentType::TORSO;
    geometry_msgs::Pose pose;
    pose.position.x = msg->pose.pose.position.x; //msg->pose.x;
    pose.position.y = msg->pose.pose.position.y; //msg->pose.y;
    tf2::Quaternion q;
    // q.setRPY(0,0,msg->pose.theta);
    pose.orientation.x = msg->pose.pose.orientation.x; // q.x();
    pose.orientation.y = msg->pose.pose.orientation.y; // q.y();
    pose.orientation.z = msg->pose.pose.orientation.z; // q.z();
    pose.orientation.w = msg->pose.pose.orientation.w; // q.w();
    agent_segment.pose.pose = pose;
    agent_segment.twist.twist = msg->twist.twist; //msg->vel;

    // tf::Quaternion q(
    //     msg->pose.pose.orientation.x,
    //     msg->pose.pose.orientation.y,
    //     msg->pose.pose.orientation.z,
    //     msg->pose.pose.orientation.w);
    // tf::Matrix3x3 m(q);
    // double roll, pitch, yaw;
    // m.getRPY(roll, pitch, yaw);

    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);


    double roll, pitch, yaw;// Define storage r\p\y The container of 
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);// convert 

	agent_segment.twist.twist.linear.x = msg->twist.twist.linear.x*cos(yaw) - msg->twist.twist.linear.y*sin(yaw);
	agent_segment.twist.twist.linear.y = msg->twist.twist.linear.x*sin(yaw) - msg->twist.twist.linear.y*cos(yaw); 
    
    // cohan_msgs::TrackedAgent tracked_agent;
    human1_orca_tracked_agent.type = cohan_msgs::AgentType::HUMAN;
    human1_orca_tracked_agent.name = "orcahuman1";
    human1_orca_tracked_agent.segments[0] = agent_segment;
    //human1_orca_tracked_agent.segments.push_back(agent_segment);
    human1_orca_tracked_agent.track_id = 2;

    cohan_msgs::TrackedAgents tracked_agents;
    tracked_agents.agents.push_back(robot_tracked_agent);
    tracked_agents.agents.push_back(human1_orca_tracked_agent);
    tracked_agents.agents.push_back(human2_orca_tracked_agent);
    tracked_agents.header.stamp = ros::Time::now();
    tracked_agents.header.frame_id = "map";

    pub_tracked_agents.publish(tracked_agents);
}

void agentCB_ORCA2(const nav_msgs::Odometry::ConstPtr& msg)
{
    cohan_msgs::TrackedSegment agent_segment;
    agent_segment.type = cohan_msgs::TrackedSegmentType::TORSO;
    geometry_msgs::Pose pose;
    pose.position.x = msg->pose.pose.position.x; //msg->pose.x;
    pose.position.y = msg->pose.pose.position.y; //msg->pose.y;
    tf2::Quaternion q;
    // q.setRPY(0,0,msg->pose.theta);
    pose.orientation.x = msg->pose.pose.orientation.x; // q.x();
    pose.orientation.y = msg->pose.pose.orientation.y; // q.y();
    pose.orientation.z = msg->pose.pose.orientation.z; // q.z();
    pose.orientation.w = msg->pose.pose.orientation.w; // q.w();
    agent_segment.pose.pose = pose;
    agent_segment.twist.twist = msg->twist.twist; //msg->vel;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);


    double roll, pitch, yaw;// Define storage r\p\y The container of 
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);// convert 

	agent_segment.twist.twist.linear.x = msg->twist.twist.linear.x*cos(yaw) - msg->twist.twist.linear.y*sin(yaw);
	agent_segment.twist.twist.linear.y = msg->twist.twist.linear.x*sin(yaw) - msg->twist.twist.linear.y*cos(yaw); 
    
    // cohan_msgs::TrackedAgent tracked_agent;
    human2_orca_tracked_agent.type = cohan_msgs::AgentType::HUMAN;
    human2_orca_tracked_agent.name = "orcahuman2";
    //human2_orca_tracked_agent.segments.push_back(agent_segment);
    human2_orca_tracked_agent.segments[0] = agent_segment;
    human2_orca_tracked_agent.track_id = 3;

    cohan_msgs::TrackedAgents tracked_agents;
    tracked_agents.agents.push_back(robot_tracked_agent);
    tracked_agents.agents.push_back(human1_orca_tracked_agent);
    tracked_agents.agents.push_back(human2_orca_tracked_agent);
    tracked_agents.header.stamp = ros::Time::now();
    tracked_agents.header.frame_id = "map";

    pub_tracked_agents.publish(tracked_agents);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "input_interface");
	ros::NodeHandle nh;

    cohan_msgs::TrackedSegment agent_segment;
    agent_segment.type = cohan_msgs::TrackedSegmentType::TORSO;

    robot_tracked_agent.segments.push_back(agent_segment);
    human1_orca_tracked_agent.segments.push_back(agent_segment);
    human2_orca_tracked_agent.segments.push_back(agent_segment);


    ros::Subscriber sub_agent = nh.subscribe("interface/in/robot_pose_vel", 10, agentCB);
    ros::Subscriber sub_agent_2 = nh.subscribe("/morse_agents/human2/odom", 10, agentCB_ORCA1);
    ros::Subscriber sub_agent_3 = nh.subscribe("/morse_agents/human3/odom", 10, agentCB_ORCA2);

    pub_tracked_agents = nh.advertise<cohan_msgs::TrackedAgents>("tracked_agents", 10);

    ros::spin();

	return 0;
}
