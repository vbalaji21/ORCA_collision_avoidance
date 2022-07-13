#!/usr/bin/python

# Brief: This node subscribes to the robots published on /humani, i=1,2, .. and robot, and publishes /tracked_agents required for CoHAN

import sys
import rospy
import tf2_ros
import tf2_geometry_msgs
import time
from cohan_msgs.msg import TrackedAgents, TrackedAgent, AgentMarkerStamped, TrackedSegment, TrackedSegmentType, AgentType
from geometry_msgs.msg import PointStamped, TwistStamped
import message_filters
from nav_msgs.msg import Odometry


class MorseAgents(object):

    def __init__(self):
        self.tracked_agents_pub = []
        self.Segment_Type = TrackedSegmentType.TORSO
        self.agents = TrackedAgents()
        self.robot = TrackedAgent()
        self.sig_1 = False
        self.sig_2 = False


    def AgentsPub(self):
        rospy.init_node('orca_agents', anonymous=True)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        agent_sub = []

        rospy.Subscriber("/morse_agents/human1/marker", AgentMarkerStamped, self.AgentsCB)
        rospy.Subscriber("/marker", AgentMarkerStamped, self.RobotCB)
        self.tracked_agents_pub = rospy.Publisher("tracked_orca_agents", TrackedAgents, queue_size=1)
        rospy.Timer(rospy.Duration(0.02), self.publishAgents)
        rospy.spin()

    def AgentsCB(self,msg):
        tracked_agents = TrackedAgents()
        agent_segment = TrackedSegment()
        agent_segment.type = self.Segment_Type
        agent_segment.pose.pose = msg.agent.pose 
        agent_segment.twist.twist = msg.agent.velocity 

        tracked_agent = TrackedAgent()
        tracked_agent.type = AgentType.HUMAN
        tracked_agent.name = "Inhus_human"
        tracked_agent.radius = 0.4
        tracked_agent.segments.append(agent_segment)
        tracked_agents.agents.append(tracked_agent)
        self.agents = tracked_agents
        self.sig_1 = True

    def RobotCB(self, msg):
        agent_segment = TrackedSegment()
        agent_segment.type = self.Segment_Type
        agent_segment.pose.pose = msg.agent.pose 
        agent_segment.twist.twist = msg.agent.velocity 
        tracked_agent = TrackedAgent()
        tracked_agent.type = AgentType.ROBOT
        tracked_agent.name = "robot"
        tracked_agent.radius = 0.6
        tracked_agent.segments.append(agent_segment)
        self.robot = tracked_agent
        self.sig_2 = True

    def publishAgents(self, event):
        if(self.sig_1 and self.sig_2):
            self.agents.header.stamp = rospy.Time.now()
            self.agents.header.frame_id = "map"
            self.agents.agents.append(self.robot)
            for agent_id in range(0, len(self.agents.agents)):
                self.agents.agents[agent_id].track_id = agent_id+1
            self.tracked_agents_pub.publish(self.agents)
            self.sig_1 = False
            self.sig_2 = False



if __name__ == '__main__':
    agents = MorseAgents()
    agents.AgentsPub()
