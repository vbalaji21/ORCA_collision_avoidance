#! /usr/bin/env python

import rospy
from visualization_msgs.msg import Marker #, MarkerArray

# def callback(data):
#   print("data", data.header)
#   # marker.pose.position.x = data.pose.position.x
#   # marker.pose.position.y = data.pose.position.y
#   # marker.pose.position.z = data.pose.position.z



def main():
  rospy.init_node('rviz_marker')

  marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 2)
  # sub  = rospy.Subscriber("/move_base/HATebLocalPlannerROS/agent_arrow", MarkerArray, callback)

  marker = Marker()

  marker.header.frame_id = "/map"
  marker.header.stamp = rospy.Time.now()

  # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
  marker.type = 1
  marker.id = 0

  # Set the scale of the marker
  marker.scale.x = 1.0 #1.938 #2.153
  marker.scale.y = 0.6 #1.20 #1.3971
  marker.scale.z = 0.6

  # Set the color
  marker.color.r = 1.0
  marker.color.g = 0.0
  marker.color.b = 0.0
  marker.color.a = 1.0

  # Set the pose of the marker
  marker.pose.position.x = 8.5 #1.131 #1.1765 
  marker.pose.position.y = -1.48 #2.27  #0.7485 
  marker.pose.position.z = 0
  marker.pose.orientation.x = 0.0
  marker.pose.orientation.y = 0.0
  marker.pose.orientation.z = 0.0
  marker.pose.orientation.w = 1.0

  while not rospy.is_shutdown():
    marker_pub.publish(marker)
    rospy.rostime.wallsleep(1.0)

if __name__ == "__main__":
    main()

