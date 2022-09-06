#! /usr/bin/env python

import rospy
from visualization_msgs.msg import Marker  # , MarkerArray
from nav_msgs.msg import Odometry

# def callback(data):
#   print("data", data.header)
#   # marker.pose.position.x = data.pose.position.x
#   # marker.pose.position.y = data.pose.position.y
#   # marker.pose.position.z = data.pose.position.z

inhus_marker = Marker()
orca_human1_marker = Marker()
orca_human2_marker = Marker()

inhus_marker_arrow = Marker()
orca_human1_marker_arrow = Marker()
orca_human2_marker_arrow = Marker()

def define_inhus_marker():
    inhus_marker.header.frame_id = "/map"
    # inhus_marker.header.stamp = 0

    # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
    inhus_marker.type = 3
    inhus_marker.id = 0

    # Set the scale of the marker
    inhus_marker.scale.x = 0.6 #0.2 #0.2  # 1.938 #2.153
    inhus_marker.scale.y = 0.6 #0.5 #0.1  # 1.20 #1.3971
    inhus_marker.scale.z = 0.3

    # Set the color
    inhus_marker.color.r = 0.0
    inhus_marker.color.g = 0.0
    inhus_marker.color.b = 1.0
    inhus_marker.color.a = 1.0

def define_inhus_marker_arrow():
    inhus_marker_arrow.header.frame_id = "/map"
    # inhus_marker_arrow.header.stamp = 0

    # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
    inhus_marker_arrow.type = 0
    inhus_marker_arrow.id = 0

    # Set the scale of the marker
    inhus_marker_arrow.scale.x = 0.6 #0.2 #0.2  # 1.938 #2.153
    inhus_marker_arrow.scale.y = 0.05 #0.5 #0.1  # 1.20 #1.3971
    inhus_marker_arrow.scale.z = 0.05

    # Set the color
    inhus_marker_arrow.color.r = 1.0
    inhus_marker_arrow.color.g = 1.0
    inhus_marker_arrow.color.b = 0.0
    inhus_marker_arrow.color.a = 1.0

def define_orca1_marker():
    orca_human1_marker.header.frame_id = "/map"
    # orca_human1_marker.header.stamp = rospy.Time.now()

    # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
    orca_human1_marker.type = 3
    orca_human1_marker.id = 0

    # Set the scale of the marker
    orca_human1_marker.scale.x = 0.6 #0.2 #0.2  # 1.938 #2.153
    orca_human1_marker.scale.y = 0.6 #0.5 #0.1  # 1.20 #1.3971
    orca_human1_marker.scale.z = 0.3

    # Set the color
    orca_human1_marker.color.r = 0.0
    orca_human1_marker.color.g = 1.0
    orca_human1_marker.color.b = 0.0
    orca_human1_marker.color.a = 1.0

def define_orca1_marker_arrow():
    orca_human1_marker_arrow.header.frame_id = "/map"
    # orca_human1_marker_arrow.header.stamp = rospy.Time.now()

    # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
    orca_human1_marker_arrow.type = 0
    orca_human1_marker_arrow.id = 0

    # Set the scale of the marker
    orca_human1_marker_arrow.scale.x = 0.6
    orca_human1_marker_arrow.scale.y = 0.05
    orca_human1_marker_arrow.scale.z = 0.05

    # Set the color
    orca_human1_marker_arrow.color.r = 1.0
    orca_human1_marker_arrow.color.g = 1.0
    orca_human1_marker_arrow.color.b = 0.0
    orca_human1_marker_arrow.color.a = 1.0


def define_orca2_marker():
    orca_human2_marker.header.frame_id = "/map"
    # orca_human2_marker.header.stamp = rospy.Time.now()

    # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
    orca_human2_marker.type = 3
    orca_human2_marker.id = 0

    # Set the scale of the marker
    orca_human2_marker.scale.x = 0.6 #0.2 #0.2  # 1.938 #2.153
    orca_human2_marker.scale.y = 0.6 #0.5 #0.1  # 1.20 #1.3971
    orca_human2_marker.scale.z = 0.3

    # Set the color
    orca_human2_marker.color.r = 0.0
    orca_human2_marker.color.g = 1.0
    orca_human2_marker.color.b = 0.0
    orca_human2_marker.color.a = 1.0

def define_orca2_marker_arrow():
    orca_human2_marker_arrow.header.frame_id = "/map"
    # orca_human2_marker_arrow.header.stamp = rospy.Time.now()

    # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
    orca_human2_marker_arrow.type = 0
    orca_human2_marker_arrow.id = 0

    # Set the scale of the marker
    orca_human2_marker_arrow.scale.x = 0.6
    orca_human2_marker_arrow.scale.y = 0.05
    orca_human2_marker_arrow.scale.z = 0.05

    # Set the color
    orca_human2_marker_arrow.color.r = 1.0
    orca_human2_marker_arrow.color.g = 1.0
    orca_human2_marker_arrow.color.b = 0.0
    orca_human2_marker_arrow.color.a = 1.0


def inhus_callback(data, args):
    args.pose.position.x = data.pose.pose.position.x
    args.pose.position.y = data.pose.pose.position.y
    args.pose.position.z = data.pose.pose.position.z

    args.pose.orientation.x = data.pose.pose.orientation.x
    args.pose.orientation.y = data.pose.pose.orientation.y
    args.pose.orientation.z = data.pose.pose.orientation.z
    args.pose.orientation.w = data.pose.pose.orientation.w

def inhus_arrow_callback(data, args):
    args.pose.position.x = data.pose.pose.position.x
    args.pose.position.y = data.pose.pose.position.y
    args.pose.position.z = data.pose.pose.position.z

    args.pose.orientation.x = data.pose.pose.orientation.x
    args.pose.orientation.y = data.pose.pose.orientation.y
    args.pose.orientation.z = data.pose.pose.orientation.z
    args.pose.orientation.w = data.pose.pose.orientation.w

def orca_human1_callback(data, args):
    args.pose.position.x = data.pose.pose.position.x
    args.pose.position.y = data.pose.pose.position.y
    args.pose.position.z = data.pose.pose.position.z

    args.pose.orientation.x = data.pose.pose.orientation.x
    args.pose.orientation.y = data.pose.pose.orientation.y
    args.pose.orientation.z = data.pose.pose.orientation.z
    args.pose.orientation.w = data.pose.pose.orientation.w

def orca_human1_arrow_callback(data, args):
    args.pose.position.x = data.pose.pose.position.x
    args.pose.position.y = data.pose.pose.position.y
    args.pose.position.z = data.pose.pose.position.z

    args.pose.orientation.x = data.pose.pose.orientation.x
    args.pose.orientation.y = data.pose.pose.orientation.y
    args.pose.orientation.z = data.pose.pose.orientation.z
    args.pose.orientation.w = data.pose.pose.orientation.w

def orca_human2_callback(data, args):
    args.pose.position.x = data.pose.pose.position.x
    args.pose.position.y = data.pose.pose.position.y
    args.pose.position.z = data.pose.pose.position.z

    args.pose.orientation.x = data.pose.pose.orientation.x
    args.pose.orientation.y = data.pose.pose.orientation.y
    args.pose.orientation.z = data.pose.pose.orientation.z
    args.pose.orientation.w = data.pose.pose.orientation.w

def orca_human2_arrow_callback(data, args):
    args.pose.position.x = data.pose.pose.position.x
    args.pose.position.y = data.pose.pose.position.y
    args.pose.position.z = data.pose.pose.position.z

    args.pose.orientation.x = data.pose.pose.orientation.x
    args.pose.orientation.y = data.pose.pose.orientation.y
    args.pose.orientation.z = data.pose.pose.orientation.z
    args.pose.orientation.w = data.pose.pose.orientation.w


def main():
    rospy.init_node("mb_humans_rviz_marker")

    inhus_marker_pub = rospy.Publisher("/movebase_inhus_visualization_marker", Marker, queue_size=10)
    orca1_marker_pub = rospy.Publisher("/movebase_orca_human1_visualization_marker", Marker, queue_size=10)
    orca2_marker_pub = rospy.Publisher("/movebase_orca_human2_visualization_marker", Marker, queue_size=10)

    inhus_marker_arrow_pub = rospy.Publisher("/movebase_inhus_visualization_marker_arrow", Marker, queue_size=10)
    orca1_marker_arrow_pub = rospy.Publisher("/movebase_orca_human1_visualization_marker_arrow", Marker, queue_size=10)
    orca2_marker_arrow_pub = rospy.Publisher("/movebase_orca_human2_visualization_marker_arrow", Marker, queue_size=10)

    define_inhus_marker()
    define_inhus_marker_arrow()
    define_orca1_marker()
    define_orca1_marker_arrow()
    define_orca2_marker()
    define_orca2_marker_arrow()


    sub1 = rospy.Subscriber("/morse_agents/human1/odom", Odometry, inhus_callback, inhus_marker)
    sub2 = rospy.Subscriber("/morse_agents/human2/odom", Odometry, orca_human1_callback, orca_human1_marker)
    sub3 = rospy.Subscriber("/morse_agents/human3/odom", Odometry, orca_human2_callback, orca_human2_marker)

    sub11 = rospy.Subscriber("/morse_agents/human1/odom", Odometry, inhus_arrow_callback, inhus_marker_arrow)
    sub22 = rospy.Subscriber("/morse_agents/human2/odom", Odometry, orca_human1_arrow_callback, orca_human1_marker_arrow)
    sub33 = rospy.Subscriber("/morse_agents/human3/odom", Odometry, orca_human2_arrow_callback, orca_human2_marker_arrow)

    while not rospy.is_shutdown():
        inhus_marker_pub.publish(inhus_marker)
        inhus_marker_arrow_pub.publish(inhus_marker_arrow)
        orca1_marker_pub.publish(orca_human1_marker)
        orca1_marker_arrow_pub.publish(orca_human1_marker_arrow)
        orca2_marker_pub.publish(orca_human2_marker)
        orca2_marker_arrow_pub.publish(orca_human2_marker_arrow)

        rospy.rostime.wallsleep(0.1)


if __name__ == "__main__":
    main()
