#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

# imports
import math
from math import sin, cos, pi

import rospy
from std_msgs.msg import String
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

# value that we'll get from the topic '/decision' and that will be use to see if we need to use this algorithm
state = 1

# this method is called everytime something is published on the topic '/decision'
def callback_decision(data):

    global state
    # we change the value by the new value
    state = int(data.data)

# this function is called everytime something is published on the topic '/wheel_informations'
def callback_info(data):
    print("something was heard")

def wheel_encoders():

    # variables concerning the current location of the robot

    x = 0.0
    y = 0.0
    th = 0.0

    # speed = ( right speed + left speed ) / 2
    # vx = speed
    # vy = 0
    # vth = (right speed - left speed) / length between two wheels

    # exemples values to test if the program is working
    vx = 0.5
    vy = 0.0
    vth = 0.5

    # initialization of the node
    rospy.init_node('wheel_encoders', anonymous=True)

    # will publish information every 1 second
    r = rospy.Rate(1)

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    # subscribe to the topic '/decision' 
    rospy.Subscriber('decision', String, callback_decision)

    # subscribe to the topic 'wheel_informations
    rospy.Subscriber('wheel_informations', String, callback_info)
    
    # publish the odometry information
    odom_pub = rospy.Publisher("odom_publisher", Odometry, queue_size=50)

    # will be used to publish tranform over tf
    odom_broadcaster = tf.TransformBroadcaster()

    while not rospy.is_shutdown():

        global state

	if state == 1:	

	    current_time = rospy.Time.now()

	    # compute odometry in a typical way given the velocities of the robot
	    dt = (current_time - last_time).to_sec()
	    delta_x = (vx * cos(th) - vy * sin(th)) * dt
	    delta_y = (vx * sin(th) + vy * cos(th)) * dt
	    delta_th = vth * dt

	    x += delta_x
	    y += delta_y
	    th += delta_th

	    # since all odometry is 6DOF (6 Degrees Of Freedom) we'll need a quaternion created from yaw
	    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

	    # first, we'll publish the transform over tf
	    odom_broadcaster.sendTransform(
		(x, y, 0.),
		odom_quat,
		current_time,
		"base_link",
		"odom"
            )

	    # next, we'll publish the odometry message over ROS
	    odom = Odometry()
	    odom.header.stamp = current_time
	    odom.header.frame_id = "odom"

	    # set the position
	    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

	    # set the velocity
	    odom.child_frame_id = "base_link"
	    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

	    # publish the message
	    odom_pub.publish(odom)
	    print("something has been published")

	    last_time = current_time

	r.sleep()

# Main
if __name__ == '__main__':
    wheel_encoders()
