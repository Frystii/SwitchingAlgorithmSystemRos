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

# value that we'll get from the topic '/decision' and that will be use to see if we need to use this algorithm
global state
state = 1

# this function is called everytime something is published on the topic '/decision'
def callback_decision(data):

    global state
    # we change the value by the new value
    state = int(data.data)
    print(state)

def slam():

    # initialization of the node
    rospy.init_node('slam', anonymous=True)

    # will refresh every second
    r = rospy.Rate(1)

    # subscribe to the topic 'decision' 
    rospy.Subscriber('decision', String, callback_decision)

    # set the update interval at 5.0 seconds between each update
    rospy.set_param('/slam_gmapping/map_update_interval',5.0)

    while not rospy.is_shutdown():

        print(state,1,2)
	if state == 2:
	    rospy.set_param('/slam_gmapping/map_update_interval',5.0)

	else:
    	     rospy.set_param('/slam_gmapping/map_update_interval',1000000.0)

    	print(rospy.get_param('/slam_gmapping/map_update_interval'))
	
	r.sleep()

# Main
if __name__ == '__main__':
    slam()
