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


# Imports
import rospy
from std_msgs.msg import String

# 0 is for the wheel encoders algorithm
# 1 is for the SLAM algorithm
# by default, we will use the wheel encoders algorithm, as it costs less more than the SLAM algorithm regarding the resources of the robot


def decision_system():

    default_value = 1

    # maximum time in second
    maximum_time_without_using_first_algo = 2
    maximum_time_without_using_second_algo = 2


    time_since_last_use_first_algo = 0
    time_since_last_use_second_algo = 0

    initial_battery_value = 100
    minimum_battery_to_use_second_algo = 30
    decrease_value_for_battery_every_loop = 2

    initial_resources_value = 0
    maximum_resources_to_use_second_algo = 50 
    increase_value_for_resources_every_loop = 2

    # Create the node
    rospy.init_node('decision_system', anonymous=True)

    # Initialize the publisher
    pub = rospy.Publisher('decision', String, queue_size=10)

    # The value is publish every 1/10 = 0.1 second
    rate = rospy.Rate(10) 

    # Initialization of the values
    value = default_value
    battery = initial_battery_value
    resources = initial_resources_value
    
    while not rospy.is_shutdown():

	print("------------")
        print("RESOURCES : " + str(resources))
	print("BATTERY LEVEL : " + str(battery))
	print("TIME SINCE LAST USE FIRST ALGO : " + str(time_since_last_use_first_algo))
	print("TIME SINCE LAST USE SECOND ALGO : " + str(time_since_last_use_second_algo))

        # Get the decision
        decision = getDecision(value)

	if int(decision) == 1:
	    
	    time_since_last_use_first_algo += 0.1

	else :

	    time_since_last_use_second_algo += 0.1


        # Write the decision in the log
        # rospy.loginfo(decision)

        # Publish on the topic '/decision'
        pub.publish(str(decision))

	# Wait 0.1 second
        rate.sleep()

        # Update the value
        if value == 1 and time_since_last_use_first_algo >= maximum_time_without_using_first_algo:

	    if battery <= minimum_battery_to_use_second_algo:
		value = 1

	    else:
		
		value = 2
	    	time_since_last_use_first_algo = 0
	    

	elif value == 2:

	    if battery <= minimum_battery_to_use_second_algo or time_since_last_use_second_algo >= maximum_time_without_using_second_algo or resources >= maximum_resources_to_use_second_algo:
	
	        value = 1
	        time_since_last_use_second_algo = 0

	if battery > 0:
		battery -= decrease_value_for_battery_every_loop

	if resources < 100:
		resources += increase_value_for_resources_every_loop


# Determine decision depending on the value
def getDecision(value):

    # We use the first algorithm (wheel encoders)
    if value == 1:

	return 1

    # We use the second algorithm (SLAM)
    else:

	return 2


decision_system()
