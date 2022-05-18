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

def decision_system():

    # Create the node
    rospy.init_node('decision_system', anonymous=True)

    # Initialize the publisher
    pub = rospy.Publisher('decision', String, queue_size=10)

    # The value is publish every 1/0,2 = 5 second
    rate = rospy.Rate(0.2) 

    # Initialization of the value
    value = 0
    
    while not rospy.is_shutdown():

        # Get the decision
        decision = getDecision(value)
        print(decision)

        # Write the decision in the log
        rospy.loginfo(decision)

        # Publish on the topic '/decision'
        pub.publish(str(decision))

	# Wait 5 second
        rate.sleep()

        # Update the value
        value = updateValue(value)

# change the value
def updateValue(value):

    return (value + 1)%2

# Determine decision depending on the value
def getDecision(value):

    # We use the first algorithm (wheel encoders)
    if value == 0:

	return 1

    # We use the second algorithm (SLAM)
    else:

	return 2

# Main
if __name__ == '__main__':
    try:
        decision_system()
    except rospy.ROSInterruptException:
        pass
