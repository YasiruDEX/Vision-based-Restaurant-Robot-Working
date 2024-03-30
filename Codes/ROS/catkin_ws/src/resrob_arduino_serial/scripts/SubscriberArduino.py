#!/usr/bin/env python3

import rospy

from std_msgs.msg import Int32

nodeName='messagesubs'

topicName='info_back'

def callBackFunction(message):
    print("from arduino we received %d" %message.data)


rospy.init_node(nodeName, anonymous=True)

rospy.Subscriber(topicName, Int32, callBackFunction)

rospy.spin()