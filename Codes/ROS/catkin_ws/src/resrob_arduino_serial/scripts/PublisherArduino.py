#!/usr/bin/env python3

import rospy
#since we are sending int messages
from std_msgs.msg import Int32

nodeName='messagepublisher'

#topic name
topicName='information'

rospy.init_node(nodeName, anonymous=True)

publisher1=rospy.Publisher(topicName, Int32, queue_size=5)

ratePublisher=rospy.Rate(1)
intMessage=0
#hsjfdbjsdfbk.dsfnl.sfdjl.sdf

while not rospy.is_shutdown():
    rospy.loginfo(intMessage)
    intMessage = int(input("Enter the command : "))
    publisher1.publish(intMessage)
    ratePublisher.sleep()


















#     #!/usr/bin/env python3

# import rospy
# #since we are sending int messages
# from std_msgs.msg import Int32

# nodeName='messagepublisher'

# #topic name
# topicName='information'

# rospy.init_node(nodeName, anonymous=True)

# publisher1=rospy.Publisher(topicName, Int32, queue_size=5)

# ratePublisher=rospy.Rate(1)

# intMessage=1

# while not rospy.is_shutdown():
#     rospy.loginfo(intMessage)
#     publisher1.publish(intMessage)
#     intMessage=intMessage+1
#     ratePublisher.sleep()