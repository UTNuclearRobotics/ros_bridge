#!/usr/bin/env python

import rospy
from ros1_custom_msgs.msg import MyMessage

def publisher():
    rospy.init_node('custom_message_publisher', anonymous=True)
    pub = rospy.Publisher('custom_topic', MyMessage, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        msg = MyMessage()
        msg.data = "Hello, ROS!"
        msg.id = 42

        rospy.loginfo(f"Publishing: {msg}")
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
