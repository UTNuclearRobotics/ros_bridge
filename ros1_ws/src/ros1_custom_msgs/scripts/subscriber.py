import rospy
from ros1_custom_msgs.msg import MyMessage

#!/usr/bin/env python


def callback(data):
    rospy.loginfo("I heard: %s", data)

def listener():
    rospy.init_node('custom_message_subscriber', anonymous=True)
    rospy.Subscriber('custom_topic', MyMessage, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()