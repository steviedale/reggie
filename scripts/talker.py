#!/usr/bin/env python

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from reggie_wheel_controller.msg import WheelVelocity

def talker():
    pub = rospy.Publisher('wheel_velocity', WheelVelocity, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = WheelVelocity()
        msg.right_velocity = 0.1
        msg.left_velocity = 1.0
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
