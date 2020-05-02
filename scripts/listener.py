#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from reggie_wheel_controller.msg import WheelVelocity
from wheel_controller import WheelController


class WheelVelocityListener:

    def __init__(self):
        rospy.init_node('wheel_controller', anonymous=False)
        rospy.Subscriber('wheel_velocity', WheelVelocity, self.callback)
        self.wheel_controller = WheelController()

    def callback(self, data):
        rospy.loginfo('Left Vel: {}'.format(data.left_velocity))
        rospy.loginfo('Right Vel: {}'.format(data.right_velocity))

        self.wheel_controller.set_left_wheel_velocity(data.left_velocity)
        self.wheel_controller.set_right_wheel_velocity(data.right_velocity)


if __name__ == '__main__':
    listener = WheelVelocityListener()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
