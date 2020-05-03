#!/usr/bin/env python

import rospy
from reggie_wheel_controller.srv import WheelCommand, WheelCommandResponse
from simple_wheel_controller import SimpleWheelController


SHORT_TRANSLATION = 0.2
LONG_TRANSLATION = 0.5
SHORT_ROTATION = 0.1
LONG_ROTATION = 0.25

class SimpleWheelCommandListener:

    def __init__(self):
        rospy.init_node('wheel_controller', anonymous=False)
        self.simple_wheel_controller = SimpleWheelController()
        self.service = rospy.Service('simple_wheel_command', WheelCommand, self.callback)
        self.busy = False

    def callback(self, data):
        if self.busy:
            rospy.loginfo('simple_wheel_command service busy! Droping request.')
            return WheelCommandResponse(False)

        rospy.loginfo('Command: {}'.format(data.command))
        rospy.loginfo('Long Move? {}'.format(data.long_move))
        
        if data.command == 0:
            self.simple_wheel_controller.timed_forward(duration=LONG_TRANSLATION if data.long_move else SHORT_TRANSLATION, rate=0.3) 
        elif data.command == 1:
            self.simple_wheel_controller.timed_backward(duration=LONG_TRANSLATION if data.long_move else SHORT_TRANSLATION, rate=0.3) 
        elif data.command == 2:
            self.simple_wheel_controller.timed_left_turn(duration=LONG_ROTATION if data.long_move else SHORT_ROTATION, rate=0.3) 
        elif data.command == 3:
            self.simple_wheel_controller.timed_right_turn(duration=LONG_ROTATION if data.long_move else SHORT_ROTATION, rate=0.3) 

        return WheelCommandResponse(True)


if __name__ == '__main__':
    listener = SimpleWheelCommandListener()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
