#!/usr/bin/env python

import rospy
from reggie_wheel_controller.srv import WheelCommand
from sensor_msgs.msg import Joy


UP_DOWN_INDEX = 7
LEFT_RIGHT_INDEX = 6
X_INDEX = 2

class JoyToReggie:

    def __init__(self):
        self.busy = False
        rospy.init_node('joy_to_reggie', anonymous=True)
        rospy.Subscriber('joy', Joy, self.callback)

    def callback(self, joy_msg):
        print("joy msg received")
        if self.busy:
            return
        self.busy = True

        # X button is NOT down, short movement
        if joy_msg.buttons[X_INDEX] == 0:
            long_move = False
        # X button IS down, long movement
        else:
            long_move = True

        if joy_msg.axes[UP_DOWN_INDEX] == 1.0:
            command = 0
        elif joy_msg.axes[UP_DOWN_INDEX] == -1.0:
            command = 1
        elif joy_msg.axes[LEFT_RIGHT_INDEX] == 1.0:
            command = 2
        elif joy_msg.axes[LEFT_RIGHT_INDEX] == -1.0:
            command = 3
        else:
            self.busy = False
            print('thowing joy msg out, no command')
            return

        rospy.wait_for_service('simple_wheel_command')

        try:
            simple_wheel_command = rospy.ServiceProxy('simple_wheel_command', WheelCommand)
            response = simple_wheel_command(command, long_move)
        except rospy.ServiceException as e:
            print("Service call failed: {}".format(e))

        self.busy = False


if __name__ == '__main__':
    driver = JoyToReggie()
    rospy.spin()
