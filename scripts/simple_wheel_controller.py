from wheel_controller import WheelController
import time


class SimpleWheelController(WheelController):

    def __init__(self):
        WheelController.__init__(self)

    def timed_forward(self, duration=1.0, rate=0.3):
        self.forward(rate)
        time.sleep(duration)
        self.stop()

    def timed_backward(self, duration=1.0, rate=0.3):
        self.backward(rate)
        time.sleep(duration)
        self.stop()

    def timed_right_turn(self, duration=1.0, rate=0.3):
        self.turn_right(rate)
        time.sleep(duration)
        self.stop()

    def timed_left_turn(self, duration=1.0, rate=0.3):
        self.turn_left(rate)
        time.sleep(duration)
        self.stop()
