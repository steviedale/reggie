from wheel_controller import WheelController
import time


class SimpleWheelController(WheelController):

    def __init__(self):
        WheelController.__init__(self)

    def timed_forward(self, duration=1.0, rate=0.0):
        self.forward(rate)
        time.sleep(duration)
        self.stop()

    def timed_backward(self, duration=1.0, rate=0.0):
        self.backward(rate)
        time.sleep(duration)
        self.stop()

    def timed_turn_right(self, duration=1.0, rate=0.0):
        self.turn_right(rate)
        time.sleep(duration)
        self.stop()

    def timed_turn_left(self, duration=1.0, rate=0.0):
        self.turn_left(rate)
        time.sleep(duration)
        self.stop()
