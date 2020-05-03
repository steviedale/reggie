from wheel_controller import WheelController
import time


class SimpleWheelController(WheelController):

    def __init__(self):
        super().__init__()

    def timed_forward(duration=1.0, rate=0.0):
        self.forward(rate)
        time.sleep(duration)
        self.stop()

    def timed_backward(duration=1.0, rate=0.0):
        self.backward(rate)
        time.sleep(duration)
        self.stop()

    def timed_turn_right(duration=1.0, rate=0.0):
        self.turn_right(rate)
        time.sleep(duration)
        self.stop()

    def timed_turn_left(duration=1.0, rate=0.0):
        self.turn_left(rate)
        time.sleep(duration)
        self.stop()
