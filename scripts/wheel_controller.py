import RPi.GPIO as GPIO


right_wheel_forward = 7
right_wheel_backward = 8
left_wheel_forward = 9
left_wheel_backward = 10

MIN_SPEED = 30
MAX_SPEED = 100

class WheelController:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(right_wheel_forward, GPIO.OUT)
        GPIO.setup(right_wheel_backward, GPIO.OUT)
        GPIO.setup(left_wheel_backward, GPIO.OUT)
        GPIO.setup(left_wheel_forward, GPIO.OUT)

        self.right_fwd_pwm = GPIO.PWM(right_wheel_forward, 100)
        self.right_fwd_pwm.start(0)
        self.left_fwd_pwm = GPIO.PWM(left_wheel_forward, 100)
        self.left_fwd_pwm.start(0)
        self.right_bwd_pwm = GPIO.PWM(right_wheel_backward, 100)
        self.right_bwd_pwm.start(0)
        self.left_bwd_pwm = GPIO.PWM(left_wheel_backward, 100)
        self.left_bwd_pwm.start(0)

    def forward(self, rate=1.0):
        self.set_right_wheel_forward(rate)
        self.set_left_wheel_forward(rate)

    def backward(self, rate=1.0):
        self.set_right_wheel_forward(-1*rate)
        self.set_left_wheel_forward(-1*rate)

    def turn_left(self, rate=1.0):
        self.set_right_wheel_forward(rate)
        self.set_left_wheel_forward(-1*rate)

    def turn_right(self, rate=1.0):
        self.set_right_wheel_forward(-1*rate)
        self.set_left_wheel_forward(rate)

    def stop(self):
        self.right_fwd_pwm.ChangeDutyCycle(0)
        self.left_fwd_pwm.ChangeDutyCycle(0)
        self.right_bwd_pwm.ChangeDutyCycle(0)
        self.left_bwd_pwm.ChangeDutyCycle(0)

    def set_right_wheel_velocity(self, rate):
        speed = abs(rate) * (MAX_SPEED - MIN_SPEED) + MIN_SPEED
        if rate >= 0:
            self.right_bwd_pwm.ChangeDutyCycle(0)
            self.right_fwd_pwm.ChangeDutyCycle(speed)
        else:
            self.right_fwd_pwm.ChangeDutyCycle(0)
            self.right_bwd_pwm.ChangeDutyCycle(speed)
        
    def set_left_wheel_velocity(self, rate):
        speed = abs(rate) * (MAX_SPEED - MIN_SPEED) + MIN_SPEED
        if rate >= 0:
            self.left_bwd_pwm.ChangeDutyCycle(0)
            self.left_fwd_pwm.ChangeDutyCycle(speed)
        else:
            self.left_fwd_pwm.ChangeDutyCycle(0)
            self.left_bwd_pwm.ChangeDutyCycle(speed)

    def __del__(self):
        GPIO.cleanup()
