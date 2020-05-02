import RPi.GPIO as GPIO
import time

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
		self.stop()
		speed = rate * (MAX_SPEED - MIN_SPEED) + MIN_SPEED
		self.left_fwd_pwm.ChangeDutyCycle(speed) 
		self.right_fwd_pwm.ChangeDutyCycle(speed)

	def backward(self, rate=1.0):
		self.stop()
		speed = rate * (MAX_SPEED - MIN_SPEED) + MIN_SPEED
		self.right_bwd_pwm.ChangeDutyCycle(speed)
		self.left_bwd_pwm.ChangeDutyCycle(speed) 

	def turn_left(self, rate=1.0):
		self.stop()
		speed = rate * (MAX_SPEED - MIN_SPEED) + MIN_SPEED
		self.right_fwd_pwm.ChangeDutyCycle(speed)
		self.left_bwd_pwm.ChangeDutyCycle(speed) 

	def turn_right(self, rate=1.0):
		self.stop()
		speed = rate * (MAX_SPEED - MIN_SPEED) + MIN_SPEED
		self.right_bwd_pwm.ChangeDutyCycle(speed)
		self.left_fwd_pwm.ChangeDutyCycle(speed) 

	def stop(self):
		self.right_fwd_pwm.ChangeDutyCycle(0)
		self.left_fwd_pwm.ChangeDutyCycle(0)
		self.right_bwd_pwm.ChangeDutyCycle(0)
		self.left_bwd_pwm.ChangeDutyCycle(0)

	def __del__(self):
		GPIO.cleanup()

if __name__=='__main__':
    controller = WheelController()
    controller.forward()
    time.sleep(1.0)
    controller.stop()
