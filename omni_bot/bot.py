#import board
import busio
import adafruit_pca9685
from time import sleep
import math

# ROS imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Constants
MAX_SPEED = 1
REAR = 2
LEFT = 1
RIGHT = 0
REAR_MIN_PULSE = 1085
REAR_MAX_PULSE = 2100
LEFT_MIN_PULSE = 1030
LEFT_MAX_PULSE = 2060
RIGHT_MIN_PULSE = 1060
RIGHT_MAX_PULSE = 2020

i2c = busio.I2C((4,3), (4,2))
pca = adafruit_pca9685.PCA9685(i2c)
pca.frequency = 50


# mapping function
def mapping(value, old_min=0, old_max=20000, new_min=0x0000, new_max=0xffff):
    if old_min == old_max:
        return new_min
    old_range = old_max - old_min
    new_range = new_max - new_min
    return int((((value - old_min) * new_range) / old_range) + new_min)


# Clamping function
def clamp(n, nmin, nmax):
    return max(min(n, nmax), nmin)


# Generate the angle triangle
def angle(a, p):
    theta = a - p
    return theta


# Determine the velocity of the motor
def get_speed(theta):
    vel = MAX_SPEED * math.sin(math.radians(theta))
    return vel


class Motor:
    def __init__(self, x, _str, min_pulse, max_pulse):
        self.motor = x
        self.name = _str
        self.min_pulse = min_pulse
        self.max_pulse = max_pulse

    def set_throttle(self, velocity):
        pulse_width = mapping(velocity, -1, 1, self.min_pulse, self.max_pulse)
        print(f"{self.name} motor is set to {pulse_width}")
        self.motor.duty_cycle = mapping(pulse_width)

    def set_pwm(self, pwm):
        print(f"{self.name} motor is set to {pwm}")
        self.motor.duty_cycle = pwm


class Robot:
    def __init__(self, a, b, c, alpha_r, alpha_l, alpha_re):
        self.right_motor = a
        self.left_motor = b
        self.rear_motor = c
        self.alpha_right = alpha_r
        self.alpha_left = alpha_l
        self.alpha_rear = alpha_re
        self.rotation = 0

    def move_angle(self, Phi, speed):
        theta1 = angle(self.alpha_right, Phi)
        theta2 = angle(self.alpha_left, Phi)
        theta3 = angle(self.alpha_rear, Phi)
        vel1 = get_speed(theta1) + self.rotation
        vel2 = get_speed(theta2) + self.rotation
        vel3 = get_speed(theta3) + self.rotation
        clamp(vel1, -1, 1)
        clamp(vel2, -1, 1)
        clamp(vel3, -1, 1)
        self.right_motor.set_throttle(vel1 * speed)
        self.left_motor.set_throttle(vel2 * speed)
        self.rear_motor.set_throttle(vel3 * speed)

    def halt(self):
        self.rear_motor.set_pwm(0)
        self.left_motor.set_pwm(0)
        self.right_motor.set_pwm(0)

    def set_rotation_offset(self, val):
        self.rotation = val
        print(f"Rotation offset: {self.rotation}")


class BotNode(Node):
    def __init__(self, bot):
        super().__init__('bot_node')
        self.subscriber = self.create_subscriber(Twist, '/cmd_vel', self.callback, 10)
        self.robot = bot

    def callback(self, twist):
        x_vel = twist.linear.x
        v_vel = twist.linear.y
        vel = math.sqrt(x_vel * x_vel + y_vel * y_vel)  # Velocity Magnitude
        vel = clamp(vel, 0, 1)
        w = clamp(twist.angular.z, -1, 1)
        theta = math.tan(y_vel / x_vel)
        if x_vel != 0 or y_vel != 0:
            self.robot.set_rotation(w)
            self.robot.move_angle(theta, vel)
        else:
            self.robot_halt()


def main(args=None):
    alpha = [30, 150, 270]
    rear_motor = Motor(pca.channels[REAR], "Rear", REAR_MIN_PULSE, REAR_MAX_PULSE)
    left_motor = Motor(pca.channels[LEFT], "Left", LEFT_MIN_PULSE, LEFT_MAX_PULSE)
    right_motor = Motor(pca.channels[RIGHT], "Right", RIGHT_MIN_PULSE, RIGHT_MAX_PULSE)

    # Loop between different movement directions
    robot = Robot(right_motor, left_motor, rear_motor, alpha[0], alpha[1], alpha[2])
    rclpy.init(args=args)
    bot_node = BotNode(robot)
    rclpy.spin(bot_node)
    bot_node.destroy_node()
    rclpy.shutdown()

    # while True:
    #     key = input("Key: ")
    #     if key == 'a':
    #         robot.move_angle(180, MAX_SPEED)
    #     elif key == 's':
    #         robot.move_angle(270, MAX_SPEED)
    #     elif key == 'd':
    #         robot.move_angle(0, MAX_SPEED)
    #     elif key == 'w':
    #         robot.move_angle(90, MAX_SPEED)
    #     elif key == 'q':
    #         robot.set_rotation_offset(-0.1)
    #
    #     elif key == 'e':
    #         robot.set_rotation_offset(0.1)
    #     else:
    #         robot.halt()
