import rclpy
from geometry_msgs.msg import Twist
import math

from abc import ABC, abstractmethod
from gpiozero import AngularServo

# webots motors are referred to by these names, make it so that the RP ones can be as well
# the pins I chose are based on the Raspberry Pi 2b
NAME_TO_PIN_MAP = {
    'back left shoulder motor': 26,
    'back right shoulder motor': 21,
    'front left shoulder motor': 4,
    'front right shoulder motor': 18,
    'back left knee motor': 19,
    'back right knee motor': 20,
    'front left knee motor': 17,
    'front right knee motor': 23
}

# the driver needs to interact with the motors somehow
# the interface class is an ABC with implementations for interactions with webots and RPi
class Interface(ABC):
    @abstractmethod
    def find_motor(self, name):
        pass

class Motor(ABC):
    @abstractmethod
    def set_velocity(self, velocity):
        pass
    
    @abstractmethod
    def get_velocity(self):
        pass
   
   # note: should only be used in the webots base class
    @abstractmethod
    def set_position(self, position):
        pass
    
    @abstractmethod
    def get_position(self):
        pass

class WebotsInterface(Interface):
    def __init__(self, webots_node):
        super().__init__()

        self.__robot = webots_node.robot
    
    def find_motor(self, name):
        return self.__robot.getDevice(name)

class WebotsMotor(Motor):
    def __init__(self, device):
        super().__init__()

        self.device = device
    
    def set_velocity(self, velocity):
        self.device.setVelocity(velocity) # their improper capitalization, not mine
    
    def get_velocity(self):
        return self.device.getVelocity()
    
    def set_position(self, position):
        self.device.setPosition(position)
    
    def get_position(self):
        return self.device.getPosition()

class RPInterface(Interface):
    def __init__(self):
        super().__init__()

class RPMotor(Motor):
    def __init__(self, name):
        super().__init__()

        self.device = AngularServo(NAME_TO_PIN_MAP[name])

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025

# extension of atan to all quadrants
def calculate_angle(x,y):
    if x>0:
        return (math.atan(y/x)+2*math.pi) % (2*math.pi)
    elif x<0:
        return math.atan(y/x)+math.pi
    else:
        return math.pi/2 + math.pi * (y<0)

# finds the angles for the leg given the relative coordinates of the desired position
# +x = forwards, +y = up
def find_leg_positions(x,y):
    # calculations shown here: https://www.desmos.com/calculator/p4zsulmvte
    length = math.hypot(x,y)

    #i1 on the desmos gives the solution with the knee as we want it
    knee_x = x/2 + y/length * math.sqrt(0.045*0.045-length*length/4)
    knee_y = y/2 - x/length * math.sqrt(0.045*0.045-length*length/4)

    # negated since the motors move cw, not ccw
    shoulder_angle = -calculate_angle(knee_y/knee_x)+2*math.pi
    knee_angle = -calculate_angle((y-knee_y)/(x-knee_x))+2*math.pi
    knee_angle += 3*math.pi - shoulder_angle

    return (shoulder_angle+5/4*math.pi) % (2*math.pi), (knee_angle+7/4*math.pi) % (2*math.pi)

# for moving and controlling a single leg
class Leg:
    def __init__(self, shoulder_motor, knee_motor):
        self.shoulder_motor = shoulder_motor
        self.knee_motor = knee_motor

        self.shoulder_motor.setPosition(float('inf'))
        self.shoulder_motor.setVelocity(0)

        self.knee_motor.setPosition(float('inf'))
        self.knee_motor.setVelocity(0)
    
    # temporary function
    def move_at_velocity(self, velocity):
        self.shoulder_motor.setVelocity(velocity)

# driver for the whole robot
class Driver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        # find the motors by their name in the world file
        shoulder_motor_names = [
            'back left shoulder motor',
            'back right shoulder motor',
            'front left shoulder motor',
            'front right shoulder motor'
        ]
        knee_motor_names = [
            'back left knee motor',
            'back right knee motor',
            'front left knee motor',
            'front right knee motor'
        ]
        self.legs = []
        
        # create each leg
        for i in range(4):
            self.legs.append(Leg(
                self.__robot.getDevice(shoulder_motor_names[i]),
                self.__robot.getDevice(knee_motor_names[i])
            ))
        
        # recieve messages for instructions
        self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z

        command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        for i in range(4):
            if i%2: # right
                self.legs[i].move_at_velocity(command_motor_right)
            else:
                self.legs[i].move_at_velocity(command_motor_left)