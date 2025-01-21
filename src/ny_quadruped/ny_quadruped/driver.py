import rclpy
from geometry_msgs.msg import Twist
import math
import time

from abc import ABC, abstractmethod
# gpiozero breaks if youre not actually on a rpi
# from gpiozero import AngularServo

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

# chosen arbitrarily, would be based on the initial positions in a physical implementation
SERVO_MIN_ANGLE = 0
SERVO_MAX_ANGLE = 180

# radians/second, based on the SG90 servo motor assuming no load
SERVO_MAX_VELOCITY = 10

# length of a single step in seconds
NORMAL_STEP_DURATION = 0.15

# distance is in meters so every distance looks small, but thats what webots uses
STEP_LENGTH = 0.02
LEG_LENGTH = 0.045

# the driver needs to interact with the motors somehow
# the interface class is an ABC with implementations for interactions with webots and RPi
class Interface(ABC):
    @abstractmethod
    def find_motor(self, name):
        pass

class Motor(ABC):
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
        return WebotsMotor(self.__robot.getDevice(name))

class WebotsMotor(Motor):
    def __init__(self, device):
        super().__init__()

        self.device = device
        self.position = 0 # getPosition doesn't exist even though getVelocity does???
        # this just specifies the max velocity since we are using position control
        self.device.setVelocity(SERVO_MAX_VELOCITY) # their improper capitalization, library is just a port of a cpp one
    
    def set_position(self, position):
        self.device.setPosition(position)
        self.position = position
    
    def get_position(self):
        return self.position

class RPInterface(Interface):
    def __init__(self):
        super().__init__()
    
    def find_motor(self, name):
        return RPMotor(name)

class RPMotor(Motor):
    def __init__(self, name):
        super().__init__()

        self.device = AngularServo(
            NAME_TO_PIN_MAP[name],
            SERVO_MIN_ANGLE,
            SERVO_MAX_ANGLE
        )
    
    def set_position(self, position):
        # gpiozero uses degrees instead of radians for some reason
        self.device.angle = position * 180/math.pi
    
    def get_position(self):
        return self.device.angle * math.pi/180

# parametric equation describing the step
def step_function(t, start, end):
    t *= math.pi

    this_step_length  = end-start

    x_coord = -this_step_length/2*(math.cos(t)-1) + start
    # adjusted by leg_length*sqrt(2) to adjust for the initial height
    y_coord = abs(this_step_length)/6*math.sin(t)-LEG_LENGTH*math.sqrt(2)
    return x_coord, y_coord

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
    knee_x = x/2 + y/length * math.sqrt(LEG_LENGTH*LEG_LENGTH-length*length/4)
    knee_y = y/2 - x/length * math.sqrt(LEG_LENGTH*LEG_LENGTH-length*length/4)

    # negated since the motors move cw, not ccw
    shoulder_angle = -calculate_angle(knee_x, knee_y)+2*math.pi
    knee_angle = -calculate_angle(x-knee_x, y-knee_y)+2*math.pi
    knee_angle += 5*math.pi - shoulder_angle

    # adjustments for the initial angles of the motor
    adjusted_shoulder = (shoulder_angle+5/4*math.pi) % (2*math.pi)
    adjusted_knee = (knee_angle+3/2*math.pi) % (2*math.pi)

    return adjusted_shoulder, adjusted_knee

# for moving and controlling a single leg
class Leg:
    def __init__(self, shoulder_motor, knee_motor):
        self.shoulder_motor = shoulder_motor
        self.knee_motor = knee_motor

    # puts the leg at a specific angle
    # assumes that the next timestep will be the same as this one
    def go_to(self, shoulder_angle, knee_angle):
        #! note: caution would need to be taken using this with real servos
        shoulder_in_range = (shoulder_angle + 3/4*math.pi) % math.pi - 3/4*math.pi
        knee_in_range = (knee_angle + 1/4*math.pi) % math.pi - 1/4*math.pi
        self.shoulder_motor.set_position(shoulder_in_range)
        self.knee_motor.set_position(knee_in_range)
    
    # returns how far in front of the shoulder the foot is
    def horizontal_foot_distance(self):
        adjusted_shoulder = self.shoulder_motor.get_position()
        adjusted_knee = self.knee_motor.get_position()

        # get the adjusted angles
        shoulder_angle = -adjusted_shoulder+5/4*math.pi
        knee_angle = adjusted_knee-3/2*math.pi-(math.pi+shoulder_angle)

        return LEG_LENGTH*(math.cos(shoulder_angle)+math.cos(knee_angle))

# driver for the whole robot
class Driver:
    def init(self, webots_node, properties):
        self.interface = WebotsInterface(webots_node)

        # find the motors by their name in the world file
        # ordered by where in the step they are
        shoulder_motor_names = [
            'front right shoulder motor',
            'back left shoulder motor',
            'front left shoulder motor',
            'back right shoulder motor'
        ]
        knee_motor_names = [
            'front right knee motor',
            'back left knee motor',
            'front left knee motor',
            'back right knee motor'
        ]
        
        # create each leg
        self.legs = []
        for i in range(4):
            self.legs.append(Leg(
                self.interface.find_motor(shoulder_motor_names[i]),
                self.interface.find_motor(knee_motor_names[i])
            ))
        
        # recieve messages for instructions
        self.__target_twist = Twist()

        # initialization stuff
        rclpy.init(args=None)
        self.__node = rclpy.create_node('driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)

        self.last_time = time.time()
        self.timestep = 0

        self.last_step_started = time.time()
        self.current_leg = 0 # kinda redundant but makes things more clear
        self.step_num = 1
        self.curr_step_duration = NORMAL_STEP_DURATION

        self.horizontals_at_step_start = [0,0,0,0]

        self.direction = "forwards"

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    # one timestep. not to be confused with the robot stepping
    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        # again, theres probably an API but I couldn't find anything
        new_time = time.time()
        self.timestep = new_time - self.last_time
        self.last_time = new_time

        # update the step if the last one ended
        if new_time - self.last_step_started >= self.curr_step_duration:
            self.update_step(new_time)

        if self.direction == "stopped":
            return

        # https://www.researchgate.net/figure/Successive-gait-pattern-of-a-crawl-gait_fig2_332471436
        # the above link shows the center of mass and leg movements for the gait implemented here

        # what part of the step we're on
        step_fraction = (new_time - self.last_step_started) / self.curr_step_duration
        assert(step_fraction<=1)

        if self.step_num == 1:
            self.first_step(step_fraction)
            return
        elif self.step_num == 2:
            self.second_step(step_fraction)
            return

        forwards = (self.direction == "forwards")

        for i in range(4):
            if i == self.current_leg:
                desired_x, desired_y = step_function(
                    step_fraction,
                    self.horizontals_at_step_start[i],
                    self.horizontals_at_step_start[i] + STEP_LENGTH*(2*forwards-1)
                )
            else:
                # assume that friction is great enough that we can move the shoulder forward
                # by telling it to move the foot backwards along the ground
                # we can't really not assume that - every walking animal relies on it
                desired_x = self.horizontals_at_step_start[i] - STEP_LENGTH/3 * step_fraction * (2*forwards-1)
                desired_y = -LEG_LENGTH*math.sqrt(2)

            shoulder, knee = find_leg_positions(desired_x, desired_y)
            self.legs[i].go_to(shoulder,knee)

    # switch to next step
    def update_step(self, new_time):
        self.last_step_started = new_time

        if self.step_num > 2: 
            forward_speed = -self.__target_twist.linear.x
        else:
            forward_speed = 1 # handle special steps first
        prev_direction = self.direction # take the same step when switching directions

        if forward_speed == 0:
            self.direction = "stopped"
            # make sure we can get the cmd quickly when it changes
            self.curr_step_duration = self.timestep
            return
        elif forward_speed > 0:
            self.direction = "forwards"
        else:
            self.direction = "backwards"

        self.step_num += 1

        self.curr_step_duration = abs(NORMAL_STEP_DURATION/forward_speed)

        if self.step_num > 3:
            if prev_direction == self.direction:
                forwards = (self.direction == "forwards")
                self.current_leg = (self.current_leg+2*forwards-1)%4

        for i in range(4):
            self.horizontals_at_step_start[i] = self.legs[i].horizontal_foot_distance()
        
    # not proud of this repetition but whatever
    def first_step(self, step_fraction):
        desired_x, desired_y = step_function(
            step_fraction,
            0,
            -2/3 * STEP_LENGTH
        )

        shoulder, knee = find_leg_positions(desired_x, desired_y)
        self.legs[0].go_to(shoulder, knee)
    
    def second_step(self, step_fraction):
        desired_x, desired_y = step_function(
            step_fraction,
            0,
            2/3 * STEP_LENGTH
        )

        shoulder, knee = find_leg_positions(desired_x, desired_y)
        self.legs[0].go_to(shoulder, knee)