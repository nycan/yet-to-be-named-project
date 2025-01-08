import rclpy
from geometry_msgs.msg import Twist

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025

# https://www.desmos.com/calculator/g42ilolx0t

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