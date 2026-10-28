import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class PIDRegulator:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0.0
        self.integral_sum = 0.0

    def calculate_output(self, target, current, dt):
        error = target - current
        self.integral_sum += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.kp * error + self.ki * self.integral_sum + self.kd * derivative
        self.previous_error = error
        return output

class WallFollowingRobot:  
    def __init__(self):
        rospy.init_node('wall_following_robot')

        self.laser_subscriber = rospy.Subscriber('scan', LaserScan, self.process_laser_scan, queue_size=10)
        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.obstacle_distance_threshold = 0.55 
        self.forward_speed = 0.2
        self.turning_speed = 0.45
        
        self.pid_regulator = PIDRegulator(kp=1.0, ki=0.0, kd=0.1)
        self.previous_time = rospy.Time.now()

    def process_laser_scan(self, data):
        movement_command = Twist()
        distances = np.array(data.ranges) 
        distances = distances - 0.180

        current_time = rospy.Time.now()
        dt = (current_time - self.previous_time).to_sec()
        self.previous_time = current_time

        if all(distances[0:20] > self.obstacle_distance_threshold) and all(distances[-20:] > self.obstacle_distance_threshold):
            movement_command.linear.x = self.forward_speed
            movement_command.angular.z = 0.0
        else:
            closest_front_distance = min(min(distances[0:20]), min(distances[-20:]))
            distance_error = self.obstacle_distance_threshold - closest_front_distance

            angular_correction = self.pid_regulator.calculate_output(0, distance_error, dt)

            movement_command.linear.x = self.forward_speed / 2.0
            movement_command.angular.z = angular_correction

        self.velocity_publisher.publish(movement_command)

if __name__ == '__main__':
    wall_following_robot = WallFollowingRobot() 
    rospy.spin()
