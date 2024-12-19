#PROGRAM TASK: robot will navigate through a series of obstacles and beep when
# it comes across a certain color obstacle within a certain distance.
#Run colorHSV.py first: ros2 run yahboomcar_astra colorHSV
#Run roboTask: cd yahboomcar_ros2_ws/yahboomcar_ws/src
# colcon build --packages-select pkg_roboTask_py, ros2 run pkg_roboTask_py roboTask
#Referenced colorTracker.py, laser_Avoidance.py, laser_Warning.py
#Potential upgrade: user can input a color for robot to detect

#ROS Library
import rclpy
import os
from rclpy.node import Node
from geometry_msgs.msg import Twist #for velocity publisher
from sensor_msgs.msg import LaserScan, Image #for the laser subscription
from std_msgs.msg import Bool,UInt16 #for the buzzer publisher
import cv2
from cv_bridge import CvBridge  #to convert ROS Image messages to OpenCV images
import numpy as np

#Common Library
import math
from yahboomcar_laser.common import * #something with PID (Proportional-Integral-Derivative)
print("import done")
RAD2DEG = 180 / math.pi


class obstacleMaze(Node):
    def __init__(self,name):
        super().__init__(name)
        #create a subscription
        self.sub_camera = self.create_subscription(Image, "/image_raw", self.process_camera_image, 2)
        self.sub_laser = self.create_subscription(LaserScan,"/scan", self.registerScan, 1)
        self.sub_JoyState = self.create_subscription(Bool,'/JoyState', self.JoyStateCallback, 1)
        
        #create a publisher
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 1)
        self.pub_Buzzer = self.create_publisher(UInt16, '/beep', 1)

        #declare parameters
        self.declare_parameter("ResponseDist", 0.3)
        self.ResponseDist = self.get_parameter('ResponseDist').get_parameter_value().double_value
        self.declare_parameter("Switch",False)
        self.Switch = self.get_parameter('Switch').get_parameter_value().bool_value
        self.declare_parameter("linear", 0.3)
        self.linear = self.get_parameter('linear').get_parameter_value().double_value
        self.declare_parameter("angular", 1.0)
        self.angular = self.get_parameter('angular').get_parameter_value().double_value
        self.declare_parameter("LaserAngle", 45.0)
        self.LaserAngle = self.get_parameter('LaserAngle').get_parameter_value().double_value
        self.declare_parameter("TargetColor", "white") #default color to detect

        self.Right_warning = 0
        self.Left_warning = 0
        self.front_warning = 0
        self.Joy_active = False
        self.Moving = False
        self.ros_ctrl = SinglePID()
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.01,self.on_timer)



    def on_timer(self):
        self.Switch = self.get_parameter('Switch').get_parameter_value().bool_value
        self.ResponseDist = self.get_parameter('ResponseDist').get_parameter_value().double_value
        self.linear = self.get_parameter('linear').get_parameter_value().double_value
        self.angular = self.get_parameter('angular').get_parameter_value().double_value
        self.LaserAngle = self.get_parameter('LaserAngle').get_parameter_value().double_value



    def JoyStateCallback(self, msg):
        if not isinstance(msg, Bool): return
        self.Joy_active = msg.data



    def registerScan(self, scan_data): #logic influenced laser_Avoidance.py
        if not isinstance(scan_data, LaserScan): 
            return
        ranges = np.array(scan_data.ranges)
        self.Right_warning = 0
        self.Left_warning = 0
        self.front_warning = 0

        for i in range(len(ranges)):
            angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG
            if angle > 180: angle -= 360
            if 20 < angle < self.LaserAngle:
                if ranges[i] < self.ResponseDist * 1.5:
                    self.Left_warning += 1
            if -self.LaserAngle < angle < -20:
                if ranges[i] < self.ResponseDist * 1.5:
                    self.Right_warning += 1
            if abs(angle) <= 20:
                if ranges[i] <= self.ResponseDist * 1.5: 
                    self.front_warning += 1

        if self.Joy_active or self.Switch:
            if self.Moving:
                self.pub_vel.publish(Twist())
                self.Moving = not self.Moving
            return

        self.Moving = True
        twist = Twist()
        if self.front_warning > 10:
            if self.Left_warning > 10 and self.Right_warning > 10:
                print('Obstacle ahead, turning right')
                twist.angular.z = -self.angular
            elif self.Right_warning > 10:
                print('Obstacle ahead, turning left')
                twist.angular.z = self.angular
            else:
                print('Obstacle ahead, turning right')
                twist.angular.z = -self.angular
        elif self.Left_warning > 10:
            print('Obstacle on the left, turning right')
            twist.angular.z = -self.angular
        elif self.Right_warning > 10:
            print('Obstacle on the right, turning left')
            twist.angular.z = self.angular
        else:
            print('No obstacles, moving forward')
            twist.linear.x = self.linear

        self.pub_vel.publish(twist)
        
        #Test for beep!
        '''minDist = 0.2 #should beep
        #minDist = 0.4 #shouldn't beep

        print("minDist: ",minDist)
        #print("minDistID: ",minDistID)
        if minDist <= self.ResponseDist:
            print("BEEP")
            beep = UInt16()
            beep.data = 1
            self.pub_Buzzer.publish(beep)

        else:
            print("no obstacles")
            beep = UInt16()
            beep.data = 0
            self.pub_Buzzer.publish(beep)'''



    def process_camera_image(self, image_data):
        print('Processing camera image...')
        if not isinstance(image_data, Image):
            return
        try:
            #convert ROS Image message to OpenCV image
            np_img = self.bridge.imgmsg_to_cv2(image_data, desired_encoding="bgr8")
            hsv_img = cv2.cvtColor(np_img, cv2.COLOR_BGR2HSV)
            hsv_img = cv2.GaussianBlur(hsv_img, (5, 5), 0) #blur to reduce noise around

            #two HSV ranges for red (lower and upper)
            lower_red1 = np.array([0, 120, 70]) #lower red range (0° to 10° hue)
            upper_red1 = np.array([10, 255, 255])
        
            lower_red2 = np.array([170, 120, 70]) #upper red range (170° to 180° hue)
            upper_red2 = np.array([180, 255, 255])

            #create masks for both red ranges
            mask1 = cv2.inRange(hsv_img, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv_img, lower_red2, upper_red2)
            mask = cv2.bitwise_or(mask1, mask2) #combine masks for final red mask

            #check if red is detected in image
            red_pixels = cv2.countNonZero(mask)
            if red_pixels > 500: #adjustable threshold
                print("Red detected!")
                beep = UInt16()
                beep.data = 1 #trigger beep
                self.pub_Buzzer.publish(beep)
            else:
                beep = UInt16()
                beep.data = 0 #turn off beep if no red detected
                self.pub_Buzzer.publish(beep) 

        except Exception as e:
            print(f"Error processing camera image: {e}")



    # keyboard command to stop car: ctrl + c
    def exit_pro(self):
        cmd1 = "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "
        cmd2 = '''"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"'''
        cmd = cmd1 + cmd2
        os.system(cmd)
        cmd3 = "ros2 topic pub --once /beep std_msgs/msg/UInt16 "
        cmd4 = '''"data: 0"'''
        cmd5 = cmd3 + cmd4
        os.system(cmd5)



def main():
    rclpy.init()
    obstacle_maze = obstacleMaze("obstacle_maze")
    print("3...2...1...Begin!")
    try:
        rclpy.spin(obstacle_maze)
    # if ctrl + c, shutdown program
    except KeyboardInterrupt:
        obstacle_maze.exit_pro()
        obstacle_maze.destroy_node()
        rclpy.shutdown()