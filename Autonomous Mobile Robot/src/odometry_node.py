#!/usr/bin/env python3 
import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import Pose2D 
from std_msgs.msg import Float32, Bool
from rclpy import qos 
import numpy as np 

#This class will compute the pose of the robot from the encoder readings. 
# This node subscribes to the /VelocityEncR and /VelocityEncL topics 
# This node publishes the pose of the robot to the /pose topic.  

class Odometry(Node):  

    def __init__(self):  
        super().__init__('odometry_node') 

        ###########  INIT PUBLISHERS ################ 
        self.pub_pose = self.create_publisher(Pose2D, 'pose', 10)  
        self.pub_reset = self.create_publisher(Bool, 'reset_origin', 10)  

        ############## SUBSCRIBERS ##################  
        '''
        The quality of service (QoS) settings are used to define the reliability and ordering of messages.
        In this case, we are using the qos_profile_sensor_data profile, which is suitable for sensor data. 
        '''
        self.create_subscription(Float32, "VelocityEncR",  self.wr_cb, qos.qos_profile_sensor_data)  # Best effort
        self.create_subscription(Float32, "VelocityEncL",  self.wl_cb, qos.qos_profile_sensor_data)  
        self.create_subscription(Bool, "line_state",  self.reset_cb, 10)  

        ############ ROBOT CONSTANTS ################  
        self.r = 0.05    #wheel radius for our simulated robot[m] 
        self.L = 0.19    #wheel separation for our simulated robot [m] 
        self.wl = 0.0    #Left wheel speed [rad/s] 
        self.wr = 0.0    #Right wheel speed [rad/s] 
        self.x = 0.0     #Robot position in x-axis [m] 
        self.y = 0.0     #Robot position in y-axis [m] 
        self.theta = 0.0 #Robot orientation [rad] 
        self.robot_pose = Pose2D() 
        self.prev_time_nano = self.get_clock().now().nanoseconds # Previous time in nanoseconds
        timer_period = 0.01
        self.create_timer(timer_period, self.main_timer_cb) 
        self.get_logger().info("Node initialized!!")       

    def main_timer_cb(self): 

        v, w = self.get_robot_velocity(self.wl, self.wr) #Get the robot's speed
        self.update_robot_pose(v,w) # Get the pose of the robot
        print("xr: " + str(self.x))
        print("yr: " + str(self.y))
        print("theta_r: " + str(self.theta))

        # Publish the robot pose
        self.pub_pose.publish(self.robot_pose)

    def wl_cb(self, wl):  
        ## This function receives the left wheel speed from the encoders  
        self.wl = wl.data 

    def wr_cb(self, wr):  
        ## This function receives the right wheel speed from the encoders 
        self.wr = wr.data 

    def reset_cb(self, msg):
        #Reset robot position
        if msg.data == False:
            self.x = 0.0 #Robot position in x-axis [m] 
            self.y = 0.0 #Robot position in y-axis [m] 
            self.theta = 0.0 #Robot orientation [rad] 
            self.pub_reset.publish(Bool(data=True))
    

    def get_robot_velocity(self, wl, wr):
        v = self.r * (wl + wr) / 2.0 # Compute the robot linear velocity [m/s]
        w = self.r * (wr - wl) / self.L # Compute the robot angular velocity [rad/s]
        print("v: " + str(v))
        print("w: " + str(w))
        return v, w
    
    def update_robot_pose(self, v, w):
        dt = (self.get_clock().now().nanoseconds - self.prev_time_nano) * 10**-9 # Delta t in seconds
        self.x = self.x + v * np.cos(self.theta) * dt
        self.y = self.y + v * np.sin(self.theta) * dt
        self.theta = self.theta + w * dt
        # Crop this angle to -pi to pi
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))

        # Filled the Pose2D() message to publish it
        self.robot_pose.x = self.x
        self.robot_pose.y = self.y
        self.robot_pose.theta = self.theta

        self.prev_time_nano = self.get_clock().now().nanoseconds

def main(args=None): 
    rclpy.init(args=args) 
    my_node=Odometry() 
    rclpy.spin(my_node) 
    my_node.destroy_node() 
    rclpy.shutdown() 

if __name__ == '__main__': 
    main() 