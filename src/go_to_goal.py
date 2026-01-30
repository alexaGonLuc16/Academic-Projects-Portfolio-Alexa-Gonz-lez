#!/usr/bin/env python3 
import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import Pose2D, Twist 
from std_msgs.msg import Bool, Int32, Float32, Float32MultiArray
import numpy as np 
import signal # To handle Ctrl+C 
import sys # To exit the program 

#This class will compute the robot speed to go to the goal 
#It will use the pose from the odometry node and the goal from the path generator 
#The robot will stop when it is close to the goal 

class GoToGoal(Node):  

    def __init__(self):  
        super().__init__('go_to_goal')

        ###########  INIT PUBLISHERS ################ 
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10) # Puzzlebot
        self.pub_odo_finish = self.create_publisher(Bool, 'odometry_finish', 10)# Controller
        # Handle shutdown gracefully (stop the robot) 
        signal.signal(signal.SIGINT, self.shutdown_function) 

        ############## INIT SUBSCRIBERS ##################  
        self.create_subscription(Int32, "color", self.color_cb, 10)  #traffic_light
        self.create_subscription(Int32, "signal", self.signal_cb, 10)  #traffic_signal
        self.create_subscription(Float32, "position", self.position_cb, 10)  # controller - line follower
        self.create_subscription(Bool, "line_state", self.line_state_cb, 10) # controller
        self.create_subscription(Float32MultiArray, "goal", self.goal_cb, 10) # path_generator
        self.create_subscription(Pose2D, "pose", self.pose_cb, 10)  # odometry

        ############ ROBOT CONSTANTS ################  
        self.color = 0 # Traffic light
        self.signal = 4 # Traffic signal
        self.pose_line = 0.0 # Position line followed
        self.line_state = True # True if line detected, False otherwise
        self.odometry_flag = False

        self.goal_received = False 
        self.xg = 0.0 # Goal position x[m] 
        self.yg = 0.0 # Goal position y[m] 
        self.goal_array = Float32MultiArray()

        self.finish = Bool() # Flag to finish the path
        self.finish.data = False
        self.state = "stop" #The robot will be initially stopped 

        self.robot_pose = Pose2D() 
        self.xr = 0.0 # Robot position x[m] 
        self.yr = 0.0 # Robot position y[m] 
        self.theta_r = 0.0 # Robot orientation [rad] 
        self.cont = 0

        # - Path generator gains - #
        self.kv = 0.8 #0.159 
        self.kw = 0.9

        # - Line follower gain - #
        self.kp = 0.0125

        # - Constant velocities - #
        self.vel_l_cons = 0.28 # Robot linear velocity [m/s]
        self.vel_a_cons = 3.0 # RObot angular velocity [rad/s]

        self.cmd_vel = Twist() 
        timer_period = 0.01

        self.start_time = self.get_clock().now()
        self.first_time = True
        self.vel_reduction = 1.0


        self.create_timer(timer_period, self.main_timer_cb) 
        self.get_logger().info("Node go_to_goal initialized!!") 
    
    def color_cb(self, colors):
        self.color = colors.data

    def signal_cb(self, signals):
        self.signal = signals.data

    def position_cb(self, position):
        self.pose_line = position.data

    def line_state_cb(self, state_line):
        self.line_state = state_line.data

    def pose_cb(self, pose):  
        ## This function receives the /pose from the odometry_node 
        self.xr = pose.x 
        self.yr = pose.y 
        self.theta_r = pose.theta 

    def goal_cb(self, goal):  
        ## This function receives the /goal from the path_generator node 
        self.goal_array = goal.data

        self.goal_received = True 

        self.state = "turning" # The robot will turn to the goal
        self.get_logger().info("Goal Received")  

    def main_timer_cb(self): 
        ## This function is called every 0.01 seconds 
        if self.line_state: # If the line is detected
            print("Line detected")
            if self.color != 0:
                if self.color == 2 and self.cmd_vel.linear.x == 0.0:
                    amb = 0.0
                elif self.color == 2 and self.cmd_vel.linear.x != 0.0:
                    amb = 0.3
                else:
                    amb = 1.0

                # - STOP - #
                if self.signal == 0:
                    if self.first_time:
                        self.first_time = False
                        self.start_time = self.get_clock().now()
                    
                    if ((self.get_clock().now() - self.start_time).nanoseconds/1e9 <= 10.0):
                        self.vel_reduction = 0.0
                    else:
                        self.vel_reduction = 1.0
                        self.first_time = True
                        self.signal = 4

                
                # - WORK IN PROGRESS - #
                elif self.signal == 1:
                    if self.first_time:
                        self.first_time = False
                        self.start_time = self.get_clock().now()
                    
                    if ((self.get_clock().now() - self.start_time).nanoseconds/1e9 <= 10.0):
                        self.vel_reduction = 0.5
                    else:
                        self.vel_reduction = 1.0
                        self.first_time = True
                        self.signal = 4

                # - GIVE WAY - #
                elif self.signal == 2:
                    if self.first_time:
                        self.first_time = False
                        self.start_time = self.get_clock().now()
                    
                    if ((self.get_clock().now() - self.start_time).nanoseconds/1e9 <= 5.0):
                        self.vel_reduction = 0.5
                    else:
                        self.vel_reduction = 1.0
                        self.first_time = True                         
                        self.signal = 4

                    
                else:
                    self.vel_reduction = 1.0
                    self.first_time = True


                error = 80 - self.pose_line

                self.cmd_vel.linear.x = self.vel_l_cons * amb * self.vel_reduction
                self.cmd_vel.angular.z = self.kp * error * self.vel_a_cons * amb * self.vel_reduction

            else:
                print("Stop: Red detected")
                self.cmd_vel.linear.x -= self.vel_l_cons / 110.0

                if self.cmd_vel.linear.x < 0.0:
                    self.cmd_vel.linear.x = 0.0

                if self.cmd_vel.angular.z > 0.0:
                    self.cmd_vel.angular.z -= self.vel_a_cons / 110.0
                    if self.cmd_vel.angular.z < 0.0:
                        self.cmd_vel.angular.z = 0.0
                elif self.cmd_vel.angular.z < 0.0:
                    self.cmd_vel.angular.z += self.vel_a_cons / 110.0
                    if self.cmd_vel.angular.z > 0.0:
                        self.cmd_vel.angular.z = 0.0
                else:
                    self.cmd_vel.angular.z = 0.0

                
            
            # Publish the command velocity

            print(f"V: {self.cmd_vel.linear.x}")
            print(f"W: {self.cmd_vel.angular.z}")
            self.pub_cmd_vel.publish(self.cmd_vel)

        else:
            print("Odometry mode")
            
            if self.color == 1 and self.odometry_flag == False:
                self.odometry_flag = True
            else:
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0
            
            if self.goal_received and self.odometry_flag:
                if self.cont == 0:
                    self.xg = self.goal_array[0]
                    self.yg = self.goal_array[1]
                else:
                    self.xg = self.goal_array[2]
                    self.yg = self.goal_array[3]

                ed, etheta = self.get_errors(self.xr, self.yr, self.theta_r, self.xg, self.yg)

                if self.state == "stop":
                    print("Stop")
                    self.cmd_vel.linear.x = 0.0 
                    self.cmd_vel.angular.z = 0.0
                    if self.cont > 1:
                        self.cont = 0
                        self.goal_received = False
                        self.finish.data = True
                        self.color = 1
                        self.odometry_flag =  False
                        self.pub_odo_finish.publish(self.finish)
                    else:
                        self.state = "turning"
                    
                
                elif self.state == "turning":
                    if (etheta >= 0.04) or (etheta <= -0.000001): # -np.pi / 120
                        print("Turning")

                        self.cmd_vel.linear.x = 0.0
                        self.cmd_vel.angular.z = self.kw * etheta

                        # if self.color == 1: # Verde
                        #     self.cmd_vel.angular.z = self.kw * etheta
                        # elif self.color == 2 and (self.cmd_vel.angular.z >= 0.5 or self.cmd_vel.angular.z <= -0.5):
                        #     self.cmd_vel.angular.z = self.kw * etheta / 2.0 
                        # else:
                        #     self.cmd_vel.angular.z = 0.0
                    else:
                        self.state = "moving"
                
                elif self.state == "moving":
                    if ed >= 0.02: # Adjust this tolerance if necessary
                        print("Moving")
                        self.cmd_vel.angular.z = 0.0
                        self.cmd_vel.linear.x = self.kv * ed

                        # if self.color == 1: # Verde
                        #     self.cmd_vel.linear.x = self.kv * ed
                        # elif self.color == 2 and self.cmd_vel.linear.x >= 0.1:
                        #     self.cmd_vel.linear.x = self.kv * ed/ 2.0
                        # else:
                        #     self.cmd_vel.linear.x = 0.0

                    else:
                        self.cont += 1
                        self.state = "stop"
                        self.cmd_vel.linear.x = 0.0

                # Limitar velocidades lineales // .47 y .05
                if(self.cmd_vel.linear.x > 0.4):
                    self.cmd_vel.linear.x = 0.4
                elif(self.cmd_vel.linear.x < -0.4):
                    self.cmd_vel.linear.x = -0.4
                elif(self.cmd_vel.linear.x < 0.10 and self.cmd_vel.linear.x > 0.000001):
                    self.cmd_vel.linear.x = 0.10
                elif(self.cmd_vel.linear.x > -0.10 and self.cmd_vel.linear.x < -0.000001):
                    self.cmd_vel.linear.x = -0.10

                # Limitar velocidads angulares // 4.94 y .4
                if(self.cmd_vel.angular.z > 4):
                    self.cmd_vel.angular.z = 4
                elif(self.cmd_vel.angular.z < -4):
                    self.cmd_vel.angular.z = -4
                elif(self.cmd_vel.angular.z < 0.4 and self.cmd_vel.angular.z > 0.0005):
                    self.cmd_vel.angular.z = 0.4
                elif(self.cmd_vel.angular.z > - 0.4 and self.cmd_vel.angular.z < -0.0005):
                    self.cmd_vel.angular.z = -0.4
                
            else:
                print("Waiting for goal") 
                print("Publish the goal to the /goal topic") 
            
            
            self.pub_cmd_vel.publish(self.cmd_vel)



    def shutdown_function(self, signum, frame): 
        self.get_logger().info("Shutting down. Stopping robot...") 
        stop_twist = Twist()  # All zeros to stop the robot 
        self.pub_cmd_vel.publish(stop_twist) 
        rclpy.shutdown() 
        sys.exit(0) 
    
    def get_errors(self, xr, yr, theta_r, xg, yg):
        ed = np.sqrt((xg - xr)**2 + (yg - yr)**2)
        thetag = np.arctan2(yg - yr, xg - xr)
        etheta = thetag - theta_r
        # Limit the angle to [-pi,pi]
        etheta = np.arctan2(np.sin(etheta), np.cos(etheta))

        print("ed: " + str(ed))
        print("etheta: " + str(etheta))
        
        return ed, etheta


def main(args=None): 
    rclpy.init(args=args) 
    my_node=GoToGoal() 
    rclpy.spin(my_node) 
    my_node.destroy_node() 
    rclpy.shutdown() 

if __name__ == '__main__': 
    main() 