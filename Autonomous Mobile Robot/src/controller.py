""" 
    This program publishes the radius and center of the detected ball   

    The radius will be zero if there is no detected object  
    published topics:  
        /processed_img [Image] 

    subscribed topics:  
        /camera    [Image]  

"""  

import rclpy 
from rclpy.node import Node 
import cv2 
import numpy as np 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist 
from rcl_interfaces.msg import SetParametersResult  

class CVExample(Node): 

    def __init__(self): 

        super().__init__('controller') 
        self.get_logger().info("The controller Node has succesfully initialized...")

        # - PARAMETERS - #
        self.declare_parameter('detection_threshold', 400) # Threshold for detection

        # Retrieve parameters
        self.threshold_bw = self.get_parameter('detection_threshold').value

        # Parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Puente CV2 - ROS2
        self.bridge = CvBridge() 

        # - SUBSCRIBERS - #
        #self.sub = self.create_subscription(Image, 'camera', self.camera_callback, 10) # SImulación
        self.sub = self.create_subscription(Image, 'video_source/raw', self.camera_callback, 10) # Real
        self.sub = self.create_subscription(Bool, 'odometry_finish', self.odo_cb, 10) # Real
        
        # - PUBLISHERS - #
        self.pub_line_position = self.create_publisher(Image, 'line_position', 10) 
        self.pub_position = self.create_publisher(Float32, 'position', 10)
        self.pub_line_state = self.create_publisher(Bool, 'line_state', 10)        
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10) # Puzzlebot

         
        self.image_received_flag = False #This flag is to ensure we received at least one image  
        dt = 0.01 
        
        self.line_state = Bool()
        self.line_state.data = True #False if no line detected

        self.odometry_finish = True
        
        self.state = 'line'

        # Size image
        self.WIDTH = 160
        self.HEIGHT = 120
        self.pos = Float32()
        self.pos.data = 0.0        
        
        self.cmd_vel = Twist() 


        self.timer = self.create_timer(dt, self.timer_callback) 

    def parameter_callback(self, params):

        for param in params:
            # Check thresholding value
            if param.name == "detection_threshold":
                if (param.value < 100) or (param.value > 1500):
                    self.get_logger().warn("The thresholfing value must be between 100 and 1500.")
                    return SetParametersResult(successful=False, reason="Invalid thresholding value")
                else:   
                    self.threshold_bw = param.value
                    self.get_logger().info(f"Thresholding value ssuccessfully changed to: {self.threshold_bw}")

        return SetParametersResult(successful=True, reason="Parameter changed successfully")
    
    def camera_callback(self, msg): 
        try:  
            # We select bgr8 because its the OpenCV encoding by default  
            self.cv_img= self.bridge.imgmsg_to_cv2(msg, "bgr8")  
            self.image_received_flag = True  

        except: 
            self.get_logger().info('Failed to get an image') 
    
    def odo_cb(self, flag):
        self.odometry_finish = flag.data

    def timer_callback(self): 
        if self.odometry_finish:
            # Deteccion de colores
            #if self.image_received_flag and self.line_state == True: 
            if self.image_received_flag: 
                self.image_received_flag=False 

                [img_processed, x, y, radius] = self.process_frame(self.cv_img)

                # Publish the processed image
                self.pub_line_position.publish(self.bridge.cv2_to_imgmsg(img_processed,'bgr8')) # 8UC1 / bgr8
                # Publish the position of the line
                print("x: ", x) 
                print("y: ", y) 
                print("radius: ", radius) 
                # Publish the 
                self.pos.data = x
                self.pub_position.publish(self.pos)

    def process_frame(self, frame):
        # Resize the image to 160x120 
        resized_image = cv2.resize(frame, (self.WIDTH,self.HEIGHT)) #(width, height)

        # Convert frame to grayscale
        gray = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)

        # To convert image into binarya Thresholding will be used
        _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)

        kernel = np.ones((5, 5), np.uint8)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

        #if self.state == 'line':    
        # Define the number of rows at the bottom of the image to detect a line
        sensor_height1 = self.HEIGHT - 30

        # Crop the image to only keep the bottom part
        cropped_image1 = thresh[sensor_height1:, 20:140]
        
        #elif self.state == 'cross':
        # Define the number of rows at the bottom of the image to detect a line
        sensor_height2 = self.HEIGHT - 60

        # Crop the image to only keep the bottom part
        cropped_image2 = thresh[sensor_height2:, 35:125]

        [cnts1, hierarchy] = cv2.findContours(cropped_image1.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)  
        center = None  

        [cnts2, hierarchy] = cv2.findContours(cropped_image2.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)  
        center = None  

        if len(cnts2) > 5:
            self.state = 'cross' #If there are more than 3 contours, it is a cross

        if self.state == 'line':
            cnts = cnts1
        else:
            cnts = cnts2

        # only proceed if at least one contour was found  
        if len(cnts) > 0:  
            # find the largest contour in the mask, then use  
            # it to compute the minimum enclosing circle and  
            # centroid  
            c = max(cnts, key=cv2.contourArea)  

            print(f"Contornos1: {len(cnts1)}")
            print(f"Contornos2: {len(cnts2)}")

            ((x, y), radius) = cv2.minEnclosingCircle(c)  
            M = cv2.moments(c)  
            #center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]) + sensor_height)  

            if self.state == 'line':
                # only proceed if the radius meets a minimum size  
                if radius > 10: #10 pixels 
                    # Draw the circle and centroid on the cv_img. 
                    cv2.circle(resized_image, (int(x) + 20, int(y) + sensor_height1), int(radius), (255, 0, 255), 1)  
                    cv2.circle(resized_image, (int(x) + 20, int(y) + sensor_height1), 2, (0, 0, 255), -1)  
                    print("line state")
                    self.line_state.data = True
                    self.pub_line_state.publish(self.line_state) #sends False to no_line topic if object detected
                    
                else: #If the detected object is too small 
                    self.state = 'cross'


            elif self.state == 'cross':
                if radius > 5: # pixels 
                    # Draw the circle and centroid on the cv_img. 
                    cv2.circle(resized_image, (int(x) + 40, int(y) + sensor_height2), int(radius), (0, 255, 0), 1)  
                    cv2.circle(resized_image, (int(x) + 40, int(y) + sensor_height2), 2, (0, 0, 255), -1)  
                    x = 60.0
                    print("cross state")
                    self.line_state.data = True
                    self.pub_line_state.publish(self.line_state) #sends False to no_line topic if object detected
                else:
                    radius = 0.0  #Just set the radius of the object to zero  
                    self.line_state.data = False
                    self.odometry_finish = False
                    self.pub_line_state.publish(self.line_state)    

                    self.state = 'line'

        else: 
            # All the values will be zero if there is no object  
            x = 60.0 
            y = 0.0 
            radius=0.0 
        # Returns the opencv image 
        return [resized_image, x + 20.0, y, radius]  


def main(args=None): 
    rclpy.init(args=args) 
    cv_e = CVExample() 
    rclpy.spin(cv_e) 
    cv_e.destroy_node() 
    rclpy.shutdown() 

if __name__ == '__main__': 

    main() 