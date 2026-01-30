""" This program publishes the radius and center of the detected ball   

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
from rcl_interfaces.msg import SetParametersResult
  

class CVExample(Node): 

    def __init__(self): 

        super().__init__('cross_path') 
        self.get_logger().info("The cross_path Node has succesfully initialized...")

        # - PARAMETERS - #
        self.declare_parameter('detection_threshold', 400) # Threshold for detection

        # Retrieve parameters
        self.threshold_bw = self.get_parameter('detection_threshold').value

        # Parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Puente CV2 - ROS2
        self.bridge = CvBridge() 

        # - SUBSCRIBERS - #
        self.sub = self.create_subscription(Image, 'video_source/raw', self.camera_callback, 10) # Real
        
        # - PUBLISHERS - #
        self.pub_line_position = self.create_publisher(Image, 'cross', 10) 
         
        self.image_received_flag = False  
        dt = 0.01 

        # Size image
        self.WIDTH = 160
        self.HEIGHT = 120
        self.pos = Float32()
        self.pos.data = 0.0

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

    def timer_callback(self): 
        # Deteccion de colores
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

    def process_frame(self, frame):
        # Resize the image to 160x120 
        resized_image = cv2.resize(frame, (self.WIDTH,self.HEIGHT)) #(width, height)

        # Convert frame to grayscale
        gray = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)

        # To convert image into binarya Thresholding will be used
        _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)

        # Define the number of rows at the bottom of the image to detect a line
        sensor_height = self.HEIGHT - 50

        # Crop the image to only keep the bottom part
        cropped_image = thresh[sensor_height:, 40:120]

        [cnts, hierarchy] = cv2.findContours(cropped_image.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)  
        center = None  
        cv2.rectangle(resized_image, (60, 70), (100, 120), (0, 255, 255), 1)

        # only proceed if at least one contour was found  
        if len(cnts) > 0:  
            # find the largest contour in the mask, then use  
            # it to compute the minimum enclosing circle and  
            # centroid  
            c = max(cnts, key=cv2.contourArea)  
            ((x, y), radius) = cv2.minEnclosingCircle(c)  
            M = cv2.moments(c)  
            #center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]) + sensor_height)  

            # only proceed if the radius meets a minimum size  
            if radius > 5: #10 pixels 
                # Draw the circle and centroid on the cv_img. 
                cv2.circle(resized_image, (int(x) + 40, int(y) + sensor_height), int(radius), (0, 255, 255), 1)  
                cv2.circle(resized_image, (int(x) + 40, int(y) + sensor_height), 2, (0, 0, 255), -1)  
                self.line_state = True

            else: #If the detected object is too small 
                radius = 0.0  #Just set the radius of the object to zero 
                x = 0.0
                y = 0.0
                self.line_state = False
        else: 
            # All the values will be zero if there is no object  
            x = 0.0 
            y = 0.0 
            radius=0.0 
        # Returns the opencv image 
        return [resized_image, x + 40, y, radius]  


def main(args=None): 
    rclpy.init(args=args) 
    cv_e = CVExample() 
    rclpy.spin(cv_e) 
    cv_e.destroy_node() 
    rclpy.shutdown() 

if __name__ == '__main__': 

    main() 