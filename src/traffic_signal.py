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
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
from std_msgs.msg import Int32
from ultralytics import YOLO  # Agrega al inicio del script

  
# 
class CVExample(Node): 

    def __init__(self): 

        super().__init__('traffic_signal') 
        self.get_logger().info("The traffic_signal Node has succesfully initialized...")

        # - PARAMETERS - #
        self.model = YOLO("/home/israels/ros2_ws/src/challenge_final/challenge_final/best_signals.pt")  # Carga el modelo YOLOv8
        
        # Puente CV2 - ROS2
        self.bridge = CvBridge()

        # - SUBSCRIBERS - #
        #self.sub = self.create_subscription(Image, 'camera', self.camera_callback, 10) # SImulación
        self.sub = self.create_subscription(Image, 'video_source/raw', self.camera_callback, 10) # Real

        # - PUBLISHERS - #
        self.image_result_pub = self.create_publisher(Int32, 'signal', 10)
        self.processed_image_pub = self.create_publisher(Image, 'processed_img_ts', 10)
         
        self.image_received_flag = False #This flag is to ensure we received at least one image  
        dt = 0.01 
        self.timer = self.create_timer(dt, self.timer_callback) 

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

            # Resize the image to 160x120 
            resized_image = cv2.resize(self.cv_img, (160,120)) #(width, height) 

            results = self.model(resized_image)[0]  # Ejecuta inferencia

            # Dibuja bounding boxes sobre la imagen original (no redimensionada si quieres más resolución)
            for box in results.boxes:
                cls_id = int(box.cls)
                label = self.model.names[cls_id]
                conf = box.conf.item()
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                print(f"P1({x1}, {y1}) y P2({x2}, {y2})")
                
                area = (x2 - x1) * (y2 -y1)

                if area >= 370 and conf >= 0.85:

                    # Dibujar caja
                    cv2.rectangle(resized_image, (x1, y1), (x2, y2), (0, 255, 0), 1)
                    #cv2.putText(resized_image, f'{conf:.2f}', (x2, y2 + 5),
                    #            cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 0, 0), 1, cv2.LINE_AA)
                    
                    cv2.putText(resized_image, f'{label} : {conf:.2f}', (x2-x1, y2 + 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 255), 1, cv2.LINE_AA)

                    if label == 'ts_stop':
                        print("STOP")
                        self.image_result_pub.publish(Int32(data=0))
                    elif label == 'ts_work':
                        print("Work in progress")
                        self.image_result_pub.publish(Int32(data=1))
                    elif label == 'ts_giveway':
                        print("Give way")
                        self.image_result_pub.publish(Int32(data=2))
                    elif label == 'ts_left':
                        print("Turn left")
                        self.image_result_pub.publish(Int32(data=3))
                    elif label == 'ts_straight':
                        print("Go straight")
                        self.image_result_pub.publish(Int32(data=4))
                    elif label == 'ts_right':
                        print("Turn right")
                        self.image_result_pub.publish(Int32(data=5))
                    else:
                        print("No clear signal detected")

            # Publicar imagen anotada
            processed_msg = self.bridge.cv2_to_imgmsg(resized_image, encoding="bgr8")
            self.processed_image_pub.publish(processed_msg)

            # Publicar clases detectadas
            classes_detected = [self.model.names[int(cls)] for cls in results.boxes.cls]
            print(f"Clases detectadas: {classes_detected}")


def main(args=None): 
    rclpy.init(args=args) 
    cv_e = CVExample() 
    rclpy.spin(cv_e) 
    cv_e.destroy_node() 
    rclpy.shutdown() 

if __name__ == '__main__': 

    main() 
