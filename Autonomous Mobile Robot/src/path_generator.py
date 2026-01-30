import rclpy 
from rclpy.node import Node 
from std_msgs.msg import Bool, Float32MultiArray, Int32
from geometry_msgs.msg import Pose2D 
from rcl_interfaces.msg import SetParametersResult

 
class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator')
        self.get_logger().info("The Path Generator Node has succesfully initialized...")

        # - PARAMETERS - #
        # We declare the parameters for the points
        self.declare_parameter('P1', [0.25, 0.0]) # Standard point
        self.declare_parameter('P2', [0.25, 0.08]) # Point Left
        self.declare_parameter('P3', [0.33, 0.0]) # Point Straight
        self.declare_parameter('P4', [0.25, -0.08]) # Point Right
        self.declare_parameter('P5', [0.0, 0.0]) # Point Stop

        # Retrieve the parameters
        self.p1 = self.get_parameter('P1').value
        self.p2 = self.get_parameter('P2').value
        self.p3 = self.get_parameter('P3').value
        self.p4 = self.get_parameter('P4').value
        self.p5 = self.get_parameter('P5').value

        # Parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Goal points
        self.goal = Float32MultiArray()

        self.send_pose = False # Flag to send pose
        self.signal = 0 # Signal to control the flow

        # PUBLISHERS
        self.pub_goal = self.create_publisher(Float32MultiArray, 'goal', 10)

        # SUBSCRIBERS
        self.signal_sub = self.create_subscription(Int32, 'signal', self.signal_cb, 10)
        self.finish_sub = self.create_subscription(Bool, 'reset_origin', self.start_odometry_cb, 10)

        # Timer callback
        self.timer = self.create_timer(0.01, self.timer_callback)

    def parameter_callback(self, params):
        for param in params:
            # Check Signal type
            if param.name in ["P1", "P2", "P3", "P4"]:
                if (len(param.value) != 2) :
                    self.get_logger().warn("Invalid point. It must contain 2 values!!!")
                    return SetParametersResult(successful=False, reason="Invalid point")
                else:  
                    if param.name == "P1":
                        self.p1 = param.value
                    elif param.name == "P2":
                        self.p2 = param.value
                    elif param.name == "P3":
                        self.p3 = param.value
                    elif param.name == "P4":
                        self.p4 = param.value
                    elif param.name == "P5":
                        self.p4 = param.value

                    self.get_logger().info(f"Point {param.name} successfully changed to: {param.value[0]}, {param.value[1]}")
                    
        return SetParametersResult(successful=True, reason="Parameter changed successfully")

    def signal_cb(self, msg):
        # This function receives the signal from the traffic signal node
        self.signal = msg.data

    def start_odometry_cb(self, msg):
        # This function receives the flag to start sending the pose
        self.send_pose = msg.data


    def timer_callback(self):
       if self.send_pose:
            self.send_pose = False # Reset the flag
            if self.signal == 3: # Left signal
                self.goal.data = [self.p1[0], self.p1[1], self.p2[0], self.p2[1]]
                print(f"The goal has been sent P1({self.p1[0]}, {self.p1[1]}), P2({self.p2[0]}, {self.p2[1]})")
            elif self.signal == 4: # Straight signal
                self.goal.data = [self.p1[0], self.p1[1], self.p3[0], self.p3[1]]    
                print(f"The goal has been sent P1({self.p1[0]}, {self.p1[1]}), P2({self.p3[0]}, {self.p3[1]})")
            elif self.signal == 5: # Right signal
                self.goal.data = [self.p1[0], self.p1[1], self.p4[0], self.p4[1]]
                print(f"The goal has been sent P1({self.p1[0]}, {self.p1[1]}), P2({self.p4[0]}, {self.p4[1]})")

            else:
                self.get_logger().warn("No direction signal received, not sending pose.")
                self.goal.data = [self.p5[0], self.p5[1], self.p5[0], self.p5[1]]
                print(f"The goal has been sent P1({self.p5[0]}, {self.p5[1]}), P2({self.p5[0]}, {self.p5[1]})")
            
            self.pub_goal.publish(self.goal) 

def main(args=None): 
    rclpy.init(args=args) 
    m_p=PathGenerator() 
    rclpy.spin(m_p) 
    m_p.destroy_node() 
    rclpy.shutdown() 


if __name__ == '__main__': 
    main() 
