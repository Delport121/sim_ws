#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import String

class MotorControlNode(Node):
    def __init__(self):
        super().__init__("motor_control_basic")

        #Max and min speeds
        self.maxPWM = 50
        self.minPWM = 30
        self.PWMrange = self.maxPWM - self.minPWM

        #Assign start motor PWM variables
        self.speedL = self.minPWM + self.PWMrange/2
        self.speedR = self.minPWM + self.PWMrange/2
        
        #Create publsher for wheel speed data
        self.publisher_ = self.create_publisher(String, 'motor_speed', 10)
        self.timer_ = self.create_timer(0.05, self.publish_speed)

        # Create subscriber for IR sensor data
        self.subscription = self.create_subscription(String,'IR_data',self.ir_callback,10)
        #self.subscription  # prevent unused variable warning

        self.get_logger().info("Basic motor control has been started!!!!")

    def ir_callback(self, msg):

        self.get_logger().info(msg.data)

        ir_values = [int(val) for val in msg.data.split(',')]
        # Assuming the middle sensor is the third one, adjust the speed based on its value

        if (ir_values[4] == 0):
            self.speedL = self.maxPWM  
            self.speedR = 0
        elif (ir_values[3] == 0):
            # Adjust speeds to turn towards the line
            self.speedL = self.maxPWM  
            self.speedR = self.minPWM
        #elif (ir_values[2] == 0):
            #self.speedL = self.minPWM + self.PWMrange/2  # keep the same speed
            #self.speedR = self.minPWM + self.PWMrange/2
        elif (ir_values[1] == 0):
            # Adjust speeds to turn towards the line
            self.speedL = self.minPWM
            self.speedR = self.maxPWM
        elif (ir_values[0] == 0):
            # Adjust speeds to turn towards the line
            self.speedL = 0
            self.speedR = self.maxPWM
       
    def publish_speed(self):
        msg = String()
        # Format the message data as "speedL,speedR"
        msg.data = f"{self.speedL},{self.speedR}"
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
