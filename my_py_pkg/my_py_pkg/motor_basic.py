#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO          
import time
     
from example_interfaces.msg import String
     
class MotorsNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("motor_basic") # MODIFY NAME
        self.subscriber_ = self.create_subscription(String, "motor_speed", self.set_speed, 10)
        self.get_logger().info("Motor control basic have been started!")

        #Assigning pins
        A1in = 12
        B1in = 13

        #Setting up GPIO pins output
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(A1in,GPIO.OUT)
        GPIO.setup(B1in,GPIO.OUT)
        GPIO.output(A1in,GPIO.LOW)
        GPIO.output(B1in,GPIO.LOW)

        #Assigning PWM functionality to pins
        self.pwmR=GPIO.PWM(A1in,1000)
        self.pwmL=GPIO.PWM(B1in,1000)

        # Starting PWM value
        self.pwmR.start(0)
        self.pwmL.start(0)

    def set_speed(self, msg):
        self.get_logger().info(msg.data)

        # Split the string into two parts based on the comma
        speed_values = msg.data.split(',')

        if len(speed_values) == 2:
            # Extract individual speed values
            speedL, speedR = speed_values

            # Assign duty cycle to the motors
            self.pwmR.ChangeDutyCycle(float(speedR))
            self.pwmL.ChangeDutyCycle(float(speedL))
        else:
            self.get_logger().warn("Invalid speed format. Expected 'speedL,speedR'.")

def main(args=None):
    rclpy.init(args=args)
    node = MotorsNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()