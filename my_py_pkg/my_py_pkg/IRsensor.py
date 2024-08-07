#!/usr/bin/env python3
import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

from example_interfaces.msg import String

class IRsensorNode(Node):
    def __init__(self):
        super().__init__("IR_sensor")

        # Create publisher
        self.publisher_ = self.create_publisher(String, 'IR_data', 10)
        self.timer_ = self.create_timer(0.05, self.publish_IR_data)
        self.get_logger().info("IR data node has been started!")

        # Assigning GPIO pins
        self.ir_pins = [17, 27, 22, 23, 24]  # Example GPIO pin numbers, modify as per your setup

        # Setting up GPIO pins input
        GPIO.setmode(GPIO.BCM)
        for pin in self.ir_pins:
            GPIO.setup(pin, GPIO.IN)

    def read_ir_values(self):
        ir_values = [GPIO.input(pin) for pin in self.ir_pins]
        return ir_values

    def publish_IR_data(self):
        ir_values = self.read_ir_values()
        ir_data_str = ','.join(str(val) for val in ir_values)
        msg = String()
        msg.data = ir_data_str
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = IRsensorNode()
    rclpy.spin(node)
    rclpy.shutdown()
    GPIO.cleanup()

if __name__ == "__main__":
    main()
