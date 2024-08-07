#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts
     
class AddTwoIntServerNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("add_two_ints_server") # MODIFY NAME
        self.server = self.create_service(AddTwoInts, "add_two_ints", self.callback_add_two_ints)
        self.get_logger().info("Add Two Ints Server has been started.")

    def callback_add_two_ints(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(str(request.a) + " + " + str(request.b) + " = " + str(response.sum))
        return response
     
def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntServerNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()