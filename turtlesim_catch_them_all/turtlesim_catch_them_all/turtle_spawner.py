#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial

from turtlesim.srv import Spawn

# <service_type> Type of service message eg: AddTwoInts (from example_interfaces.srv)
# <service_name> Name of service to connect to (must be same as the one that server connected to) eg: add_two_ints
 
class SpawnTurtleNode(Node):
    def __init__(self):
        super().__init__("turtle_spawn_client")
        self.call_spawn_service(3.0, 2.0, 0.0, "s1")

    def call_spawn_service(self, x, y, theta, spawnName):        
        client = self.create_client(Spawn, "spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Spawn Server...")

        #SAMPLE
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = spawnName

        # We prefer to use async since it prevents the whole system from stopping
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_spawn_service, x=x, y=y, theta=theta, name=spawnName))

    def callback_call_spawn_service(self, future, x, y, theta, name):
        try:
            response = future.result()
            self.get_logger().info("x = " + str(x) + "\ny = " + str(y) + "\ntheta = " + str(theta) + "\nname = " + name)
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = SpawnTurtleNode() #MODIFY
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
