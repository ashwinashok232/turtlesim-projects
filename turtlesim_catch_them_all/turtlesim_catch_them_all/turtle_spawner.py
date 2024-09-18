#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial

from turtlesim.srv import Spawn
from turtlesim_interfaces.msg import SpawnInfo
import random

# <service_type> Type of service message eg: AddTwoInts (from example_interfaces.srv)
# <service_name> Name of service to connect to (must be same as the one that server connected to) eg: add_two_ints
 
class SpawnTurtleNode(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        self.spawnCoordinateList_ = []
        self.spawn_count_ = 0
        self.create_timer(3.0,self.turtle_spawn_timer_callback)
        self.publisher_ = self.create_publisher(SpawnInfo, "spawn_coordinates", 10)

    def call_spawn_service(self, x, y, theta, name):        
        client = self.create_client(Spawn, "spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Spawn Server...")

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name

        # We prefer to use async since it prevents the whole system from stopping
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_spawn_service, x=x, y=y, theta=theta, name=name))

    def callback_call_spawn_service(self, future, x, y, theta, name):
        try:
            response = future.result()
            self.get_logger().info("x = " + str(x) + "\ny = " + str(y) + "\ntheta = " + str(theta) + "\nname = " + name)
            msg = SpawnInfo()
            msg.x_coord = x
            msg.y_coord = y
            msg.theta = theta
            msg.name = name
            self.publisher_.publish(msg)
            self.spawn_count_ += 1
            
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    def turtle_spawn_timer_callback(self):
        x_min,x_max,y_min,y_max = 0.0, 10.0, 0.0, 10.0
        
        x_spawn = round(random.uniform(x_min,x_max),1)
        y_spawn = round(random.uniform(y_min,y_max),1)
        self.call_spawn_service(x_spawn, y_spawn, 0.0, f"Spawn{self.spawn_count_}")

def main(args=None):
    rclpy.init(args=args)
    node = SpawnTurtleNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
