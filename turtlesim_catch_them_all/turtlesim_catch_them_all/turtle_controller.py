#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist 
from turtlesim_interfaces.msg import SpawnInfo
from turtlesim.msg import Pose
import math
from turtlesim.srv import Kill
from functools import partial
 
class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.spawn_coordinate_list_ = []
        self.turtle_pose_ = (0.0, 0.0)
        self.goalseeking_ = False
        self.target_pose_ = None
        self.target_name_ = None
        self.main_turtle_pose_subscriber_ = self.create_subscription(Pose, "turtle1/pose", self.pose_callback, 10)
        self.spawn_coordinate_subscriber_ = self.create_subscription(SpawnInfo, "spawn_coordinates", self.spawn_coordinate_callback, 10)
        self.velocity_publisher_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.get_logger().info("Controller Node started")

    def spawn_coordinate_callback(self, msg):
        self.update_spawn_coordinate_list(msg)
        self.find_closest_spawn_point()

            
    def update_spawn_coordinate_list(self, msg):
        self.get_logger().info("LOGGING COORDINATES")
        self.spawn_coordinate_list_.append({"x": msg.x_coord, "y": msg.y_coord, "name": msg.name})
        self.get_logger().info(str(self.spawn_coordinate_list_))
        
    def find_closest_spawn_point(self):
        turtle_coord = [
            self.turtle_pose_[0], 
            self.turtle_pose_[1]
            ]
        if not self.goalseeking_:
            smallestDist, targetPoint, spawnName = math.inf, None, None
            for spawn_dict in self.spawn_coordinate_list_:
                spawn_coord = [spawn_dict["x"], spawn_dict["y"]]
                currentDist = math.dist(spawn_coord, turtle_coord)
                if currentDist < smallestDist:
                    smallestDist = currentDist
                    targetPoint = spawn_coord
                    spawnName = spawn_dict["name"]
            self.target_pose_ = targetPoint
            self.target_name_ = spawnName
            self.goalseeking_ = True
            # self.get_logger().info("FINAL CLOSEST POINT:" + str(smallestDist) + " " + str(targetPoint))
            
    def pose_callback(self, msg):
        self.turtle_pose_ = (msg.x, msg.y, msg.theta, msg.linear_velocity, msg.angular_velocity)
        # self.get_logger().info("ROBOT POSE")
        # self.get_logger().info(str(self.turtle_pose_))
        self.velocity_callback()
        
    def velocity_callback(self):
        linear_gain = 1.0
        angular_gain = 1.5
        goal_tol = 0.1

        if self.goalseeking_:
            polar_distance_error = math.dist(self.target_pose_, [self.turtle_pose_[0], self.turtle_pose_[1]])
            polar_angle_error = convertPrincipalToFullCircleAngle(math.atan2(self.target_pose_[1]-self.turtle_pose_[1], 
                                           self.target_pose_[0]-self.turtle_pose_[0])) - convertPrincipalToFullCircleAngle(self.turtle_pose_[2])
            if (polar_distance_error <= goal_tol):
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y = 0.0, 0.0, 0.0, 0.0
                self.velocity_publisher_.publish(msg)
                self.spawn_coordinate_list_.remove({"x": self.target_pose_[0], "y": self.target_pose_[1], "name": self.target_name_})
                self.call_kill_service(self.target_name_)
                self.goalseeking_ = False
                self.get_logger().info('################## GOAL REACHED ##################')
            # while self.target_pose_ != [self.turtle_pose_[0], self.turtle_pose_[1]]:
            # self.get_logger().info(f"DISTANCE ERROR: {polar_distance_error}   ANGLE ERROR: {polar_angle_error}")
            # self.get_logger().info(f"{math.atan2(self.target_pose_[1]-self.turtle_pose_[1], self.target_pose_[0]-self.turtle_pose_[0])}         {self.turtle_pose_[2]}")
            else:
                msg = Twist()
                msg.linear.x = linear_gain*polar_distance_error
                msg.angular.z = angular_gain*polar_angle_error
                msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y = 0.0, 0.0, 0.0, 0.0
                self.velocity_publisher_.publish(msg)
            
    def call_kill_service(self, name):        
        client = self.create_client(Kill, "kill")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Kill Server...")

        #SAMPLE
        request = Kill.Request()
        request.name = name

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_kill_service, name=name))

    def callback_call_kill_service(self, future, name):
        try:
            response = future.result()
            self.get_logger().info(f"Killing {name}")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()

def convertPrincipalToFullCircleAngle(principalAngle):
    return (principalAngle + 2*math.pi) % (2*math.pi)