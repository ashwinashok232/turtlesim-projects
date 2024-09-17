'''
To do when creating a publisher:

 1. Add imported libraries to package.xml
    <depend>package_name</depend>
    
 2. Update setup.py
    
    entry_points={
        'console_scripts': [
            "py_node = <package_name>.<node_executable_name>:main"
        ],
    }
    Note: node_executable_name does not need to be the same as the name of the node in the python script
 
 3. Make file executable
    
    chmod +x <python_filename>.py

 4. Source and build the package:
    
    $ source ~/.bashrc
    $ colcon build --packages-select <package_name> --symlink-install

** IMPORTANT: colcon build must be done in the ~/ros2_ws directory, not in any subdirectory (eg: ~/ros2_ws/src)
----------------

To run the publisher:

    ros2 run <package_name> <node_name_in_python>

Some optional arguments you can add:

    - Renaming node at runtime (eg: if you run the same node twice, you should name them differently to avoid conflicts):
        ros2 run <package_name> <node_name_in_python> --ros-args --remap __node:=<custom node name>
'''

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist 
from turtlesim_interfaces.msg import SpawnList
from geometry_msgs.msg import Vector3
from turtlesim.msg import Pose
import math
 
class TurtleControllerNode(Node): #MODIFY
    def __init__(self):
        super().__init__("turtle_controller")
        self.spawn_coordinate_list_ = []
        self.turtle_pose_ = (0.0, 0.0)
        self.goalseeking_ = False
        self.target_pose_ = None
        # self.publisher_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.main_turtle_pose_subscriber_ = self.create_subscription(Pose, "turtle1/pose", self.pose_callback, 10)
        self.spawn_coordinate_subscriber_ = self.create_subscription(Vector3, "spawn_coordinates", self.spawn_coordinate_callback, 10)
        self.velocity_publisher_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        # self.timer_ = self.create_timer(0.1,self.velocity_callback)
        self.get_logger().info("Controller Node started")

    def spawn_coordinate_callback(self, msg):
        self.get_logger().info("LOGGING COORDINATES")
        self.spawn_coordinate_list_.append([msg.x, msg.y])
        turtle_coord = [self.turtle_pose_[0], self.turtle_pose_[1]]
        self.get_logger().info(str(self.spawn_coordinate_list_))
        if not self.goalseeking_:
            smallestDist = math.inf
            targetPoint = None
            for spawn_coord in self.spawn_coordinate_list_:
                self.get_logger().info(str(spawn_coord) + " " + str(turtle_coord))
                currentDist = math.dist(spawn_coord, turtle_coord)
                if currentDist < smallestDist:
                    smallestDist = currentDist
                    targetPoint = spawn_coord
                    self.get_logger().info("CALCULATING TARGET: " + str(smallestDist) + " " + str(targetPoint))
            self.target_pose_ = targetPoint
            self.goalseeking_ = True
            self.get_logger().info("FINAL CLOSEST POINT:" + str(smallestDist) + " " + str(targetPoint))
            
    def pose_callback(self, msg):
        self.turtle_pose_ = (msg.x, msg.y, msg.theta, msg.linear_velocity, msg.angular_velocity)
        self.get_logger().info("ROBOT POSE")
        self.get_logger().info(str(self.turtle_pose_))
        self.velocity_callback()
        
    def velocity_callback(self):
        linear_gain = 0.1
        angular_gain = 0.5
        if self.goalseeking_:
            while self.target_pose_ != [self.turtle_pose_[0], self.turtle_pose_[1]]:
                polar_distance_error = math.dist(self.target_pose_, [self.turtle_pose_[0], self.turtle_pose_[1]])
                polar_angle_error = math.atan2(self.target_pose_[0]-self.turtle_pose_[0], self.target_pose_[1]-self.turtle_pose_[1]) - self.turtle_pose_[2]
                # self.get_logger().info(f"DISTANCE ERROR: {polar_distance_error}   ANGLE ERROR: {polar_angle_error}")
                msg = Twist()
                msg.linear.x = linear_gain*polar_distance_error
                msg.angular.z = angular_gain*polar_angle_error
                msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y = 0.0, 0.0, 0.0, 0.0
                self.velocity_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
