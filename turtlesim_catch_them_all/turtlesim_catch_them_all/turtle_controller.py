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
 
 
class TurtleControllerNode(Node): #MODIFY
    def __init__(self):
        super().__init__("turtle_controller")
        self.publisher_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.timer_ = self.create_timer(0.5,self.callback)
        self.get_logger().info("Node started")

    def callback(self):
        msg = Twist()
        msg.linear.x = 1.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
