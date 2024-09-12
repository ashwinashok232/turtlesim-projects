### Video Notes
- 3 nodes
    - Turtlesim node
    - Turtle spawner node
    - Turtle control node


### Expected behaviour

- Turtles are spawned at random in the map
- Main turtle moves towards each spawned turtle
- When it reaches location, spawn turtle will be removed
- Main trutle will keep moving towards the closest spawn turtle
- Turtle moves in a natural path (curved), rather than turning all the way towards spawn turtle
and moving straight
- One launch file will control everything
- Spawn turtles have different colors


### Functional Breakdown

##### Turtlesim

- Launch using:
    - ros2 run turtlesim turtlesim_node
    - ros2 run turtle_teleop_key
- Topics:
    - /turtle1/cmd_vel: linear + angular x,y,z
    - /turtle1/color_sensor: rgb values
    - /turtle1/pose: x,y,theta,linear velocity,angular velocity
- Services
    - /clear
    - /kill
    - /reset
    - /spawn
    - /turtle1/set_pen

##### Turtlesim Node

- Creates and launches moving turtle
- Use /turtle1/pose topic to know its location and velocity

##### Turtle Spawner Node

- Creates new randomly generated turtles
- Use python randint(start,end) to generate random coordinates
    - Need to find size of map to know start and end values for random coodinate generation
- Use /spawn service to create new target turtles

##### Turtle Control Node

- Controls movement of turtle
- Can use propritional control to keep it simple
    - Control input proportional to difference between set point and current state