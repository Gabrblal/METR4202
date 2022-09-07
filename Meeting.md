# Meeting Notes

## 8/9/2022 @ 8:00am

Input / Output of the system
- Input:
  - From the camera ros nodes and the dynamixel joint nodes we can determine the current configuration of the robot.
  - `JointState` velocities, positions, and torque (from milliamps).
  - Position and orientation of the boxes in space.
- Output:
  - The desired `JointState` positions, velocities and torque.

Want to get to the output configuration with the gripper above the cube. We know the current orientation of every joint of the cube. Pickup the cube. Transform to configuration to the dropoff point.

State machine for logic controller?

The camera is located above of the whole system.

Camera provides entire 3D pose (position and orientation) of each cube at all times, and the id of each code.

References
- Camera
    - [aruco_detect](http://wiki.ros.org/aruco_detect) / [fiducials](http://wiki.ros.org/fiducials) / [Detection](https://docs.opencv.org/3.1.0/d5/dae/tutorial_aruco_detection.html)
    - [Python Dynamixel Slider Example](https://github.com/UQ-METR4202/dynamixel_slider/blob/master/slider_publisher)
- Motors
    - [Dynamixel Interface](https://github.com/UQ-METR4202/dynamixel_interface/blob/master/tutorials/tutorial_1_using_the_controller.md)
    - [JointState](https://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/JointState.html)


The `JointState` published by the dynamixel / sent for desired `JointState`:
```
This is a message that holds data to describe the state of a set of torque controlled joints. 

The state of each joint (revolute or prismatic) is defined by:
  * the position of the joint (rad or m),
  * the velocity of the joint (rad/s or m/s) and 
  * the effort that is applied in the joint (Nm or N).

Each joint is uniquely identified by its name The header specifies the time at which the joint states were recorded. All the joint states in one message have to be recorded at the same time.

This message consists of a multiple arrays, one for each part of the joint state.  The goal is to make each of the fields optional. When e.g. your joints have no
effort associated with them, you can leave the effort array empty. 

All arrays in this message should have the same size, or be empty. This is the only way to uniquely associate the joint name with the correct
states.

Header header

string[] name
float64[] position
float64[] velocity
float64[] effort

For example:
  name: ['joint_1', 'joint_2']
  position: [0.0000, 0.0000]
  velocity: [0.0000, 0.0000]
  effort: [0.0000, 0.0000]
```
TODO:
- Create a node for determining the end effector configuration of the robot.
  - Create an PoE model for the robot.
  - Implement a PoE node that takes the angles of the joints and publishers the end effecor position.
- Create a logic node for determining the desired angles and what to do next.
- Create an inverse kinematics node for determining the desired angles and trajectory.