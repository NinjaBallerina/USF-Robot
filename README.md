![USF](https://github.com/user-attachments/assets/9260fb70-299b-48f8-ab3f-0137140074c8)

### USF Robot ###
![USF Robot 3D Printed](https://github.com/user-attachments/assets/6138831c-cb3c-42aa-8bf0-c28a9c1c77e7)

### Mechanical Design (Autodesk Fusion) ###
<img width="955" alt="USF Robot" src="https://github.com/user-attachments/assets/1fb16194-85bb-4b31-90d5-9e7a2b0da2bb">

### Wiring Schematics ###
<img width="780" alt="USF Robot Wiring Schematics" src="https://github.com/user-attachments/assets/9a715ee7-1f29-409d-8d8e-353582f34d93">

### Joystick Control
![joystick control](https://github.com/user-attachments/assets/47673854-e3c0-47de-a158-6e74cfb0968f)

### USF Robot ROS 2 Package ###

This ROS 2 package, `usf_robot`, is designed to control a 4-DOF robot using a `Pololu Micro Maestro` 6-channel controller and an `8BitDo Ultimate C` joystick. The package includes nodes for handling joystick inputs and controlling the servos on the robot.

### Description

The `usf_robot` package consists of two main nodes:
1. `joystick_control`: Subscribes to joystick inputs and logs the data.
2. `robot_controller`: Controls the robot's servos based on joystick inputs.

### Nodes

1. **Joystick Control Node**:
  - **Subscription**: Subscribes to the `joy` topic to receive joystick inputs.
  - **Purpose**: Logs joystick inputs for debugging and monitoring.

2. **Robot Controller Node**:
  - **Subscription**: Subscribes to the `joy` topic to receive joystick inputs.
  - **Purpose**: Controls the robot's servos based on the received joystick inputs.

### Degrees of Freedom (DOF)

The robot has the following 4 degrees of freedom:
1. **Base Rotation (Servo 0)**: Rotates the robot at the base.
2. **Link 1 Movement (Servo 1)**: Moves the first link of the robot arm.
3. **Link 2 Movement (Servo 2)**: Moves the second link of the robot arm.
4. **Wrist Rotation (Servo 3)**: Rotates the wrist or end effector.

Also, the robot is equipped with a gripper:

**Gripper Operation (Servo 4)**: Opens and closes the gripper. This function is important for object manipulation, but is not counted in the primary DOF calculation.

### Running the Package

1. First, make sure you have ROS 2 `Humble Hawksbill` installed. Open a new terminal then navigate to `usf_robot` from the root:
`cd ~/usf_robot`

2. Before running the package, you need to build it using `colcon`. This compiles the workspace and prepares the necessary executables:
`colcon build`
  
3. Source the ROS 2 `Humble` environment:
`source /opt/ros/humble/setup.bash`

4. Source the workspace:
`source install/setup.bash`

5. To run the package, use the provided launch file:
`ros2 launch usf_robot robot_joystick.launch.py`

**NOTE**: If you encounter this error:
` AttributeError: 'RobotController' object has no attribute 'last_direction' `

Please press a button a few times on the joystick such as Button A or Button B, then hit `CTRL+C` and type this again:
`ros2 launch usf_robot robot_joystick.launch.py`
    
Perform this until the error is gone.

### What Happens When You Run It?

- **joy_node**: This node starts and publishes joystick inputs on the `joy` topic.
- **joystick_control**: This node subscribes to the `joy` topic and logs the joystick inputs for monitoring.
- **robot_controller**: This node subscribes to the `joy` topic and translates joystick inputs into servo commands, controlling the robot's movements.

### Using the Pololu Maestro Control Center

If you need to adjust servo positions without using the joystick, you can use the `Pololu Maestro Control Center`. Connect the USB from the `Pololu Micro Maestro` to the `Raspberry Pi 5` and follow these steps:

1. Navigate to the `Maestro Control Center` directory:
`cd ~/Documents/maestro-linux/`

2. Run the `Maestro Control Center` using Mono:
`mono MaestroControlCenter`

This will launch the `Pololu Maestro Control Center`, where you can manually adjust the positions of your servos using sliders.
