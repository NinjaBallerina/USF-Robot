![USF](https://github.com/user-attachments/assets/9260fb70-299b-48f8-ab3f-0137140074c8)

### USF Robot ###
![USF Robot](https://github.com/user-attachments/assets/57f332e3-9ef5-4b41-9066-4761693ca48a)

### Mechanical Design (Autodesk Fusion) ###
<img width="953" alt="USF Robot Fusion" src="https://github.com/user-attachments/assets/e1a8c93d-64b6-45ac-8a92-17b934459180">

### Wiring Schematics ###
<img width="901" alt="USF Robot Wiring" src="https://github.com/user-attachments/assets/758406f2-8fb8-494d-95e3-4ef925842b36">

### Joystick Control
![joystick control](https://github.com/user-attachments/assets/47673854-e3c0-47de-a158-6e74cfb0968f)

### USF Robot ROS 2 Package ###

This ROS 2 package, `usf_robot`, is designed to control a 5-DOF robot using a `Pololu Micro Maestro` 6-channel controller and an `8BitDo Ultimate C` joystick. The package includes nodes for handling joystick inputs and controlling the servos on the robot.

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

The robot has the following 5 degrees of freedom:
1. **Base Rotation (Servo 0)**: Rotates the robot at the base.
2. **Link 1 Movement (Servo 1)**: Moves the first link of the robot arm.
3. **Link 2 Movement (Servo 2)**: Moves the second link of the robot arm.
4. **Wrist Rotation (Servo 3)**: Rotates the wrist or end effector.
5. **Gripper Operation (Servo 4)**: Opens and closes the gripper.

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

### Important: Servo Centering Before Assembly

Before assembling the robot, you must center each servo motor using the **Pololu Maestro Control Center**. If the servos are not centered before assembly, the controller may not correctly control the servos, even if the control range is set appropriately.

To center the servos:
1. Connect the Pololu Micro Maestro to your Raspberry Pi 5 with the USB cable.
2. Open the Maestro Control Center on the Raspberry Pi 5 (see instructions above).
3. For each servo (channels 0-4), use the slider in the Maestro Control Center to set its position to 1500, the center position.
4. While the servo is held at the center position by the Maestro Control Center, manually adjust the servo horn to its physical center position. This means aligning the servo arm to be roughly at a 90-degree angle. This guarantees that the servo's physical position matches the center command from the Maestro.

<img width="659" alt="ServoCenterPosition" src="https://github.com/user-attachments/assets/06b1efab-64b4-4264-b33a-332352d4aca1">
