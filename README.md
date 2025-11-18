# Pegasus Robot Arm Bridge System


![FYRobot](https://github.com/user-attachments/assets/37473287-e853-4a17-bd1c-745ef53172a1)



<img width="1366" height="768" alt="Screenshot from 2025-11-17 19-36-39" src="https://github.com/user-attachments/assets/8e4c958e-4c92-4c56-8dd4-44f5211f464a" />



A comprehensive ROS2-based control system for a 5-DOF Pegasus robotic arm with Arduino bridge integration.

## Overview

This project provides a complete software stack for controlling a Pegasus robotic arm using ROS2, featuring:
- Arduino-based hardware bridge for real-time motor control
- ROS2 integration with description packages and MoveIt planning
- GUI commander interface for manual control
- Enhanced monitoring system with real-time feedback
- Automated launch system for full stack deployment

## System Architecture




```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   ROS2 Stack    │◄──►│  Arduino Bridge  │◄──►│  Physical Arm   │
│                 │    │                  │    │                 │
│ • Description   │    │ • Serial Comm    │    │ • 5-DOF Joints  │
│ • MoveIt        │    │ • Motor Control  │    │ • Servo Motors  │
│ • GUI Commander │    │ • Sensor Reading │    │ • End Effector  │
│ • Path Listener │    │ • Safety Checks  │    │                 │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```





## Hardware Requirements

- **Robot Arm**: 5-DOF Pegasus robotic arm
- **Controller**: Arduino Uno/Nano (connected via USB)
- **Computer**: Linux system with ROS2 installed
- **Connection**: USB cable for Arduino communication (typically `/dev/ttyUSB0`)

## Software Dependencies

### ROS2 Requirements
- ROS2 (Humble/Iron/Jazzy recommended)
- MoveIt2 planning framework
- ros2_control packages
- Joint trajectory controllers

### System Requirements
- Ubuntu 20.04+ or compatible Linux distribution
- Python 3.8+
- Arduino IDE (for firmware modifications)

## Installation

1. **Clone the repository:**
   ```bash
   git clone https://github.com/mmm404/robot_arm_based_on_pegasus.git
   cd robot_arm_based_on_pegasus
   ```

2. **Setup ROS2 workspace:**
   ```bash
   # Ensure you're in a ROS2 workspace (e.g., ~/ros2-ws)
   # If not, create one:
   mkdir -p ~/ros2-ws/src
   cd ~/ros2-ws/src
   # Move or clone the project here
   ```

3. **Install dependencies:**
   ```bash
   cd ~/ros2-ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. **Build the workspace:**
   ```bash
   colcon build
   source install/setup.bash
   ```

5. **Arduino Setup:**
   - Upload the provided Arduino sketch to your Arduino board
   - Connect the Arduino to your robot arm hardware
   - Ensure proper wiring for servo control and sensor feedback

## Quick Start

### Launch the Complete System

<img width="1349" height="652" alt="Screenshot from 2025-09-17 12-14-17" src="https://github.com/user-attachments/assets/4a16e9c8-9cde-4c5d-a75f-4dba73512dba" />



From your ROS2 workspace directory:

```bash
scripts/run_full_stack.sh
```

This automated launcher will:

1. **[1/6]** Launch ROS2 robot description in a new terminal
2. **[2/6]** Check and verify Arduino connection at `/dev/ttyUSB0`
3. **[3/6]** Initialize the Arduino bridge communication
4. **[4/6]** Start the enhanced monitoring system
5. **[5/6]** Open the GUI commander interface
6. **[6/6]** Launch the planned path listener for trajectory execution

### Expected Output

```
================================================
    PEGASUS ARM BRIDGE SYSTEM LAUNCHER
================================================
✓ ROS2 environment sourced
✓ ROS2 description launched in separate terminal
✓ Real Arduino found and accessible at: /dev/ttyUSB0
✓ Arduino Bridge mode: bridge
✓ GUI Commander: Running in separate terminal
✓ Listener: Running in separate terminal
✓ Monitor: Running in separate terminal
================================================
    BRIDGE SYSTEM READY
================================================
Press Ctrl+C here to stop everything and perform cleanup
```

## Manual Launch (Alternative)

If you prefer to launch components individually:

```bash
# Terminal 1: Robot Description
ros2 launch pegasus_description display.launch.py

# Terminal 2: Arduino Bridge
ros2 run pegasus_bridge arduino_bridge_node

# Terminal 3: GUI Commander
ros2 run pegasus_commander gui_commander

# Terminal 4: Path Listener
ros2 run pegasus_assistant planned_path_listener

# Terminal 5: Monitor
ros2 run pegasus_monitor enhanced_monitor
```


<img width="1363" height="688" alt="Screenshot from 2025-09-17 12-17-16" src="https://github.com/user-attachments/assets/343690af-ea94-4917-bbee-030143d7a40a" />


## Package Structure

```
robot_arm_based_on_pegasus/
├── scripts/
│   └── run_full_stack.sh          # Main launcher script
├── src/
│   ├── pegasus_description/        # URDF/XACRO robot model
│   ├── pegasus_bridge/            # Arduino communication bridge
│   ├── pegasus_commander/         # GUI control interface
│   ├── pegasus_assistant/         # Path planning utilities
│   ├── pegasus_monitor/           # System monitoring tools
│   └── pegasus_moveit_config/     # MoveIt configuration
└── arduino/
    └── pegasus_firmware/          # Arduino sketch
```

## Usage

### GUI Commander
- Use the graphical interface to manually control joint positions
- Real-time joint state feedback
- Emergency stop functionality
- Predefined pose shortcuts

### Path Planning
- Utilize MoveIt integration for complex trajectory planning
- Collision avoidance with environment obstacles
- Smooth trajectory execution with velocity/acceleration limits

### Monitoring
- Real-time joint positions and velocities
- Arduino communication status
- System health indicators
- Error logging and diagnostics

## Troubleshooting

### Arduino Connection Issues
```bash
# Check available serial ports
ls /dev/ttyUSB* /dev/ttyACM*

# Verify permissions
sudo usermod -a -G dialout $USER
# (logout and login again)

# Test connection
arduino-cli board list
```

### Bridge Process Not Running
- Check `/tmp/arduino_sim.log` for detailed error messages
- Verify Arduino firmware is properly uploaded
- Ensure proper baud rate configuration (typically 115200)

### ROS2 Node Communication Issues
```bash
# Check active nodes
ros2 node list

# Verify topic communication
ros2 topic list
ros2 topic echo /joint_states

# Check service availability
ros2 service list
```

## Development

### Adding New Functionality
1. Create new packages following ROS2 conventions
2. Update the launch script to include new components
3. Modify Arduino firmware if hardware changes are needed

### Testing
```bash
# Run system tests
colcon test
colcon test-result --verbose

# Manual testing
ros2 run pegasus_bridge test_communication
```

## Contributing

1. Fork the repository
2. Create a feature branch: `git checkout -b feature-name`
3. Commit changes: `git commit -am 'Add feature'`
4. Push to branch: `git push origin feature-name`
5. Submit a Pull Request


<img width="1349" height="652" alt="Screenshot from 2025-09-17 12-13-53" src="https://github.com/user-attachments/assets/7ce4aab3-1c19-49c5-b739-b4f00f342bb8" />



## License

This project is licensed under the [MIT License](LICENSE) - see the LICENSE file for details.

## Support

For issues and questions:
- Check the [Issues](https://github.com/mmm404/robot_arm_based_on_pegasus/issues) page
- Review the troubleshooting section above
- Examine log files in `/tmp/` directory for detailed error information

## Acknowledgments

- ROS2 community for the robotics framework
- MoveIt for motion planning capabilities
- Arduino community for embedded control solutions
