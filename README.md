# Pegasus Arm Project

## Overview

This repository contains the code and documentation for the Pegasus Arm, a robotic arm project utilizing ROS2, MoveIt, and custom hardware interfaces. The project integrates a simulated environment, CAD designs, and a Kinect sensor for motion capture.

---

## Setup Instructions<img width="806" height="736" alt="Screenshot from 2025-07-31 14-30-52" src="https://github.com/user-attachments/assets/fe2a3875-a7ca-42a0-9ebb-bb29a66489c0" />


### Activating Python3 Workspace

Activate your virtual environment from any directory:
```bash
source ~/arduino_communication/venv/bin/activate
```

Run project files from any location:
```bash
cd ~/Documents/some_project
python ~/arduino_communication/vscode_sender.py
```

### Launching MoveIt

```bash
ros2 launch pegasus_arm_description moveit_display.launch.py
```

### Running the Controller

```bash
ros2 run pegasus_arm_commander pegasus_commander
```

### Cleaning the Workspace

```bash
rm -rf install build src
```

### Checking Port Usage and Permissions

```bash
sudo lsof /dev/ttyUSB0
ls -l /dev/ttyUSB0
```

### Monitoring Data with Simulator

```bash
screen /tmp/virtual_monitor_sim 9600
```

### Starting the Serial Bridge

```bash
ros2-ws/scripts/serial_bridge.sh
```

### Starting PuTTY Using Screen

```bash
screen /tmp/virtual_monitor 9600
```

### Creating Virtual Linked Ports

```bash
socat -d -d PTY,link=/tmp/ttyV0,raw,echo=0 PTY,link=/tmp/ttyV1,raw,echo=0
```

### PuTTY Configuration

Connection type: Serial  
Serial line: /tmp/ttyV1  
Speed: 9600

### Opening Setup Assistant

```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

---

## Progress Report (24/July/2025)

### ROS2 and Scripts

**Issues:**
- Joint state publisher and arm commanders not initializing  
- MoveIt fails to plan  
- Hardware interface unconfigured  
- Interactive GUI failure  
<img width="811" height="506" alt="Screenshot from 2025-07-31 14-27-43" src="https://github.com/user-attachments/assets/59db9698-a1a7-486c-9e7a-b432f8ae10a8" />
<img width="1331" height="204" alt="Screenshot from 2025-07-31 14-29-17" src="https://github.com/user-attachments/assets/e17fa68a-cd88-4e28-9e24-ab41763fb573" />

**Achievements:**
- URDF loads correctly in RViz  
- Commanders functional  
- GUI works (not yet linked to hardware)  
- Partial hardware interface success  

**Next Steps:**
- Finalize hardware interface integration  
- Sync GUI with real-time control  
- Debug MoveIt planning  
- Test full launch file sequence  

---<img width="806" height="736" alt="Scr<img width="806" height="736" alt="Screenshot from 2025-07-31 14-30-32" src="https://github.com/user-attachments/assets/db72662c-74d7-4439-a9d1-42ecab60dec7" />
eenshot from 2025-07-31 14-30-52" src="https://github.com/user-attachments/assets/d02fb238-2df1-421f-9991-92946165b056" />


### CAD

**Issues:**
- Arm off-center  
- Bearing holders misaligned  
- No electrical compartment  
- Spur gear loosely fixed  
- Screws too bulky  

**Achievements:**
- Bearing holders repositioned  
- Electrical module included  
- Spur gear now fits  

**Next Steps:**
- Center arm and optimize symmetry  
- Replace screws with lighter types  
- Finalize clearances and tolerances  

---

### Kinect

**Issues:**
- Not integrating with Ubuntu  
- No skeletal recognition  

**Achievements:**
- Kinect works on Ubuntu 22  
- Skeleton shows at ~3fps  

**Next Steps:**
- Improve FPS and stability  
- Test with motion capture data  
- Begin MoveIt integration  

---

## Deliverables

**Next Steps:**
- Redesign slides to be visual-heavy  
- Add system flowcharts and diagrams  
- Embed YouTube demos and interaction paths  

---<img width="1299" height="523" alt="Screenshot from 2025-07-31 14-27-20" src="https://github.com/user-attachments/assets/64c4d91f-5cab-4273-aff1-93be8b28a79b" />


## Contributing

Feel free to fork this repository, submit issues, or create pull requests to contribute to the project.
