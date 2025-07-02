# robot_arm_based_on_pegasus
This project handles a 5-DOF robot exploiting ROS2


starting RVIZ    ->  ros2 launch pegasus_arm_description display.launch.py
starting MoveIT  ->  ros2 launch pegasus_arm_moveit pegasus_moveit.launch.py

RVIZ model is as the diagram; 
![image](https://github.com/user-attachments/assets/75f51dc3-2cc1-4eb1-80a3-c336d12a35d3)
file:///home/mmms/Pictures/rosgraph.png

![image](https://github.com/user-attachments/assets/2d9f290c-c92e-4738-b07e-bbec099e564b)


activating a python3 workspace:

# From any directory, activate your venv
source ~/arduino_communication/venv/bin/activate

# Now you can work from any directory
cd ~/Documents/some_project
python ~/arduino_communication/vscode_sender.py



Launching MoveIt:
ros2 launch pegasus_arm_description moveit_display.launch.py

Running the controller:
ros2 run pegasus_arm_commander pegasus_commander

Cleaning the workspace:
rm -rf install build src

creating virtual linked ports:

socat -d -d PTY,link=/tmp/ttyV0,raw,echo=0 PTY,link=/tmp/ttyV1,raw,echo=0
 #this creates ttyV0 and ttyV1
 
 ---commands on putty ; 
	Connection type: Serial
	Serial line: /tmp/ttyV1
	Speed: 9600




cd ~/ros2-ws/Sim
colcon build --symlink-install
source install/setup.bash
ros2 launch pegasus_arm_description moveit_display.launch.py
ros2 run pegasus_arm_description pegasus_commander.py
