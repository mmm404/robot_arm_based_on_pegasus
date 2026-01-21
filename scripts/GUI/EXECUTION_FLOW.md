# Pegasus Arm Full Stack Execution Flow

## Complete Startup Sequence

### **STEP 1: ROS2 Description** (Terminal 1)
```bash
ros2 launch update_pegasus_description demo.launch.py
```
**Output:**
- ✅ RViz window opens
- ✅ `/tf` publisher active (robot_state_publisher)
- ✅ `/joint_states` topic available
- ✅ MoveIt2 services ready: `/compute_ik`, `/compute_fk`, etc.

---

### **STEP 2: Arduino Detection & Bridge Setup**
**Detects:**
```bash
if [ -e /dev/ttyUSB0 ]; then BRIDGE_MODE="bridge"; else BRIDGE_MODE="simulation"; fi
```

---

### **STEP 3: Arduino Bridge** (Background)
```bash
BRIDGE_MODE=$BRIDGE_MODE ./arduino_bridge_simulator.sh
```
**Simulation Mode Creates:**
- `/tmp/virtual_arduino_sim` → Virtual serial port
- `/tmp/virtual_monitor_sim` → Monitor FIFO
- `/tmp/arduino_sim.log` → Debug log

---

### **STEP 4: Monitor** (Terminal 2)
```bash
gnome-terminal -- cat /tmp/virtual_monitor_sim
```
**Shows:** Real-time bridge output

---

### **STEP 5: GUI Commander** (Terminal 3)
```bash
source ~/ros2-ws/install/setup.bash
export ROS_DOMAIN_ID=0
python3 ~/ros2-ws/scripts/GUI/GUI_commander.py
```

**Execution Order:**
1. PegasusArmGUI.__init__() → GUI appears immediately (non-blocking)
2. PegasusCommander.__init__() in background thread
3. ros_spin_thread() in daemon thread
4. GUI fully responsive

---

### **STEP 6: Listener** (Terminal 4)
```bash
python3 ~/ros2-ws/src/update_pegasus_description/scripts/planned_path_listener.py
```
**Monitors:** `/display_planned_path` topic

---

## Data Flow for Cartesian Control

```
User moves X slider
    ↓
handle_cartesian_slider_continuous(idx)
    ↓
if slider_active[idx]: execute in background thread
    ↓
commander.move_to_pose([x,y,z,roll,pitch,yaw], velocity)
    ↓
plan_to_cartesian_pose(x, y, z, roll, pitch, yaw)
    ↓
solve_ik() → RViz shows motion in real-time
    ↓
plan_to_joint_values(joint_solution)
    ↓
trajectory_pub.publish(traj_msg)
    ↓
execute_live_trajectory()
    ↓
play_trajectory() in background
    ↓
joint_state_pub.publish() → RViz animates
```

---

## Thread Safety Architecture

✅ **GUI Thread** (Main Tkinter loop)
- Always responsive
- Slider events trigger background threads

✅ **Commander Thread** (Background initialization)
- Initializes ROS2 node
- Loads joint limits
- Creates service clients

✅ **ROS Spin Thread** (Daemon)
- `rclpy.spin(commander)`
- Processes callbacks

✅ **Planning Threads** (On-demand)
- `handle_cartesian_slider_continuous()`
- `handle_slider_continuous()`
- `play_trajectory()`
- Each in separate thread via `threading.Thread(..., daemon=True)`

---

## Method Call Verification

| Method | Called From | Purpose | Return Value |
|--------|------------|---------|--------------|
| `GUI_commander.py: main()` | Shell script | Entry point | Program startup |
| `PegasusArmGUI.__init__()` | main() | Create GUI | GUI window |
| `PegasusCommander.__init__()` | Background thread | ROS2 init | Commander node |
| `move_to_pose()` | `handle_cartesian_slider_continuous()` | Plan Cartesian | (success, error_code) |
| `plan_to_cartesian_pose()` | `move_to_pose()` | IK + planning | {"success": bool, ...} |
| `solve_ik()` | `plan_to_cartesian_pose()` | Inverse kinematics | [j1, j2, j3, j4, j5] |
| `plan_to_joint_values()` | `plan_to_cartesian_pose()` | Create trajectory | {"success": bool, "raw_trajectory": msg} |
| `execute_live_trajectory()` | `plan_to_cartesian_pose()` | Execute motion | None (async) |
| `play_trajectory()` | `execute_live_trajectory()` | Animate in RViz | None (daemon thread) |

---

## Key Fixes Applied

✅ **Removed duplicate methods:**
- `move_to_pose()` (was defined twice)
- `close_error_window()` (was defined twice)
- `set_moveit_enabled()` (was defined twice)

✅ **Fixed method signatures:**
- `move_to_pose(pose, velocity_scaling)` returns `(bool, int)`
- Compatible with GUI calling convention

✅ **Fixed return types:**
- `plan_to_cartesian_pose()` returns dict with "success" and "message"
- `execute_live_trajectory()` properly handles dict input

✅ **Environment setup in shell:**
- `export ROS_DOMAIN_ID=0`
- `export ROBOT_PORT` set correctly
- All terminals inherit environment

---

## Testing Flow

1. **Start full stack:** `./run_full_stack.sh`
2. **Wait 15 seconds** for ROS2 to initialize
3. **GUI appears** and is immediately responsive
4. **Move joint sliders** → Arm moves in RViz in real-time
5. **Switch to Cartesian mode** → New control interface
6. **Move X slider** → IK solves, trajectory shown, arm animates
7. **Check monitor** → See bridge activity
8. **Check logs** → `/tmp/ros2_node_output.log` shows command flow

---

## Non-Blocking Architecture

```
run_full_stack.sh (blocking wait for Ctrl+C)
├─ ROS2 Description (terminal 1) - Blocking GUI process
├─ Arduino Bridge (background) - Non-blocking daemon
├─ Monitor (terminal 2) - Non-blocking cat process
├─ GUI Commander (terminal 3)
│  ├─ Main: Tkinter mainloop (blocking, but responsive)
│  ├─ Background: Socket server init (non-blocking)
│  ├─ Background: Commander init (non-blocking)
│  │  └─ Background: ROS spin (daemon, non-blocking)
│  └─ Background: Planning threads (on-demand, non-blocking)
└─ Listener (terminal 4) - Non-blocking daemon
```

**Result:** GUI always responsive ✅

---

## Environment Variables at Runtime

When GUI_commander.py starts:
```bash
ROS_DOMAIN_ID=0
ROBOT_PORT=/tmp/virtual_arduino_sim  # (or /dev/ttyUSB0)
PYTHONPATH=~/ros2-ws/install/lib/python3.x/site-packages:...
```

These allow:
- ✅ ROS2 communication on correct domain
- ✅ Arduino serial port access
- ✅ Import of all ROS2 packages
- ✅ Import of pegasus_commander module

---

## Verification Commands

```bash
# Terminal 1: Check ROS2 initialization
ros2 node list
# Should show: /robot_state_publisher, /rviz2

# Terminal 2: Check topics
ros2 topic list
# Should show: /joint_states, /tf, /tf_static

# Terminal 3: Check services
ros2 service list
# Should show: /compute_ik, /compute_fk, /plan_kinematic_path

# Terminal 4: Check socket server
nc -zv localhost 5000
# Should show: Connection successful
```

---

## Summary

✅ **All code properly called**
✅ **Correct method signatures**
✅ **Proper thread safety**
✅ **Non-blocking GUI**
✅ **Complete error handling**
✅ **Real-time RViz visualization**
✅ **Cartesian and Joint control working**
