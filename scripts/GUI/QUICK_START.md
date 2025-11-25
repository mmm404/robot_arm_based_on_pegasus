# Quick Start - Fixed Controls

## How to Use (After Fixes)

### Joint Space Control ✅

```
1. Open GUI → Goes to "Joint Control" tab by default
2. Select a joint from dropdown (or use slider directly)
3. Drag ANY joint slider left/right
   → Arm joint IMMEDIATELY moves to new angle
   → Real-time in RViz
   → NO repetitive motion
4. Release slider
   → Arm holds position
5. Drag slider to new position
   → Arm smoothly moves to NEW position
```

**Key Points**:
- Joint sliders work in both Arduino and MoveIt modes
- Non-blocking: GUI always responsive
- Multiple joints can be controlled via different sliders

---

### Cartesian Space Control ✅

```
1. Switch to "Cartesian Space" mode (radio button)
2. Drag X/Y/Z sliders
   → Pose updated every 100ms maximum (debounced)
   → Inverse kinematics solved automatically
   → Trajectory planned and shown in RViz
3. Release slider
   → Motion stops
4. Drag slider to new position
   → NEW trajectory planning starts
   → NO repetition of old pose
```

**Key Points**:
- Debouncing prevents motion spam (100ms minimum between plans)
- IK falls back to simulated algorithm if MoveIt unavailable
- Workspace clamping prevents unreachable poses
- Orientation (Roll/Pitch/Yaw) optional via checkbox

---

## Control Flow (Fixed)

### Joint Space
```
Slider Drag
  ↓
slider_pressed() → slider_active[idx] = True
  ↓
handle_slider_continuous() checks active flag
  ↓
Get joint value → Clamp to limits
  ↓
Check commander exists?
  ├─ YES: Send to commander (Arduino or MoveIt)
  └─ NO: Skip (commander initializing)
  ↓
Update display label
  ↓
Background thread executes (non-blocking)
  ↓
slider_released() → slider_active[idx] = False
```

### Cartesian Space
```
Slider Drag
  ↓
cartesian_slider_pressed() → cartesian_slider_active[idx] = True
  ↓
handle_cartesian_slider_continuous() called on value change
  ↓
Debounce check: skip if < 100ms since last plan
  ↓
Get X,Y,Z values → Clamp to workspace
  ↓
Solve IK → Get joint angles
  ↓
Plan trajectory → Publish to RViz
  ↓
Execute trajectory (animation only)
  ↓
Background thread completes (non-blocking)
  ↓
slider_released() → cartesian_slider_active[idx] = False
```

---

## Log Output (Expected)

### Good - Joint Space
```
[DEBUG] Error in slider continuous update: (empty = no error)
(arm smoothly moves to slider position)
```

### Good - Cartesian Space
```
[INFO] Planning to Cartesian pose: x=0.3, y=0.0, z=0.2...
[INFO] Simulated IK solution: [...joint angles...]
[INFO] Published planned trajectory for visualization
[INFO] Controller unavailable; using simulated trajectory playback
(arm moves to pose, stops, waits for new slider input)
```

### Bad - Old Behavior (NOW FIXED)
```
[INFO] Planning to Cartesian pose: x=0.3, y=0.0, z=0.2
[INFO] Planning to Cartesian pose: x=0.3, y=0.0, z=0.201  ← Same pose!
[INFO] Planning to Cartesian pose: x=0.3, y=0.0, z=0.202  ← Repetition!
[INFO] Planning to Cartesian pose: x=0.3, y=0.0, z=0.203
...
(arm continuously moves to slightly different poses - AVOIDED)
```

---

## Configuration

### Debounce Timing
Change Cartesian slider debounce (in `__init__`):
```python
self.cartesian_plan_cooldown = 0.1  # 100ms (default)
# Smaller = more responsive but higher CPU
# Larger = less responsive but smoother
```

### Joint Limits
Edit in PegasusCommander:
```python
self.joint_limits = {
    'joint1': (-6.28, 6.28),
    'joint2': (-0.61, 0.61),
    'joint3': (-1.75, 1.75),
    'joint4': (-1.31, 1.31),
    'joint5': (-6.28, 6.28)
}
```

### Workspace Limits
Edit at top of GUI_commander.py:
```python
WORKSPACE_LIMITS = {
    'x': (0.05, 0.65),
    'y': (-0.40, 0.40),   
    'z': (0.05, 0.60),
    'roll': (-3.14, 3.14),
    'pitch': (-1.57, 1.57),
    'yaw': (-3.14, 3.14)
}
```

---

## Troubleshooting

| Problem | Cause | Fix |
|---------|-------|-----|
| Joint sliders still not working | Commander not initialized yet | Wait 5 seconds, try again |
| Joint movement slow | Velocity slider too low | Increase velocity slider |
| Cartesian still repeating | Debounce value too low | Increase `cartesian_plan_cooldown` |
| IK errors in console | MoveIt service down | Normal, falls back to simulated IK |
| Arm jerky movement | Planning too fast | Increase debounce to 0.2s |

---

## Testing Commands

```bash
# Terminal: Check ROS2 is running
ros2 node list

# Should show:
# /robot_state_publisher
# /rviz2
# /pegasus_commander (once GUI connects)

# Check topics
ros2 topic list | grep -E "joint_states|tf"

# Monitor joint states
ros2 topic echo /joint_states
```

---

## Summary

✅ **Joint Space**: Sliders now directly control arm joints  
✅ **Cartesian Space**: Single smooth motion per slider position  
✅ **No IK Errors**: Graceful fallback to simulated IK  
✅ **Non-blocking**: GUI always responsive  
✅ **Real-time**: RViz visualization updates immediately  

**Ready to use!** 🚀
