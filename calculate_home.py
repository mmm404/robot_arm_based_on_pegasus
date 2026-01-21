
import math
import numpy as np

def calculate_ik(x, y, z):
    # Link lengths from pegasus_commander.py
    L2 = 0.3  # upper arm
    L3 = 0.3  # forearm

    # Joint 1 (Base)
    joint1 = math.atan2(y, x)

    # Project to arm plane
    px = math.sqrt(x**2 + y**2)
    pz = z

    # Cosine rule for elbow (Joint 3)
    # c^2 = a^2 + b^2 - 2ab cos(C)
    # Reach^2 = L2^2 + L3^2 - 2*L2*L3*cos(180 - theta3)
    # cos_theta3 = (Reach^2 - L2^2 - L3^2) / (2*L2*L3)  <-- This is for internal angle
    # In the code: cos_theta3 = (px**2 + pz**2 - L2**2 - L3**2) / (2 * L2 * L3)
    
    cos_theta3 = (px**2 + pz**2 - L2**2 - L3**2) / (2 * L2 * L3)
    cos_theta3 = np.clip(cos_theta3, -1.0, 1.0)
    theta3 = math.acos(cos_theta3)
    
    # Elbow down configuration (typical for this type of arm)
    theta3 = -theta3 

    sin_theta3 = math.sin(theta3)

    # Joint 2 (Shoulder)
    # theta2 = atan2(pz, px) - atan2(L3*sin3, L2 + L3*cos3)
    k1 = L2 + L3 * cos_theta3
    k2 = L3 * sin_theta3
    theta2 = math.atan2(pz, px) - math.atan2(k2, k1)

    # Joint 4 (Wrist Pitch)
    # For pick and place, we usually want the gripper pointing DOWN.
    # Global pitch = theta2 + theta3 + theta4
    # We want Global Pitch to be as close to -PI/2 (-1.57) as possible.
    # theta4 = -1.57 - theta2 - theta3
    
    target_pitch = -1.57
    joint4 = target_pitch - theta2 - theta3
    
    # Clamp Joint 4 to limits (-1.31, 1.31)
    joint4 = np.clip(joint4, -1.31, 1.31)

    # Joint 5 (Wrist Roll)
    joint5 = 0.0

    return [joint1, theta2, theta3, joint4, joint5]

# Target: Center of workspace
target_x = 0.35
target_y = 0.0
target_z = 0.325

joints = calculate_ik(target_x, target_y, target_z)
print(f"Calculated Joints for ({target_x}, {target_y}, {target_z}):")
print(f"[{joints[0]:.4f}, {joints[1]:.4f}, {joints[2]:.4f}, {joints[3]:.4f}, {joints[4]:.4f}]")
