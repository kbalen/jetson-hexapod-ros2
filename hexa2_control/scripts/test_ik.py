#!/usr/bin/env python3
import math
import sys

def inverse_kinematics(leg_name, target_position):
    """
    Compute inverse kinematics for a leg.
    
    Args:
        leg_name (str): Name of the leg (e.g., 'front_left')
        target_position (list): Target foot position [x, y, z] in the robot's body frame
        
    Returns:
        list: Joint angles [coxa_angle, femur_angle, tibia_angle] in radians
    """
    # Define robot dimensions (in meters)
    coxa_length = 0.05
    femur_length = 0.10
    tibia_length = 0.12

    # Define leg base positions (relative to the robot's body center)
    leg_positions = {
        'front_left':  [ 0.110,  0.080, 0.0],
        'middle_left': [ 0.000,  0.130, 0.0],
        'back_left':   [-0.110,  0.080, 0.0],
        'back_right':  [-0.110, -0.080, 0.0],
        'middle_right':[ 0.000, -0.130, 0.0],
        'front_right': [ 0.110, -0.080, 0.0],
    }

    # Define leg orientations (in radians)
    leg_orientations = {
        'front_left':   math.pi/4,    # 45 degrees
        'middle_left':  math.pi/2,    # 90 degrees
        'back_left':    3*math.pi/4,  # 135 degrees
        'back_right':   -3*math.pi/4, # -135 degrees
        'middle_right': -math.pi/2,   # -90 degrees
        'front_right':  -math.pi/4,   # -45 degrees
    }

    if leg_name not in leg_positions or leg_name not in leg_orientations:
        print(f"Unknown leg name '{leg_name}' for IK computation", file=sys.stderr)
        return [0.0, 0.0, 0.0]

    # Get the leg's mounting position and orientation
    base_pos = leg_positions[leg_name]
    orientation = leg_orientations[leg_name]

    # Transform the target position from the body frame to the leg's local frame
    local_x = target_position[0] - base_pos[0]
    local_y = target_position[1] - base_pos[1]
    local_z = target_position[2] - base_pos[2]

    # Compute the distance in the horizontal (XY) plane
    len_xy = math.sqrt(local_x**2 + local_y**2)

    # Compute the coxa angle: the angle in the horizontal plane from the leg's base
    coxa_angle = math.atan2(local_y, local_x) - orientation

    # Adjust the horizontal distance by subtracting the coxa length
    foot_offset_x = len_xy - coxa_length

    # Compute the distance from the femur joint to the target foot position
    leg_length = math.sqrt(foot_offset_x**2 + local_z**2)

    # Check if the target is reachable; if not, clamp leg_length
    if leg_length > (femur_length + tibia_length):
        print(f"Target position unreachable for leg {leg_name}, clamping distance", file=sys.stderr)
        leg_length = femur_length + tibia_length - 0.001

    # Use the law of cosines to compute the tibia angle
    cos_angle = (femur_length**2 + tibia_length**2 - leg_length**2) / (2 * femur_length * tibia_length)
    # Clamp to valid range in case of numerical issues
    cos_angle = max(-1.0, min(1.0, cos_angle))
    tibia_angle = -(math.pi - math.acos(cos_angle))

    # Compute the femur angle
    gamma = math.atan2(local_z, foot_offset_x)
    cos_beta = (femur_length**2 + leg_length**2 - tibia_length**2) / (2 * femur_length * leg_length)
    cos_beta = max(-1.0, min(1.0, cos_beta))
    beta = math.acos(cos_beta)
    femur_angle = -(gamma + beta)

    return [coxa_angle, femur_angle, tibia_angle]

def test_inverse_kinematics():
    # Define some sample target positions (in meters) relative to the robot's body frame
    test_positions = [
        [0.05, 0.0, -0.1],
        [0.1, 0.0, -0.05],
        [0.0, 0.0, -0.15],
    ]

    # List of legs to test
    legs = ['front_left', 'middle_left', 'back_left', 'back_right', 'middle_right', 'front_right']

    for leg in legs:
        print(f"Testing IK for leg: {leg}")
        for pos in test_positions:
            angles = inverse_kinematics(leg, pos)
            angles_deg = [angle * 180 / math.pi for angle in angles]
            print(f"  Target position {pos} => Joint angles (rad): {angles}")
            print(f"                          => Joint angles (deg): {angles_deg}")
        print("")

if __name__ == '__main__':
    test_inverse_kinematics()
