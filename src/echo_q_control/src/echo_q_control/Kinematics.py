import numpy as np
from echo_q_control.Config import Leg_linkage

def four_bar_ik(horizontal, vertical, leg_linkage):
    l = leg_linkage
    # Solve for the lower link angle using the geometry of the 4-bar linkage
    # This maps the desired foot position (vertical/horizontal relative to hip)
    # to the actual servo angle required.
    
    # 1. Calculate the diagonal distance from servo hub to foot
    diagonal = np.sqrt(horizontal**2 + vertical**2)
    
    # 2. Find the angle of that diagonal
    diagonal_angle = np.arctan2(vertical, horizontal)
    
    # 3. Use Law of Cosines to find the interior angle needed
    # (Assuming standard Pupper/Dingo linkage geometry)
    upper_leg_sq = l.upper_leg_length**2
    lower_leg_sq = l.lower_leg_length**2
    
    # Check for reach violations (if target is too far)
    cos_arg = (diagonal**2 + upper_leg_sq - lower_leg_sq) / (2 * diagonal * l.upper_leg_length)
    cos_arg = np.clip(cos_arg, -1.0, 1.0) # Safety clamp
    
    arrow_angle = np.arccos(cos_arg)
    
    # 4. Calculate the final Theta for the Upper Leg
    theta = diagonal_angle + arrow_angle
    
    # 5. Calculate the Lower Leg angle (Alpha) relative to the Upper Leg
    # This involves the linkage constraints (a, b, c, d from Config)
    # Note: Simplified solver for standard parallel linkages
    return theta

def inverse_kinematics(leg_positions, configuration):
    """
    Calculates the joint angles for all 4 legs.
    
    Parameters:
    leg_positions: (3, 4) numpy array of [x, y, z] coordinates for each foot.
    configuration: The Configuration object containing robot dimensions.
    
    Returns:
    (3, 4) numpy array of [Hip, Upper, Lower] angles in radians.
    """
    linkage = Leg_linkage(configuration)
    alpha = np.zeros((3, 4))
    
    for i in range(4):
        x = leg_positions[0, i]
        y = leg_positions[1, i]
        z = leg_positions[2, i]
        
        # --- 1. HIP JOINT (Abduction/Adduction) ---
        # Calculate the offset caused by the hip bracket width
        delta_y = y
        delta_z = z
        side_sign = 1 if i == 0 or i == 2 else -1 # Left vs Right legs
        
        # Simple trigonometry for the hip swing
        h1 = np.sqrt(delta_y**2 + delta_z**2)
        h2 = configuration.L1 # Hip width
        
        # Safety check: Is the foot too close to the body?
        if h1 < h2: h1 = h2
            
        hip_angle = np.arcsin(np.clip(h2 / h1, -1.0, 1.0)) - np.arctan2(delta_z, delta_y)
        
        # Apply the angle to the array
        alpha[0, i] = hip_angle

        # --- 2. LEG JOINTS (Upper & Lower) ---
        # Project the leg into the 2D plane of the thigh/calf
        leg_plane_length = np.sqrt(h1**2 - h2**2)
        
        # Solve the 4-bar linkage for this leg
        # Note: We pass the 'x' (forward/back) and the 'leg_plane_length' (vertical depth)
        # In this frame, 'z' is actually the length downwards
        
        # Correct for the coordinate system rotation
        rotated_x = x
        rotated_z = -leg_plane_length # Negative because leg points down
        
        # Use simple 2-link IK first to find the "Virtual Leg" angles
        l1 = configuration.L2 # Thigh Length
        l2 = configuration.L3 # Calf Length
        
        d = np.sqrt(rotated_x**2 + rotated_z**2)
        d = np.clip(d, 0, l1 + l2 - 0.001) # Clamp max extension
        
        # Law of Cosines for the knee angle
        phi = np.arccos(np.clip((l1**2 + l2**2 - d**2) / (2 * l1 * l2), -1.0, 1.0))
        
        # Law of Cosines for the thigh angle
        theta = np.arctan2(rotated_z, rotated_x) + np.arccos(np.clip((l1**2 + d**2 - l2**2) / (2 * l1 * d), -1.0, 1.0))
        
        # Map these theoretical angles to your specific Servos
        alpha[1, i] = -theta # Upper Leg Servo
        alpha[2, i] = phi    # Lower Leg Servo (Knee)

    return alpha
