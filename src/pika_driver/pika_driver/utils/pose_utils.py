#!/usr/bin/env python3
import numpy as np

# Helper: convert position + quaternion to a 4x4 transform matrix
def xyzQuaternion2matrix(x, y, z, qx, qy, qz, qw):
    """
    Convert position and quaternion to a 4x4 transform matrix.
    """
    # Build rotation matrix
    rot_matrix = np.array([
        [1 - 2*qy*qy - 2*qz*qz, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw, 0],
        [2*qx*qy + 2*qz*qw, 1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qx*qw, 0],
        [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx*qx - 2*qy*qy, 0],
        [0, 0, 0, 1]
    ])
    
    # Set translation
    rot_matrix[0, 3] = x
    rot_matrix[1, 3] = y
    rot_matrix[2, 3] = z
    
    return rot_matrix

# Helper: convert xyz position + rpy angles to a 4x4 transform matrix
def xyzrpy2Mat(x, y, z, roll, pitch, yaw):
    """
    Convert xyz position and rpy angles to a 4x4 transform matrix.
    """
    # Build rotation matrix
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    
    rot_matrix = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr, x],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr, y],
        [-sp, cp*sr, cp*cr, z],
        [0, 0, 0, 1]
    ])
    
    return rot_matrix

# Helper: convert a 4x4 transform matrix to position + quaternion
def matrixToXYZQuaternion(matrix):
    """
    Convert a 4x4 transform matrix to position and quaternion.
    """
    # Extract translation
    x = matrix[0, 3]
    y = matrix[1, 3]
    z = matrix[2, 3]
    
    # Extract 3x3 rotation matrix
    rot_matrix = matrix[:3, :3]
    
    # Compute quaternion from rotation matrix
    trace = rot_matrix[0, 0] + rot_matrix[1, 1] + rot_matrix[2, 2]
    
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (rot_matrix[2, 1] - rot_matrix[1, 2]) * s
        qy = (rot_matrix[0, 2] - rot_matrix[2, 0]) * s
        qz = (rot_matrix[1, 0] - rot_matrix[0, 1]) * s
    elif rot_matrix[0, 0] > rot_matrix[1, 1] and rot_matrix[0, 0] > rot_matrix[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rot_matrix[0, 0] - rot_matrix[1, 1] - rot_matrix[2, 2])
        qw = (rot_matrix[2, 1] - rot_matrix[1, 2]) / s
        qx = 0.25 * s
        qy = (rot_matrix[0, 1] + rot_matrix[1, 0]) / s
        qz = (rot_matrix[0, 2] + rot_matrix[2, 0]) / s
    elif rot_matrix[1, 1] > rot_matrix[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rot_matrix[1, 1] - rot_matrix[0, 0] - rot_matrix[2, 2])
        qw = (rot_matrix[0, 2] - rot_matrix[2, 0]) / s
        qx = (rot_matrix[0, 1] + rot_matrix[1, 0]) / s
        qy = 0.25 * s
        qz = (rot_matrix[1, 2] + rot_matrix[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + rot_matrix[2, 2] - rot_matrix[0, 0] - rot_matrix[1, 1])
        qw = (rot_matrix[1, 0] - rot_matrix[0, 1]) / s
        qx = (rot_matrix[0, 2] + rot_matrix[2, 0]) / s
        qy = (rot_matrix[1, 2] + rot_matrix[2, 1]) / s
        qz = 0.25 * s
    
    return x, y, z, qx, qy, qz, qw