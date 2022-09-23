from typing import Sequence
from numpy import asarray, eye, sin, cos, ndarray
from math import cos, pi, sin, sqrt
import numpy as np
# Inverse Kinematics
def inverse_kinematics(
        end_effector_pos : list,
        link_length : list
    ):
    """Calculate the joint positions from an end effector configuration.
    
    Args:
        pose: The end effector configuration.
        lengths: The lengths of each links.

    Returns:
        The angles of each joint to the end effector.
    """
    # Define variables (Don't exceed robot arm length)
    # Coordinates of end-effector (cubes)
    x = end_effector_pos[0] # x-coordinate of end-effector
    y = end_effector_pos[1] # y-coordinate of end-effector
    z = end_effector_pos[2] #z-coordinate of end-effector
    #*****^^^This will be replaced by subscriber/publisher***********# 

    alpha = pi/2 # angle of gripper (0 to 90), set to 90

    # Dimentsions of the robot (in mm)
    L1 = link_length[0]
    L2 = link_length[1]
    L3 = link_length[2]
    L4 = link_length[3]

    # Position of joint 3
    px = x - L4*cos(alpha) #joint 3 x coordinate
    py = y - L4*sin(alpha) #joint 3 y coordinate
    pz = z #joint 3 z coordinate

    C_theta_2 = (px**2+py**2-L1**2-L2**2)/(2*L1*L2) 

    #Angle Calculations (in radians)
    theta_1 = np.arctan2(y,x)
    theta_3 = np.arctan2(C_theta_2,-sqrt(1-C_theta_2**2))
    theta_2 = (np.arctan2(py,px)-np.arctan2(L1+L2*cos(theta_3),L2*sin(theta_3)))
    theta_4 = alpha - theta_2 - theta_3
    #Update angles
    theta_2 = (pi/2)-theta_2
    theta_3 = -theta_3

    #Covert to degrees
    theta_1 = np.rad2deg(theta_1)
    theta_2 = np.rad2deg(theta_2)
    theta_3 = np.rad2deg(theta_3)
    theta_4 = np.rad2deg(theta_4)

    # Publish thetas to the robot
    # return list of thetas
    theta_list = [theta_1, theta_2,theta_3,theta_4]
    print(theta_list)
    return theta_list

# Forward Kinematics for testing Inverse Kinematics
def poe(
        M : ndarray,
        twist : Sequence[ndarray],
        theta : Sequence[float],
        *,
        body : bool,
        decomposed : bool = True
    ):
    """Calculate the forward kinematics product of exponentials in the space
    frame of reference.

    Args:
        M: The initial configuration.
        twist: The sequence of twists to perform on the initial configuration.
        theta: The angles of each twist.
        body: True for the body frame or False for the space frame.
        decomposed: True to return (R, p) or false to return T

    Returns:
        The transformation matrix of the final configuration.
    """

    # Choose the direction of iteration over the twists depending on if the
    # twists are in the body frame or space frame.
    if body:
        it = zip(twist, theta)
    else:
        it = reversed(list(zip(twist, theta)))

    # Set the last configuration to the initial configuration.
    T = M

    for V, t in it:
        w, v = V[0:3], V[3:6]

        # Calculate the skew symmetric matrix representation of the twist axis.
        skew = asarray([
            [0, -w[2], w[1]],
            [w[2], 0, -w[0]],
            [-w[1], w[0], 0]
        ])

        sin_t, cos_t = sin(t), cos(t)

        # Rotation matrix in space frame equal to e ** ([w] * theta)
        R = eye(3) + sin_t * skew + (1 - cos_t) * (skew @ skew)

        # Position in space frame.
        p = (eye(3) * t + (1 - cos_t) * skew + (t - sin_t) * (skew @ skew)) @ v

        # Construct the ith transformation matrix.
        Ti, Ti[:3, :3], Ti[:3, 3] = eye(4), R, p

        # Transform the previous configuration by the twist.
        if body:
            T = T @ Ti
        else:
            T = Ti @ T

    if not decomposed:
        return T

    # Return the rotation matrix R and position p in the space frame.
    return T[:3, :3], T[:3, 3]



# Main function for testing 
def main():
    """vv change this for anything within the limit"""
    end_effector_pos = [150,60,70] 
    link_lengths = [75,115,95,85]
    inverse_kinematics(end_effector_pos, link_lengths)


# Output of IK--> Input of FK
# Returns
# Output of FK = Input of IK
    pass

if __name__ == '__main__':
    main()
