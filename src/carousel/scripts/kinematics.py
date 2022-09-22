from typing import Sequence
from numpy import ndarray

from math import *

def inverse_kinematics(
        pose : ndarray,
        lengths : Sequence[float]
    ):
    """Calculate the joint positions from an end effector configuration.
    
    Args:
        pose: The end effector configuration.
        lengths: The lengths of each links.

    Returns:
        The angles of each joint to the end effector.
    """

    # return list of thetas
pass



def main():

    inverse_kinematics(None, None)

    pass

if __name__ == '__main__':
    main()
