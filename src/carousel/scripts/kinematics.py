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
    pass
    # return thetas
