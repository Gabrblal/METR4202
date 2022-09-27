from numpy import ndarray, asarray, eye, sin, cos
from typing import Sequence

def poe(
        M : ndarray,
        twist : Sequence[ndarray],
        theta : Sequence[float],
        *,
        body : bool,
        decomposed : bool = True
    ):
    """Calculate the forward kinematics product of exponentials.

    Args:
        M: The initial configuration.
        twist: The sequence of twists to perform on the initial configuration.
        theta: The theta values of each twist.
        body: True for the body frame or False for the space frame.
        decomposed: True to return a tuple of the rotation matrix and the
            translation vector, or False to return the transformation matrix.

    Returns:
        The transformation matrix of the final configuration, or a tuple of
        the rotation matrix and the translation vector.
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
