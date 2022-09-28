from functools import reduce
from typing import Sequence, Tuple

from numpy import ndarray, asarray, eye, sin, cos, arccos, tan, trace, sqrt
from numpy import zeros, pi
from numpy.linalg import norm

def texp(S : ndarray, theta : float, *, decomposed : bool = False):
    """Calculate the transformation matrix exponential e ** ([S] * theta).

    Args:
        S: The screw axis.
        theta: The transformation matrix parameter.
        decomposed: True to return a tuple of the rotation matrix and the
            translation vector, or False to return the transformation matrix.

    Returns:
        If decomposed is True then a tuple of the rotation matrix and
        translation vector (R, p), or if decomposed is False then the
        transformation matrix.
    """
    w, v = S[0:3], S[3:6]

    # Calculate the skew symmetric matrix representation of the twist axis.
    skew = asarray([
        [0, -w[2], w[1]],
        [w[2], 0, -w[0]],
        [-w[1], w[0], 0]
    ])

    sin_t, cos_t = sin(theta), cos(theta)

    # Rotation matrix equal to e ** ([w] * theta)
    R = eye(3) + sin_t * skew + (1 - cos_t) * (skew @ skew)

    # Translation.
    p = (eye(3) * theta + (1 - cos_t) * skew + (theta - sin_t) * (skew @ skew)) @ v

    if decomposed:
        return R, p

    # Construct the transformation matrix.
    T, T[:3, :3], T[:3, 3] = eye(4), R, p
    return T

def tlog(*T : ndarray | Tuple[ndarray, ndarray]):
    """Calculate the matrix logaritm of a transformation matrix.

    Args:
        T: Either the 4x4 transformation matrix, or a 3x3 rotation matrix
            and a 3x1 translation vector, to take the logarithm of.

    Returns:
        A tuple (w, v, theta) of the rotation axis, linear velocity and
        theta. 
    """
    # Get the rotation matrix and translation vector.
    try:
        R, p = T
    except Exception:
        R, p = T[0][:3, :3], T[0][:3, 3]

    # If the rotation is identity then there is no axis of rotation, but
    # travel the distance |p| in the direction of p.
    if R == eye(3):
        return zeros(3), p / norm(p), norm(p)

    # If the rotation is pi.
    if trace(R) == -1:
        theta = pi
        w = 1 / sqrt(2 * (1 + R[2, 2])) * (R[:, 2] + asarray([0, 0, 1]))

    # Otherwise calculate the angle theta, the skew symmetric matrix and extract
    # the rotation axis.
    else:
        theta = arccos((trace(R) - 1) / 2)
        skew = 1 / (2 * sin(theta)) * (R - R.T)
        w = asarray([skew[1, 2], skew[0, 2], skew[1, 0]])

    Gi = (
        1 / theta * eye(3) -
        0.5 * skew +
        (1 / theta - 0.5 * 1 / tan(theta / 2)) * (skew @ skew)
    )

    v = Gi @ p

    return w, v, theta

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
    # Choose the direction of iteration and order of operation.
    if body:
        it = zip(twist, theta)
        op = lambda a, b: a @ texp(*b)
    else:
        it = reversed(list(zip(twist, theta)))
        op = lambda a, b: texp(*b) @ a

    # Calculate the transformation matrix.
    T = reduce(op, it, M)

    # Return the transformation matrix if not decomposed.
    if not decomposed:
        return T

    # Otherwise decompose in the matrix R and translation p.
    return T[:3, :3], T[:3, 3]
