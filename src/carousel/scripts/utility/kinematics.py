from functools import reduce
from itertools import accumulate
from typing import Sequence, Tuple, Union

from numpy import ndarray, asarray, eye, sin, cos, arccos, tan, trace, sqrt
from numpy import zeros, pi, concatenate, vstack, abs, arctan2
from numpy.linalg import norm, pinv

def mlog(R : ndarray, *, decomposed = False):
    """Calculate the logarithm of a rotation matrix.
    
    Args:
        R: The rotation matrix.

    Returns:
        The 3x3 skew symmetric logarithmic matrix.
    """

    # If the rotation is identity then there is no rotation.
    if (R == eye(3)).all():
        return R

    # If the rotation is pi.
    if trace(R) == -1:
        theta = pi
        w = 1 / sqrt(2 * (1 + R[2, 2])) * (R[:, 2] + asarray([0, 0, 1]))
        skew = asarray([
            [0,    -w[2],  w[1]],
            [w[2],    0,  -w[0]],
            [-w[1], w[0],     0]
        ])

    # Otherwise calculate the angle theta, the skew symmetric matrix and extract
    # the rotation axis.
    else:
        theta = arccos((trace(R) - 1) / 2)
        skew = 1 / (2 * sin(theta)) * (R - R.T)
        w = asarray([skew[2, 1], skew[0, 2], skew[1, 0]])

    if decomposed:
        return w, theta

    return skew * theta

def mexp(w : ndarray, theta : float):

    skew = asarray([
        [0, -w[2], w[1]],
        [w[2], 0, -w[0]],
        [-w[1], w[0], 0]
    ])

    sin_t, cos_t = sin(theta), cos(theta)

    # Rotation matrix equal to e ** ([w] * theta)
    return eye(3) + sin_t * skew + (1 - cos_t) * (skew @ skew)

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
    assert len(S) == 6

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

def tlog(*T : Union[ndarray, Tuple[ndarray, ndarray]], vector = False):
    """Calculate the matrix logaritm of a transformation matrix.

    Args:
        T: Either the 4x4 transformation matrix, or a 3x3 rotation matrix
            and a 3x1 translation vector, to take the logarithm of.
        vector: True to return a screw vector V, or False to return
            The transformation matrix.

    Returns:
        A screw V of the transformation if vector is True, or the transformation
        matrix if vector is False.
    """
    # Get the rotation matrix and translation vector.
    try:
        R, p = T
    except ValueError:
        R, p = T[0][:3, :3], T[0][:3, 3]

    # If the rotation is identity then there is no axis of rotation, but
    # travel the distance |p| in the direction of p.
    if (R == eye(3)).all():
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

    # array[None] converts a 1D array to a 2D array.
    if vector:
        return concatenate((w * theta, v * theta))[None].T

    # Transformation matrix given by the skew symmatrix rotation matrix and
    # the translation vector p.
    return asarray([
        [0,    -w[2],  w[1], p[0]],
        [w[2],    0,  -w[0], p[1]],
        [-w[1], w[0],     0, p[2]],
        [0,        0,     0,    0]
    ]) * theta

def tinv(T):
    """Calculate the inverse of a transformation matrix.

    Args:
        T: The transformation matrix to take the inverse of.

    Returns:
        The inverse transformation matrix.
    """
    Rt, p = T[:3, :3].T, T[:3, 3]
    Ti, Ti[:3, :3], Ti[:3, 3] = eye(4), Rt, -Rt @ p
    return Ti

def tadj(*T : Union[ndarray, Tuple[ndarray, ndarray]]):
    """Calculate the adjoint representation of a transformation matrix.

    Args:
        T: Either a transformation matrix, or a tuple of the rotation matrix and
            the translation vector.

    Returns:
        The adjoint representation of the transformation matrix.
    """
    try:
        R, p = T
    except ValueError:
        R, p = T[0][:3, :3], T[0][:3, 3]

    adj = zeros((6, 6))
    adj[:3, :3] = adj[3:, 3:] = R
    adj[3:, :3] = asarray([
        [0, -p[2], p[1]],
        [p[2], 0, -p[0]],
        [-p[1], p[0], 0]
    ]) @ R

    return adj

def jacobian(
        screws : Sequence[ndarray],
        thetas : Sequence[float],
        body = False
    ):
    """Calculate the jacobian from a sequence of screw and screw parameters.

    Args:
        screw: The number of joints in the arm.
        thetas: The parameters of the screw.

    Returns:
        The jacobian of the screws.
    """
    if body:
        it1 = reversed([[screw, -theta] for screw, theta in zip(screws, thetas)])
    else:
        it1 = zip(screws, thetas)

    transforms = list(accumulate(
        (texp(screw, theta) for screw, theta in it1),
        lambda a, b: a @ b
    ))

    if body:
        it2 = reversed(list(zip(transforms, reversed(screws))))
    else:
        it2 = zip(transforms, screws)

    return vstack([tadj(T) @ S for T, S in it2]).T

def poe(
        M : ndarray,
        screws : Sequence[ndarray],
        theta : Sequence[float],
        *,
        body : bool = False,
        decomposed : bool = True
    ):
    """Calculate the forward kinematics product of exponentials.

    Args:
        M: The initial configuration.
        screws: The sequence of twists to perform on the initial configuration.
        theta: The theta values of each screw.
        body: True for the body frame or False for the space frame.
        decomposed: True to return a tuple of the rotation matrix and the
            translation vector, or False to return the transformation matrix.

    Returns:
        The transformation matrix of the final configuration, or a tuple of
        the rotation matrix and the translation vector.
    """
    # Expect a 2D array with 6 columns, or a list of 6 element sequences.
    assert isinstance(screws, ndarray)
    assert len(screws.shape) == 2
    assert screws.shape[1] == 6

    # Choose the direction of iteration and order of operation.
    if body:
        it = zip(screws, theta)
        op = lambda a, b: a @ texp(*b)
    else:
        it = reversed(list(zip(screws, theta)))
        op = lambda a, b: texp(*b) @ a

    # Calculate the transformation matrix.
    T = reduce(op, it, M)

    # Return the transformation matrix if not decomposed.
    if not decomposed:
        return T

    # Otherwise decompose in the matrix R and translation p.
    return T[:3, :3], T[:3, 3]

def newton_raphson(
        screws : ndarray,
        M : ndarray,
        T : ndarray,
        theta : ndarray,
        *,
        w_tolerance : float = 1e-3,
        v_tolerance : float = 1e-3,
        body = False,
        max_iterations : int = 100
    ):
    """Perform newton raphson inverse kinematics numerical analysis.

    Args:
        screw: The definitions of the joint screws.
        M: The initial configuration of the joints.
        theta: The initial joint variables of the screws, as close to desired
            as possible for fast and accurate inverse kinematics.
        T: The desired end effector configuration.
        w_tolerance: The threshold for which the axi
        v_tolerance: The threshold
        body: True if the screws are in the body frame, or False if they are
            in the space frame.
        max_iterations: The maximum number of iterations to perform.

    Returns:
        The joint parameters theta of the screws to the desired configuration
        on convergence, or None if failed to converge.
    """
    assert isinstance(screws, ndarray)
    assert len(screws.shape) == 2
    assert screws.shape[1] == 6
    assert isinstance(theta, ndarray)
    assert len(theta.shape) == 2
    assert theta.shape[1] == 1

    for _ in range(max_iterations):
        Ti = poe(M, screws, theta, body = body, decomposed = False)
        V = tlog(tinv(Ti) @ T, vector = True)

        if not body:
            V = tadj(Ti) @ V

        v, w = V[:3], V[3:]
        if norm(w) < w_tolerance and norm(v) < v_tolerance:
            return theta

        theta = theta + pinv(jacobian(screws, theta, body = body)) @ V

    return None

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
    theta_1 = arctan2(y,x)
    theta_3 = arctan2(C_theta_2,-sqrt(1-C_theta_2**2))
    theta_2 = (arctan2(py,px)-arctan2(L1+L2*cos(theta_3),L2*sin(theta_3)))
    theta_4 = alpha - theta_2 - theta_3
    #Update angles
    theta_2 = (pi/2)-theta_2
    theta_3 = -theta_3

    # Publish thetas to the robot
    # return list of thetas
    return [theta_1, theta_2, theta_3,theta_4]

def test_space_jacobian():
    screws = asarray([
        [0, 0, 1,   0, 0.2, 0.2],
        [1, 0, 0,   2,   0,   3],
        [0, 1, 0,   0,   2,   1],
        [1, 0, 0, 0.2, 0.3, 0.4]])
    theta = asarray([0.2, 1.1, 0.1, 1.2])

    result = jacobian(screws, theta, body = False)
    expected = asarray([
        [  0, 0.98006658, -0.09011564,  0.95749426],
        [  0, 0.19866933,   0.4445544,  0.28487557],
        [  1,          0,  0.89120736, -0.04528405],
        [  0, 1.95218638, -2.21635216, -0.51161537],
        [0.2, 0.43654132, -2.43712573,  2.77535713],
        [0.2, 2.96026613,  3.23573065,  2.22512443]
    ])

    if not (result - expected < 1e-5).all():
        raise AssertionError(f"\n{result}\n!=\n{expected}")

def test_body_jacobian():
    screws = asarray([
        [0, 0, 1,   0, 0.2, 0.2],
        [1, 0, 0,   2,   0,   3],
        [0, 1, 0,   0,   2,   1],
        [1, 0, 0, 0.2, 0.3, 0.4]
    ])
    theta = asarray([0.2, 1.1, 0.1, 1.2])

    result = jacobian(screws, theta, body = True)
    expected = asarray([
        [-0.04528405, 0.99500417,           0,   1],
        [ 0.74359313, 0.09304865,  0.36235775,   0],
        [-0.66709716, 0.03617541, -0.93203909,   0],
        [ 2.32586047,    1.66809,  0.56410831, 0.2],
        [-1.44321167, 2.94561275,  1.43306521, 0.3],
        [-2.06639565, 1.82881722, -1.58868628, 0.4]
    ])

    if not (result - expected < 1e-5).all():
        raise AssertionError(f"\n{result}\n!=\n{expected}")

def test_poe_space():

    M = asarray([
        [-1, 0,  0, 0],
        [ 0, 1,  0, 6],
        [ 0, 0, -1, 2],
        [ 0, 0,  0, 1]
    ])
    screws = asarray([
        [0, 0,  1,  4, 0,    0],
        [0, 0,  0,  0, 1,    0],
        [0, 0, -1, -6, 0, -0.1]
    ])
    theta = asarray([pi / 2.0, 3, pi])

    result = poe(M, screws, theta, body = False, decomposed = False)
    expected = asarray([
        [0, 1,  0,         -5],
        [1, 0,  0,          4],
        [0, 0, -1, 1.68584073],
        [0, 0,  0,          1]
    ])

    if not (result - expected < 1e-5).all():
        raise AssertionError(f"\n{result}\n!=\n{expected}")

def test_poe_body():

    M = asarray([
        [-1, 0,  0, 0],
        [ 0, 1,  0, 6],
        [ 0, 0, -1, 2],
        [ 0, 0,  0, 1]
    ])
    screws = asarray([
        [0, 0, -1, 2, 0,   0],
        [0, 0,  0, 0, 1,   0],
        [0, 0,  1, 0, 0, 0.1]
    ])
    theta = asarray([pi / 2.0, 3, pi])

    result = poe(M, screws, theta, body = True, decomposed = False)
    expected = asarray([
        [0, 1,  0,         -5],
        [1, 0,  0,          4],
        [0, 0, -1, 1.68584073],
        [0, 0,  0,          1]
    ])

    if not (result - expected < 1e-5).all():
        raise AssertionError(f"\n{result}\n!=\n{expected}")

def test_newton_raphson_space():

    screws = asarray([
        [0, 0,  1,  4, 0,    0],
        [0, 0,  0,  0, 1,    0],
        [0, 0, -1, -6, 0, -0.1]
    ])
    M = asarray([
        [-1, 0,  0, 0],
        [ 0, 1,  0, 6],
        [ 0, 0, -1, 2],
        [ 0, 0,  0, 1]
    ])
    T = asarray([
        [0, 1,  0,     -5],
        [1, 0,  0,      4],
        [0, 0, -1, 1.6858],
        [0, 0,  0,      1]
    ])
    theta = asarray([1.5, 2.5, 3])[None].T

    result = newton_raphson(
        screws, M, T, theta,
        v_tolerance = 0.001,
        w_tolerance = 0.01,
        body = False
    )
    expected = asarray([1.57073819, 2.999667, 3.14153913])[None].T

    if not (abs(result - expected) < 1e-5).all():
        raise AssertionError(f"\n{result}\n!=\n{expected}")

def test_newton_raphson_body():

    screws = asarray([
        [0, 0, -1, 2, 0,   0],
        [0, 0,  0, 0, 1,   0],
        [0, 0,  1, 0, 0, 0.1]
    ])
    M = asarray([
        [-1, 0,  0, 0],
        [ 0, 1,  0, 6],
        [ 0, 0, -1, 2],
        [ 0, 0,  0, 1]
    ])
    T = asarray([
        [0, 1,  0,     -5],
        [1, 0,  0,      4],
        [0, 0, -1, 1.6858],
        [0, 0,  0,      1]
    ])
    theta = asarray([1.5, 2.5, 3])[None].T

    result = newton_raphson(
        screws, M, T, theta,
        v_tolerance = 0.001,
        w_tolerance = 0.01,
        body = True
    )
    expected = asarray([1.57073819, 2.999667, 3.14153913])[None].T

    if not (abs(result - expected) < 1e-5).all():
        raise AssertionError(f"\n{result}\n!=\n{expected}")

def test_analytical_inverse_kinematics():
    from robot import carousel
    from math import radians, degrees

    # end_effector_pos = [150,60,70] #Change this for anything within the limit
    # link_lengths = [75,115,95,85]
    # inverse_kinematics(end_effector_pos, link_lengths)#This prints the angles, just for looking

    theta0 = [
        radians(0),
        radians(45),
        radians(45),
        radians(45),
    ]
    print('theta0: ' + ', '.join([f'{round(degrees(t), 2)}' for t in theta0]))

    p0 = poe(carousel.M, carousel.screws, theta0)[1]
    print('p0: ' + ', '.join([f'{round(x, 2):2}' for x in p0]))

    theta1 = inverse_kinematics(p0, carousel.L)
    print('theta1: ' + ', '.join([f'{round(degrees(t), 2)}' for t in theta1]))

    p1 = poe(carousel.M, carousel.screws, theta1)[1]
    print('p1: ' + ', '.join([f'{round(x, 2)}' for x in p1]))

def test_explodes():
    from modern_robotics import IKinSpace, FKinSpace
    from math import radians

    M = asarray([
        [1, 0, 0, 3],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 0]
    ])
    screws = asarray([
        asarray([0, 0, 1, 0, 0, 0]),
        asarray([0, 0, 1, 0, 1, 0]),
        asarray([0, 0, 1, 0, 2, 0])
    ]).T

    initial_theta = asarray([0, 0, 0])
    final_theta = asarray([0, radians(10), 0])

    T = FKinSpace(M, screws, final_theta)
    print(T)
    print(IKinSpace(screws, M, T, initial_theta, 0.01, 0.001))

if __name__ == '__main__':
    test_space_jacobian()
    test_body_jacobian()
    test_poe_space()
    test_poe_body()
    test_newton_raphson_space()
    test_newton_raphson_body()
    test_analytical_inverse_kinematics()
