from abc import ABC, abstractmethod
from typing import Sequence, Tuple

from scipy.interpolate import splprep, BSpline
from numpy import ndarray, asarray, piecewise, logical_and, eye

from .kinematics import mexp, mlog

import rospy as ros

class Parameterisation(ABC):

    @abstractmethod
    def __call__(self, t : float) -> float:
        """Evaluate the parameterisation of the trajectory.

        The parameterisation is a value on [0, 1].

        Args:
            t: The times at which to evaluate the trajectory parameter.

        Returns:
            The parameterisation of the trajectory at the provided time.
        """
        pass

    @abstractmethod
    def duration(self):
        pass

class Rotation(ABC):

    @abstractmethod
    def __call__(self, x : float) -> ndarray:
        """Evaluate the rotation of the trajectory.

        Args:
            x: The value at which to evaluate the rotation.

        Returns:
            The 3x3 rotation matrix.
        """
        pass

class Translation(ABC):

    @abstractmethod
    def __call__(self, x : float) -> ndarray:
        """Evaluates the translation of the trajectory.

        Args:
            x: The value at which to evaluate the translation.

        Returns:
            The 3x1 translation vector.
        """
        pass

class Trajectory:

    def __init__(self, rotation : Rotation, translation : Translation):
        self._rotation = rotation
        self._translation = translation

    def __call__(self, t : float) -> ndarray:
        """Evaluates the transformation matrix T of the trajectory.

        Args:
            s: The parameter at which to evaluate the transformation matrix.

        Returns:
            The 4x4 transformation matrix.
        """
        T, T[:3, :3], T[:3, 3] = eye(4), self._rotation(t), self._translation(t)
        return T

class Linear(Parameterisation):

    def __init__(self, start, duration):
        self._start = start
        self._duration = duration
        self._end = start + duration

    def __call__(self, t):
        if t > self._end:
            t = self._end
        elif t < self._start:
            t = self._start

        return (t - self._start) / self._duration

    def duration(self):
        return self._duration

class SCurve(Parameterisation):

    @staticmethod
    def from_velocity_acceleration(start, velocity, acceleration):
        """Create a new S Curve parameterisation.

        Acceleration is limited to a maximum of velocity squared.

        Args:
            start: The time at which the curve should start.
            velocity: The maximum coasting velocity.
            acceleration: The acceleration.
        """
        return SCurve(
            start,
            v := velocity, 
            a := max(v ** 2, acceleration),
            T := (a + v**2) / (a * v)
        )

    def __init__(self, start, v, a, T):
        """Create a new S Curve parameterisation.

        Args:
            start: The time at which the curve should start.
            v: The maximum coasting velocity.
            a: The acceleration.
            T: The period.
        """
        self._start = start
        self._velocity = v
        self._acceleration = a
        self._duration = T

        # Piecewise function of t.
        self._s = lambda t: piecewise(
            t,
            condlist = (
                t < 0,
                logical_and(0 <= t, t < v / a),
                logical_and(v / a <= t, t < T - v / a),
                logical_and(T - v / a <= t, t <= T),
                t > T
            ),
            funclist = (
                lambda t: 0 * t,
                lambda t: 0.5 * a * t ** 2,
                lambda t: v * t - v ** 2 / (2 * a),
                lambda t: (2*a*v*T - 2*v**2 - a**2*(t - T)**2) / (2*a),
                lambda t: t / t
            )
        )

    def __call__(self, t) -> float:
        """Get the parameterisation s of the S curve at time t.

        Args:
            t: The time at which to evaluate the S curve.

        Return:
            The parameterisation s.
        """
        return self._s(t - self._start)

    def duration(self):
        return self._duration

    def plot(self):
        """Plot the S curve parameterisation."""
        import matplotlib.pyplot as plt
        from numpy import linspace

        ts = linspace(self._start, self._start + self._duration, 100)
        s = self(ts)

        plt.plot(ts, s)
        plt.xlabel('t')
        plt.ylabel('s')
        plt.title('S Curve Parameterisation')
        plt.show()

class InterpolatedRotation(Rotation):
    """
    References:
        Formula: Modern Robotics pp. 328 / pdf 346
        Rotation Exponential: Modern Robotics pp. 84 / pdf 102
        Rotation Logarithm: Modern Robotics pp. 85 / pdf 103
    """

    def __init__(self, R0, R1, parameterisation):
        """Instantiate a new interpolated rotation.

        Args:
            R0: The initial rotation.
            R1: The final rotation.
            parameterisation: The parameterisation of time to use to determine
                the interpolated rotation matrix.
        """
        self._R0 = asarray(R0)
        self._R1 = asarray(R1)
        self._parameterisation = parameterisation

        if self._R0.shape != (3, 3) or self._R1.shape != (3, 3):
            raise RuntimeError("Rotation matricies not 3x3.")

    def __call__(self, t):
        """Get the interpolated rotation matrix at time t.

        Args:
            t: The time at which to evaluate the interpolated rotation.

        Return:
            The 3x3 rotation matrix.
        """
        w, t = mlog(self._R0.T @ self._R1, decomposed = True)
        s = self._parameterisation(t)

        return self._R0 @ mexp(w, t * s)

class Spline(Translation):

    def __init__(self, control, parameterisation):
        """Instantiate a new spline.

        Args:
            control: The control points at which the splines should pass
                through.
            parameterisation:
        """
        self._parameterisation = parameterisation

        try:
            (tck, c, k), _ = splprep(list(zip(*control)), k = 3, s = 0)
            self._spline = BSpline(tck, asarray(c).T, k)
        except TypeError:
            raise RuntimeError("Spline must have at least 4 control points.")

    def __call__(self, t):
        """Get the translation of the spline at time t.

        Args:
            t: The time at which to evaluate the translation from the spline.

        Returns:
            The translation at time t.
        """
        return self._spline(self._parameterisation(t))

    def plot(self):
        import matplotlib.pyplot as plt
        from numpy import linspace

        ts = linspace(0, self._parameterisation.duration(), 100)
        p = [self(t) for t in ts]

        x, y, z = list(zip(*p))

        figure, axes = plt.subplot_mosaic([
                ('x', '3d'),
                ('y', '3d'),
                ('z', '3d')
            ]
        )

        for label, data in [('x', x), ('y', y), ('z', z)]:
            axes[label].plot(ts, data)
            axes[label].set_xlabel('t')
            axes[label].set_ylabel(label)
            axes[label].set_title(f'{label.capitalize()} versus Time', fontsize = 'medium')

        ss = axes['3d'].get_subplotspec()
        axes['3d'].remove()
        axes['3d'] = figure.add_subplot(ss, projection = '3d')
        axes['3d'].set_xlabel('x')
        axes['3d'].set_ylabel('y')
        axes['3d'].set_zlabel('z')
        axes['3d'].plot(x, y, z)

        figure.set_figwidth(8)
        figure.set_figheight(6)
        plt.subplots_adjust(
            left = 0.1,
            bottom = 0.10,
            right = 0.90,
            top = 0.95,
            wspace = 0.2,
            hspace = 0.6
        )
        plt.show()

def plot_spline_trajectory():
    # s = SCurve(0, 1, 5)
    # s.plot()

    Spline(
        [(-100, 0, 0), (-50, 0, 0), (50, 0, 0), (100, 0, 0)],
        SCurve.from_velocity_acceleration(0, 20 / 100, 0)
    ).plot()

def test_linear_parameterisation():
    assert abs(Linear(0, 5)(5) - 1) < 1e-6
    assert abs(Linear(-10, 10)(0) - (-10)) < 1e-6
    assert abs(Linear(0, 1)(0)) < 1e-6

def test_interpolated_rotation():
    from numpy import radians, degrees, arcsin, linspace
    from tf.transformations import euler_matrix

    yaw = lambda R: degrees(arcsin(R[0, 1]))

    R0 = euler_matrix(0, 0, radians(-45))[:3, :3]
    R1 = euler_matrix(0, 0, radians(45))[:3, :3]

    print(R0, R1, sep = '\n\n')

    rotation = InterpolatedRotation(R0, R1, Linear(0, 1))

    print([yaw(rotation(t)) for t in linspace(0, 1, 5)])

def test_trajectory():
    from numpy import eye

    trajectory = Trajectory(
        [(0, 0, 0), (0, 1, 1), (2, -1, 0), (-1, -2, -1), (0, -2, 2), (1, 2, 1)],
        [eye(3), eye(3)],
        0, 10, 2
    )

    trajectory.plot()

if __name__ == '__main__':
    plot_spline_trajectory()
    # print(Linear(1665115075.1237726, 1665115175.1237726)(1665115075.1237726))
    # test_interpolated_rotation()
