from abc import ABC, abstractmethod
from typing import Sequence, Tuple

from scipy.interpolate import splprep, BSpline
from numpy import ndarray, asarray, piecewise, logical_and, eye

from .kinematics import mexp, mlog

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

    def transformation(self, t : float) -> ndarray:
        """Evaluates the transformation matrix T of the trajectory.

        Args:
            s: The parameter at which to evaluate the transformation matrix.

        Returns:
            The 4x4 transformation matrix.
        """
        T, T[:3, :3], T[:3, 3] = eye(4), self._rotation(t), self._translation(t)
        return T

class SCurve(Parameterisation):

    def __init__(
            self,
            start,
            velocity,
            acceleration
        ):
        """Create a new S Curve parameterisation.

        Args:
            start: The time at which the curve should start.
            velocity: The maximum coasting velocity.
            acceleration: The acceleration.
        """

        # S curve parameters.
        v = velocity
        a = max(v ** 2, acceleration)
        T = (acceleration + velocity**2) / (acceleration * velocity)

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

        self._start = start
        self._velocity = v
        self._acceleration = a
        self._duration = T

    def __call__(self, t) -> float:
        """Get the parameterisation s of the S curve at time t.

        Args:
            t: The time at which to evaluate the S curve.

        Return:
            The parameterisation s.
        """
        return self._s(asarray(t - self._start))

class InterpolatedRotation(Rotation):

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

class Trajectory:
    """A 3D trajectory composed of spline interpolation and linearly
    interpolated rotation."""

    def __init__(
            self,
            control : Sequence[Tuple[float, float, float]],
            rotation : Tuple[ndarray, ndarray],
            start : float,
            acceleration : float,
            velocity : float,
            /
        ):
        """Create a new spline from the provided control points.

        Required that velocity ** 2 / acceleration <= 1 in order to reach
        the coasting velocity.

        Args:
            control: A sequence of control points that define the spline. Must
                be at least length 4.
            start: The time at which to start the trajectory.
            acceleration: The acceleration to get to the coasting velocity.
            velocity: The coasting velocity of the trajectory.
        """
        # Rotation matricies.
        if len(rotation) != 2:
            raise RuntimeError("Two rotation matricies required.")

        self._R0 = asarray(rotation[0])
        self._R1 = asarray(rotation[1])

        if self._R0.shape != (3, 3) or self._R1.shape != (3, 3):
            raise RuntimeError("Rotation matricies not 3x3.")

        # Spline.
        try:
            (tck, c, k), _ = splprep(list(zip(*control)), k = 3, s = 0)
            self._spline = BSpline(tck, asarray(c).T, k)
        except TypeError:
            raise RuntimeError("Must have at least 4 control points.")

        # S curve parameters.
        v = velocity
        a = max(v ** 2, acceleration)
        T = (acceleration + velocity**2) / (acceleration * velocity)

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

        self._start = start
        self._velocity = v
        self._acceleration = a
        self._duration = T

    @property
    def velocity(self):
        return self._velocity

    @property
    def acceleration(self):
        return self._acceleration

    @property
    def duration(self):
        return self._duration

    def s(self, t):
        return self._s(asarray(t - self._start))

    def R(self, t):
        return [self._R0 @ mexp(self._rotation_factor, x) for x in self._s(t)]

    def p(self, t):
        return self._spline(self._s(t))

    def at(self, t, *, decomposed = False):
        # Calculate the s curve interpolation constant.
        s = self.s(t)

        w, t = mlog(self._R0.T @ self._R1, decomposed = True)

        R = self._R0 @ mexp(w, t * s)

        p = self._spline(s)

        if decomposed:
            return R, p

        T, T[:3, :3], T[:3, 3] = eye(4), R, p
        return T

def plot(translation, rotation):
    """Plots the trajectory over time against time and 3D space, and the
    parameterisation over time.
    """
    import matplotlib.pyplot as plt
    from numpy import linspace

    t = linspace(0, self.duration, 100)
    R, p = self(t)
    x, y, z = list(zip(*p))

    figure, axes = plt.subplot_mosaic([
            ('x', '3d'),
            ('y', '3d'),
            ('z', 's')
        ]
    )

    for label, data in [('x', x), ('y', y), ('z', z), ('s', s)]:
        axes[label].plot(t, data)
        axes[label].set_xlabel('t')
        axes[label].set_ylabel(label)
        axes[label].set_title(f'{label.capitalize()} versus Time', fontsize = 'medium')

    ss = axes['3d'].get_subplotspec()
    axes['3d'].remove()
    axes['3d'] = figure.add_subplot(ss, projection = '3d')

    axes['3d'].plot(x, y, z)

    axes['3d'].set_xlabel('x')
    axes['3d'].set_ylabel('y')
    axes['3d'].set_zlabel('z')

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

if __name__ == '__main__':
    from numpy import eye

    trajectory = Trajectory(
        [(0, 0, 0), (0, 1, 1), (2, -1, 0), (-1, -2, -1), (0, -2, 2), (1, 2, 1)],
        [eye(3), eye(3)],
        0, 10, 2
    )

    trajectory.plot()
