from typing import Sequence, Tuple

from scipy.interpolate import splprep, BSpline
from numpy import ndarray, asarray, piecewise, logical_and

from .trajectory import texp, tlog

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

        self._rotation_factor = tlog(self._R0.T @ self._R1)

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
        """Evaluates the parameterisation of the trajectory over time.

        Args:
            t: The times at which to evaluate the trajectory parameter.

        Returns:
            An array or single value of the trajectory parameters.
        """
        return self._s(asarray(t - self._start))

    def R(self, t):
        """Evaluates the rotation of the trajectory as a function of time.

        Args:
            t: The times at which to evaluate the rotation.

        Returns:
            The rotation matricies at the provided times.
        """
        return [self._R0 @ texp(self._rotation_factor * x) for x in self._s(t)]

    def p(self, t):
        """Evaluates the translation of the trajectory as a function of time.

        Args:
            t: The times at which to evaluate the translation.

        Returns:
            An array of or singular position at s.
        """
        return self._spline(self._s(t))

    def __call__(self, t):
        """Evaluates the trajectory at the provided time.

        Args:
            t: The time in seconds to evaluate the configuration of the
                trajectory.

        Returns:
            A tuple of (rotations, translations) at the provided times.
        """
        # Calculate the s curve interpolation constant.
        s = self.s(t)

        R = [self._R0 @ texp(self._rotation_factor * x) for x in s]
        p = self._spline(s)

        return R, p

    def plot(self):
        """Plots the trajectory over time against time and 3D space, and the
        parameterisation over time.
        """

        import matplotlib.pyplot as plt
        from numpy import linspace

        t = linspace(0, self.duration, 100)
        s = self.s(t)
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
