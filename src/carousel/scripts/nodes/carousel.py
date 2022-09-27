import rospy as ros

from ..topics import Topics

class Carousel:
    """The carousel logic controller node.

    Takes the locations of the cubes and the current end effector location, and
    outputs the desired end effector location and the gripper percentage open.
    """

    def __init__(self, /):
        """Create a new carousel logic controller instance."""
        # Input. The positions of the boxes and the end effector location.
        self._box_sub = Topics.box.subscriber(self._box_callback)
        self._effector_sub = Topics.effector.subscriber(self._end_effector_callback)

        # Output. The desired end effector location and gripper open percentage.
        self._effector_desired_pub = Topics.effector_desired.publisher()
        self._gripper_pub = Topics.gripper.publisher()

    def _end_effector_callback(self):
        """Callback on receiving a new end effector location."""
        pass

    def _box_callback(self):
        """Callback on receiving a new cube location and orientation."""
        pass

    def _initialise(self):
        """Calibrates the carousel."""
        pass

    def _main(self):
        """Main method to begin sorting cubes."""

        # Repeat:

        # Find a cube and lock onto it.

        # Position the end effector over the cube.

        # Could determine the future position of the cube using the
        # instantaneous centre of location of the cubes rotation, and then
        # using the angular speed of the cube to figure out where it would be
        # in the future. Does not consider the box stopping in the meantime.

        # Determine the position of the end effector to pick the cube up.

        # Pick up the cube.

        # Place the cube.

        pass

    def main(self):
        """Main method """

        # Initialise and calibrate the robot.
        self._initialise()

        # Perform the main task.
        self._main()

        # The task is finished, wait for the node to be restarted.
        ros.spin()

if __name__ == '__main__':
    Carousel().main()
