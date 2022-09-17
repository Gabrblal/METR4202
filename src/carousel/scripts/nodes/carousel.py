import sys
import rospy as ros

from abc import ABC, abstractmethod

from std_msgs.msg import Float32
from geometry_msgs.msg import Pose

class Carousel(ABC):
    """A carousel logic controller node.

    Contains common operations between all carousel logic controllers.

    Input:
        The positions and orientations of the boxes.
    Output:
        The position and orientation of the end effector.
    """

    def __init__(
            self,
            box_topic : str = 'carousel_box',
            carousel_effector : str = 'carousel_effector',
            carousel_effector_desired : str = 'carousel_effector_desired',
            gripper_topic : str = 'carousel_gripper',
            /
        ):
        """

        Args:
            box_topic: The topic to receive box poses from.
            carousel_effector: The topic to receive end effector pose from.
            carousel_effector_desired: The topic to publish the desired end
                effector pose.
            gripper_topic: The topic to send gripper commands to.
        """
        # Create subscribers and publishers for node input and output.
        self._box_sub = ros.Subscriber(box_topic, Pose, self._box_callback)
        self._effector = ros.Subscriber(carousel_effector, Pose, self._end_effector_callback)
        self._effector_desired = ros.Publisher(carousel_effector_desired, Pose, queue_size = 10)
        self._gripper_pub = ros.Publisher(gripper_topic, Float32, queue_size = 10)

    def _end_effector_callback(self):
        pass

    def _box_callback(self):
        pass

    def _initialise(self):
        """Calibrates the carousel."""
        pass

    def _select_box(self):
        pass

    @abstractmethod
    def _main(self):
        """Main method to begin logic control."""
        pass

    def main(self):
        """Main method """
        
        # Initialise and calibrate the robot.
        self._initialise()

        # Perform the main task.
        self._main()

        # The task is finished, wait for the node to be restarted.
        ros.spin()

class CarouselTask1(Carousel):
    """
    
    Task description:
        One cube (random colour) will be placed on the conveyor. The conveyor
        will spin at a constant speed of up to 6 RPM for approximately 5 to 8
        seconds. It will then stop for exactly 10 seconds before moving again at
        the same speed for 3 seconds, then stop for 10 seconds, and so on. Once
        your robot is able to pick-up and sort the luggage, more cubes will be
        added (one at the time).
    """
    pass

class CarouselTask2(Carousel):
    """

    Task description:
        Up to three cubes (random colours) will be placed on the conveyor. The
        cubes are placed in such a way that they do not obstruct each other
        (i.e. they can be picked up without planning for obstructions). Similar
        to Task Difficulty Level 1, the conveyor will spin at a constant speed
        of up to 6 RPM for approximately 5 to 8 seconds. It will then stop for
        exactly 10 seconds before moving again at the same speed for 3 seconds,
        then stop for 10 seconds, and so on. As soon as the robot picks-up and
        sorts a cube, a replacement cube (random colour) will be added. Ideally,
        your robot must be able to plan the locations of the cubes ahead, to
        optimize time (i.e. you might be able to pick up more than 1 block every
        time the conveyor stops).
    """
    pass

class CarouselTask3a(Carousel):
    """
    
    Task description:
        Up to four cubes (random colours) will be placed on the conveyor. The
        cubes may or may not obstruct each other. e.g. a cube might not allow
        another cue to be picked up on a particular orientation. Your program
        should take obstructions into consideration and needs to implement logic
        that allows for collisions or pick up the blocks in a particular order.
        The conveyor will not be rotating during this task. Different cube
        configurations will be tested.
    """
    pass

class CarouselTask3b(Carousel):
    """
    Task description:
        Up to three cubes (random colours) will be placed on the conveyor. The
        cubes are placed in such a way that they do not obstruct each other
        (i.e. they can be picked up without planning for obstructions). The
        conveyor will spin at a constant speed between 4 to 6 RPM for between 8
        to 10 seconds. It will then stop for approximately 1 second before
        moving again at a different speed for between 8 to 10 seconds. It will
        then stop for 1 second, and start moving again at a different speed (and
        so on). The goal of this task is to pick-up the cube while the conveyor
        is in motion.
    """
    pass

class CarouselTask4(Carousel):
    """
    
    Task description:
        A pre-requisite for attempting this task is to be able to achieve Task
        Difficulty Level 1. The task will be the same, with one exception - the
        Drop-off zones will be out of reach! Can you toss the cubes and make
        them land on the correct, specified zones?
    """
    pass

if __name__ == '__main__':

    n = len(sys.argv)

    # Mapping of task arguments to carousel logic controllers.
    tasks = {
        'task1' : CarouselTask1,
        'task2' : CarouselTask2,
        'task3a' : CarouselTask3a,
        'task3b' : CarouselTask3b,
        'task4' : CarouselTask4
    }

    if n < 3 or sys.argv[2] not in tasks.keys():
        print(f"Unrecognised task not in {', '.join(tasks)}.")

    tasks[sys.argv[2]]().main()
