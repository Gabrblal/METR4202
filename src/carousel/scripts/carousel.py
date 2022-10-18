from calendar import c
import rospy as ros
import math
from utility.topics import Topics

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

        #Stored Variables.
        self._box_positions #list of poses, [pose, pose, pose]
        self._effector_position #pose

    def _end_effector_callback(self, data):
        self._effector_position = data.pose.pose
        """Callback on receiving a new end effector location."""

    def _box_callback(self, data):
        box_positions = []
        for position in data:
            box_positions.append(data.pose.pose)
        self._box_positions = box_positions

        """Callback on receiving a new cube location and orientation."""

    def _initialise(self):
        """Calibrates the carousel."""
        pass

    def _main(self):
        """Main method to begin sorting cubes."""
        # Repeat:

        while True:
            ros.Timer(ros.Duration(2), self._box_sub)
            ros.Timer(ros.Duration(2), self._effector_sub)

            cube_to_pickup = self.desired_cube()
            # Find a cube and lock onto it.


            # Position the end effector over the cube.
            # Determine the position of the end effector to pick the cube up.
            #use publishers for desired positions, and subscribers for current positions.
            # Pick up the cube.
            self._effector_desired_pub.publish(cube_to_pickup) #this should be in the right reference frame?

            while self._effector_position != cube_to_pickup:
                ros.Timer(ros.Duration(1), self._effector_sub)

            #TODO self._gripper_pub.publish( close )

            # Could determine the future position of the cube using the
            # instantaneous centre of location of the cubes rotation, and then
            # using the angular speed of the cube to figure out where it would be
            # in the future. Does not consider the box stopping in the meantime.

            # Place the cube.
            
        pass

    def desired_cube(self):
        #find the average x, y position of the cubes
        cube_position_average_x = 0
        cube_position_average_y = 0

        for cube in self._box_positions:
            cube_position_average_x += cube.x
            cube_position_average_y += cube.y

        cube_position_average_x/len(self._box_positions)
        cube_position_average_y/len(self._box_positions)


        #find the cube that is the furthest away from the average, gives best chance of it being the best to pick up.
        best_distance = 0
        best_cube = self._box_positions[0]
        if (len(self._box_positions) in (1, 3,4, 5)):
            for cube in self._box_positions:
                x_dist = (cube_position_average_x + cube.x) ** 2
                y_dist = (cube_position_average_y + cube.y) ** 2
                dist_from_centre =  math.sqrt(x_dist + y_dist)
                
                if dist_from_centre > best_distance:
                    best_distance = dist_from_centre
                    best_cube = cube
        else:
            best_cube = cube = self._box_positions[0]

        return best_cube


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
