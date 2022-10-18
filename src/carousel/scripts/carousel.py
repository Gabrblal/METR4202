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


    """
    determine the direction of the carousel movement.
    @return "clockwise" for clockwise and "anticlockwise" for anticlockwise
    """
    def carousel_direction(self):
        clockwise = "clockwise"
        anticlockwise = "anticlockwise"

        box1 = self._box_positions[0]
        ros.Timer(ros.Duration(0.1), self._box_sub)
        box2 = self._box_positions[0]

        distance_to_centre_carousel = 0.19

        #determine if change in x is positive / negative.

        x_change_dist = (box1.x - box2.x) / abs(box1.x - box2.x)
        y_change_dist = (box1.y - box2.y) / abs(box1.y - box2.y)

        #two cases, one for closer than 0.19, one for further.


        #in far region
        if box1.y > distance_to_centre_carousel and box2.y > distance_to_centre_carousel:
            
            #the x has changed positive direction thus clockwise
            if (x_change_dist == 1):
                return clockwise
            else:
                return anticlockwise

        #in close region
        elif box1.y < distance_to_centre_carousel and box2.y < distance_to_centre_carousel:
            
            if (x_change_dist == -1):
                return clockwise
            else:
                return anticlockwise

        #the box has changed regions in between
        else:
            if (box2.y > box1.y and box1.x < 0) or (box2.y < box1.y and box1.x > 0):
                return clockwise
            else:
                return anticlockwise
            #


    """
    determine if the carousel is stopped.
    magnitude of difference between frames in 0.1 seconds
    @return true if distance is less than 0.001 #TODO ADJUST THIS
    """
    def check_if_stopped(self):
        
        #first

        box1 = self._box_positions[0]
        
        ros.Timer(ros.Duration(0.1), self._box_sub)

        #second

        box2 = self._box_positions[0]

        distance = math.sqrt((box2.x-box1.x)**2 + (box2.y-box2.y)**2)

        if distance > 0.001:
            return False
        else: 
            return True 


    """
    Determine the range of a cube, from the base frame
    """
    def cube_range_from_base(self, cube):
        #cube.x, cube.y
        return math.sqrt(cube.x**2 + cube.y**2)

    """
    find the best cube by it's distance from the rest or orientation for the cube.
    """
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
        #if there are two boxes, then we can just pick either of them.

        best_distance = 0
        best_cube = self._box_positions[0]
        if (len(self._box_positions) in (1,3,4,5)):
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


class StateMachine:

    def __init__(self, first):
        """Initialises the sate machine with the first state"""
        self._state = first

    def spin(self):
        """Main loop of the state machine that iterates through states"""

        while True:
            setattr(self._state, "_machine", self)

            self._state.on_entry()
            next_state = self_state.main()
            self._state_on_exit()

            if next_state is None:
                break

            self_state = next_state

class StartState:
    """State 1"""

    def __init_subclass__(cls) -> None:
        """Register the subclassed state

        This method allows subclasses of State to recursively reference each other
        """
        setattr(StartState, cls.__name__, cls)

    def on_entry(self):
        """Options method to call when a state is entered"""
        pass

    def on_exit():
        """optional method to call when a state is exited"""
        pass

    def main(self):
        """main loop"""
        pass

class Search:
    """Search
    
    Try Find the Best box, if not cycle until found.
    """

    def __init_subclass__(cls) -> None:
        """Register the subclassed state

        This method allows subclasses of State to recursively reference each other
        """
        setattr(Search, cls.__name__, cls)

    def on_entry(self):
        """Options method to call when a state is entered"""
        pass

    def on_exit():
        """optional method to call when a state is exited"""
        pass

    def main(self):
        """main loop"""
        pass


class Move:
    """Move
    
    move until desired end effector equals current end effector
    """

    def __init_subclass__(cls) -> None:
        """Register the subclassed state

        This method allows subclasses of State to recursively reference each other
        """
        setattr(Move, cls.__name__, cls)

    def on_entry(self):
        """Options method to call when a state is entered"""
        pass

    def on_exit():
        """optional method to call when a state is exited"""
        pass

    def main(self):
        """main loop"""
        pass


class PickUp:
    """PickUp
    
    close gripper
    """

    def __init_subclass__(cls) -> None:
        """Register the subclassed state

        This method allows subclasses of State to recursively reference each other
        """
        setattr(PickUp, cls.__name__, cls)

    def on_entry(self):
        """Options method to call when a state is entered"""
        pass

    def on_exit():
        """optional method to call when a state is exited"""
        pass

    def main(self):
        """main loop"""
        pass


class DropOff:
    """DropOff
    
    open gripper
    """

    def __init_subclass__(cls) -> None:
        """Register the subclassed state

        This method allows subclasses of State to recursively reference each other
        """
        setattr(DropOff, cls.__name__, cls)

    def on_entry(self):
        """Options method to call when a state is entered"""
        pass

    def on_exit():
        """optional method to call when a state is exited"""
        pass

    def main(self):
        """main loop"""
        pass


class ColourCheck:
    """Colour Check
    
    wait until colour found
    """

    def __init_subclass__(cls) -> None:
        """Register the subclassed state

        This method allows subclasses of State to recursively reference each other
        """
        setattr(ColourCheck, cls.__name__, cls)

    def on_entry(self):
        """Options method to call when a state is entered"""
        pass

    def on_exit():
        """optional method to call when a state is exited"""
        pass

    def main(self):
        """main loop"""
        pass

    
    @property
    def machine(self):
        """Get the state machine to which this state belongs to"""



if __name__ == '__main__':
    Carousel().main()
