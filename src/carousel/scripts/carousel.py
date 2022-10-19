#! /usr/bin/python3

from threading import Lock

import rospy as ros

from numpy import ndarray, asarray, arctan2, pi, hstack, subtract
from numpy.linalg import norm
from scipy.spatial.transform import Rotation

from utility.topics import Topics, Pose, Float32
from utility.statemachine import StateMachine, State

class Carousel(StateMachine):
    """The carousel logic controller node.

    Takes the locations of the cubes and the current end effector location, and
    outputs the desired end effector location and the gripper percentage open.
    """

    def __init__(self, first, /):
        """Create a new carousel logic controller instance."""
        super().__init__(first)
    
        # Input. The positions of the cubees and the end effector location.
        self._cube_sub = Topics.correct_frames.subscriber(self._cube_callback)
        self._effector_sub = Topics.effector.subscriber(self._end_effector_callback)
        self._colour_sub = Topics.block_colour.subscriber(self._colour_callback)

        # Output. The desired end effector location and gripper open percentage.
        self._effector_desired_pub = Topics.effector_desired.publisher()
        self._gripper_pub = Topics.gripper.publisher()

        #Stored Variables.
        self._cube_lock = Lock()
        self._effector_lock = Lock()
        self._colour_lock = Lock()

        self._cube : dict = {}
        self._best_cube = None
        self._effector_position = None
        self._colour = 'red'

        # Orientation error threshold for choosing a cube.
        self._orientation_threshold = 20 * pi / 180
        self._movement_threshold = 5
        self._rotation_threshold = 2
        self._colour_check_position = asarray([0, 215, 315])

        self._home_position = asarray([0, 0, 200])
        self._red_dropoff = asarray([100, 50, 0])
        self._blue_dropoff = asarray([100, 50, 0])
        self._green_dropoff = asarray([100, 50, 0])
        self._yellow_dropoff = asarray([100, 50, 0])

    def _end_effector_callback(self, data):
        """Callback on receiving a new end effector location."""
        self._effector_position = data.pose.pose
        self._effector_lock.acquire()

        p = data.transform.translation
        self._effector_position = asarray([(p.x, p.y, p.z)])

        self._effector_lock.release()

    def _cube_callback(self, data):
        """Callback to record the current cube poses."""

        self._cube_lock.acquire()

        for cube in data.transforms:
            id = cube.fiducial_id
            p = cube.transform.translation
            R = cube.transform.rotation

            yaw, pitch, roll = Rotation.from_quat(
                [R.x, R.y, R.z, R.w]
            ).as_euler('ZYX')
    
            self._cube[id] = (asarray([(p.x, p.y, p.z)]), arctan2(p.y, p.x), yaw)

        self._cube_lock.release()

    def _colour_callback(self, data):
        self._colour_lock.acquire()
        self._colour = data
        self._colour_lock.release()

class Search(State):
    """Search
    
    Try Find the Best cube, if not cycle until found.
    """

    def wait_until_stopped(self): 
        """Waits until the carousel has stopped."""     
        ros.loginfo('Waiting for carousel to stop.')

        id = self._cube.keys()[0]

        while True:

            start = self.machine._cube[id]
            ros.sleep(0.2)
            end = self.machine._cube[id]

            distance = norm(subtract(start, end))

            if distance <= self._rotation_threshold:
                ros.loginfo('Guessed conveyor stopped.')
                break

    def desired_cube(self):

        ros.loginfo('Choosing cube.')

        cubes = [key for key in self.machine._cubes]

        # Filter by orientation.
        for id in cubes:
            yaw_to = self._cube[id][1]
            yaw_of = self._cube[id][2]
            yaw_of = [yaw_of, yaw_of + pi/2, yaw_of + pi, yaw_of + 3*pi/2]

            error = min([abs(yaw_to - yaw) for yaw in yaw_of])
            if error > self.machine._orientation_threshold:
                cubes.pop(cubes.index(id))
                ros.loginfo(f'Cube {id} eliminated for orientation error {error}.')

        #find the cube that is the furthest away from the average, gives best chance of it being the best to pick up.
        #if there are two cubees, then we can just pick either of them.

        ps = hstack([cube[0].reshape([3, 1]) for cube in self.machine._cubes.values()])

        average = average(ps, axis = 0).reshape([3, 1])
        distances = norm(subtract(ps, average.reshape(1, 3).T), axis = 0)

        # Sort the cubes in descending order.
        cubes.sort(key = lambda id: distances[cubes.index(id)], reverse = True)

        return cubes[0]

    def main(self):

        ros.loginfo('Entered search state.')

        while not ros.is_shutdown():

            self.machine._cube_lock.acquire()
            if not self.machine._cube:
                self.machine._cube_lock.release()
                continue

            # Call until stopped here 
            self.machine.wait_until_stopped()

            self.machine._best_cube = self.desired_cube()
            ros.loginfo(f'Selected cube {self.machine._best_cube}.')

            position = self._cubes[self.machine._best_cube][0] + asarray([0, 0, 20])

            self.machine._cube_lock.release()
            return State.Move(position, State.PickUp())

class Move(State):
    """Move
    
    move until desired end effector equals current end effector
    """

    def __init__(
            self,
            position : ndarray,
            after_state : State
        ):
        self._position = position
        self._after_state = after_state

    def main(self):
        """main loop"""
        ros.loginfo('Entered move state.')

        end = Pose()
        end.position.x, end.position.y, end.position.z = self._position
        self.machine._effector_desired_pub.publish(end)

        ros.loginfo(f'Moving to {tuple(self._position)}.')

        while not ros.is_shutdown():
            ros.sleep(0.2)

            self._effector_lock.acquire()
            distance = norm(
                self._position - self.machine._effector_position
            )
            self._cube_lock.release()
    
            ros.loginfo(f'Distance away is {distance}.')

            if distance < self.machine._movement_threshold:
                ros.loginfo(f'Moved to {tuple(self._position)}!')
                return self._after_state

class PickUp(State):

    def __init__(self, stage = 0):
        self._stage = stage

    def main(self):
        """main loop"""
        ros.loginfo(f'Entered pickup state at stage {self._stage}.')

        percent = Float32()

        if self._stage == 0:

            # Open grippper and wait to open.
            ros.loginfo(f'Opening gripper.')
            percent.data = 1.0
            self.machine._gripper_pub.publish(percent)
            ros.sleep(0.5)

            self._effector_lock.acquire()
            position = self.machine._effector_position + asarray([0, 0, -40])
            self._effector_lock.release()

            return State.Move(position, PickUp(stage = 1))
        
        if self._stage == 1:

            # Close the gripper and wait to close.
            ros.loginfo(f'Closing gripper.')
            percent.data = 0.0
            self.machine._gripper_pub.publish(percent)
            ros.sleep(0.5)

            self._effector_lock.acquire()
            position = self.machine._effector_position + asarray([0, 0, 40])
            self._effector_lock.release()

            # Move above the cube, move to the colour check position, and then
            # check the colour.
            return State.Move(
                position,
                State.Move(
                    self._colour_check_position,
                    State.ColourCheck()
                )
            )

class ColourCheck:

    def main(self):
        ros.loginfo('Entered colour checking state.')

        self.machine._colour_lock.acquire()
        colour = self.machine._colour
        self.machine._colour_lock.release()

        if colour == 'red':
            position = self.machine._red_dropoff
        elif colour == 'blue':
            position = self.machine._blue_dropoff
        elif colour == 'green':
            position = self.machine._green_dropoff
        elif colour == 'yellow':
            position = self.machine._yellow_dropoff

        ros.loginfo(f'Cube colour is {colour}.')

        return Move(position, State.DropOff())

class DropOff(State):

    def main(self):
        """main loop"""
        ros.loginfo('Entered drop off state.')

        ros.loginfo(f'Opening gripper.')
        percent = Float32()
        percent.data = 0.0
        self.machine._gripper_pub.publish(percent)
        ros.sleep(0.5)

        self.machine._colour_lock.acquire()
        colour = None
        self.machine._colour_lock.release()
    
        self.machine._cubes.pop(self.machine._best_cube)

        return State.Move(self._home_position, State.Search())

if __name__ == '__main__':
    Carousel(State.Search()).spin()
