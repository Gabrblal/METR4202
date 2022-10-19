#! /usr/bin/python3

from threading import Lock

import rospy as ros

from numpy import ndarray, asarray, arctan2, pi, vstack, subtract, average
from numpy.linalg import norm
from scipy.spatial.transform import Rotation

from utility.kinematics import angle_wrap
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
        self._colour = None

        # Orientation error threshold for choosing a cube.
        self._orientation_threshold = 20 * pi / 180
        self._movement_threshold = 5
        self._rotation_threshold = 2
        self._colour_check_position = asarray([0, 215, 315])

        self._home_position = asarray([0, 0, 200])

        self._dropoff = {
            'red': asarray([-200, -50, 0]),
            'green': asarray([-50, -150, 0]),
            'blue': asarray([50, -150, 0]),
            'yellow': asarray([200, -50, 0]),
        }

    def _end_effector_callback(self, data):
        """Callback on receiving a new end effector location."""
        self._effector_lock.acquire()

        p = data.position
        self._effector_position = asarray([(p.x, p.y, p.z)])

        self._effector_lock.release()

    def _cube_callback(self, data):
        """Callback to record the current cube poses."""

        self._cube_lock.acquire()

        for cube in data.transforms:
            id = cube.fiducial_id
            p = cube.transform.translation
            R = cube.transform.rotation

            yaw, _, _ = Rotation.from_quat(
                [R.x, R.y, R.z, R.w]
            ).as_euler('ZYX')

            self._cube[id] = (asarray([p.x, p.y, p.z]), arctan2(p.y, p.x), yaw)

        self._cube_lock.release()

    def _colour_callback(self, data):
        self._colour_lock.acquire()
        self._colour = data
        self._colour_lock.release()

class Search(State):
    """Search
    
    Try Find the Best cube, if not cycle until found.
    """

    def _has_stopped(self): 
        """Waits until the carousel has stopped."""     
        ros.loginfo('Waiting for carousel to stop.')

        id = tuple(self.machine._cube.keys())[0]

        start = self.machine._cube[id][0]
        ros.sleep(0.2)
        end = self.machine._cube[id][0]

        # ros.loginfo(f'{start}')
        # ros.loginfo(f'{end}')
        # ros.loginfo(f'{norm(subtract(start, end))}')
        distance = norm(subtract(start, end))

        return distance <= self.machine._rotation_threshold

    def _desired_cube(self):

        ros.loginfo('Choosing cube.')

        cubes = [key for key in self.machine._cube.keys()]

        # Filter by orientation.
        for id in cubes:
            yaw_to = self.machine._cube[id][1]
            yaw_of = self.machine._cube[id][2]
            yaw_of = [yaw_of, yaw_of + pi/2, yaw_of + pi, yaw_of + 3*pi/2]
 
            error = min([abs(angle_wrap(yaw_to - yaw)) for yaw in yaw_of])
            if error > self.machine._orientation_threshold:
                cubes.pop(cubes.index(id))
                ros.loginfo(f'Cube {id} eliminated for orientation error {error}.')

        # If no cubes are grabbable then return no desired cubes.
        if not cubes:
            return None

        ps = vstack([cube[0] for cube in self.machine._cube.values()])
        mu = average(ps, axis = 0).reshape([3, 1])
        distances = norm(subtract(ps, mu.reshape([3, 1]).T), axis = 0)

        # Sort the cubes in descending order by distance away from the cluster.
        ros.loginfo(f'Cubes: {cubes}')

        return cubes[distances.argmin()]

    def main(self):

        ros.loginfo('Entered search state.')

        while not ros.is_shutdown():
            ros.sleep(2)

            self.machine._cube_lock.acquire()
            if not self.machine._cube:
                self.machine._cube_lock.release()
                continue

            # Wait until the carousel has stopped.
            if not self._has_stopped():
                self.machine._cube_lock.release()
                continue

            ros.loginfo('Guessed conveyor stopped.')
            self.machine._best_cube = self._desired_cube()

            # If there are no good cubes to pick up then continue searching.
            if self.machine._best_cube is None:
                self.machine._cube_lock.release()
                continue

            ros.loginfo(f'Selected cube {self.machine._cube[self.machine._best_cube]}.')

            position = self.machine._cube[self.machine._best_cube][0] + asarray([0, 0, 20])
            ros.loginfo(f'Cube at {tuple(position)}.')

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
        end.position.x = self._position[0]
        end.position.y = self._position[1]
        end.position.z = self._position[2]
        self.machine._effector_desired_pub.publish(end)

        ros.loginfo(f'Moving to {tuple(self._position)}.')

        while not ros.is_shutdown():
            ros.sleep(0.2)

            self.machine._effector_lock.acquire()
            distance = norm(
                self._position - self.machine._effector_position
            )
            self.machine._effector_lock.release()

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
        return State.Search()

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

        colour = 'red'

        position = self.machine._dropoff[colour]
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
    
        self.machine._cube.pop(self.machine._best_cube)

        return State.Move(self._home_position, State.Search())

if __name__ == '__main__':
    ros.init_node('carouselLogicNode')
    Carousel(State.Search()).spin()
