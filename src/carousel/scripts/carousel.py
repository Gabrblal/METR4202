#! /usr/bin/python3

from collections import deque
from threading import Lock

import rospy as ros

from numpy import ndarray, asarray, arctan2, pi, vstack, subtract, average
from numpy.linalg import norm
from scipy.spatial.transform import Rotation

from utility.robot import carousel
from utility.kinematics import angle_wrap, inverse_analytical_4R
from utility.topics import Topics, Pose, Float32, JointState, Header
from utility.statemachine import StateMachine, State

class Carousel(StateMachine):
    """The carousel logic controller node.

    Takes the locations of the cubes and the current end effector location, and
    outputs the desired end effector location and the gripper percentage open.
    """

    def __init__(self, first, /):
        """Create a new carousel logic controller instance."""
        super().__init__(first)

        #Stored Variables.
        self._cube_lock = Lock()
        self._effector_lock = Lock()
        self._colour_lock = Lock()

        self._cube : dict = {}
        self._best_cube = None
        self._effector_position = None
        self._colour = None
        self._pickup_pitch = -pi/4

        # Orientation error threshold for choosing a cube.
        self._orientation_threshold = 20 * pi / 180
        self._movement_threshold = 15
        self._rotation_threshold = 2
        self._colour_check_position = asarray([0, 215, 315])

        self._home_position = asarray([0, 10, 390])

        self._dropoff = {
            'red': asarray([-200, -50, 0]),
            'green': asarray([-50, -150, 0]),
            'blue': asarray([50, -150, 0]),
            'yellow': asarray([200, -50, 0]),
        }

        # Input. The positions of the cubees and the end effector location.
        self._cube_sub = Topics.correct_frames.subscriber(self._cube_callback)
        self._effector_sub = Topics.effector.subscriber(self._end_effector_callback)
        self._colour_sub = Topics.block_colour.subscriber(self._colour_callback)

        # Output. The desired end effector location and gripper open percentage.
        self._effector_desired_pub = Topics.effector_desired.publisher()
        self._gripper_pub = Topics.gripper.publisher()
        self._joint_pub = Topics.desired_joint_states.publisher()

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

        ps = vstack([self.machine._cube[id][0] for id in cubes])
        mu = average(ps, axis = 0).reshape([3, 1])
        distances = norm(subtract(ps, mu.reshape([3, 1]).T), axis = 0)

        ros.loginfo(f'ps: {ps}')
        ros.loginfo(f'mu: {mu}')

        # Sort the cubes in descending order by distance away from the cluster.
        ros.loginfo(f'Cubes: {cubes}')
        ros.loginfo(f'{distances}')
        ros.loginfo(f'Maximum: {distances.argmax()}')

        return cubes[distances.argmax()]

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

            if norm(position[0:2]) > carousel.L[1] + carousel.L[2]:
                self.machine._pickup_pitch = -pi/4
            else:
                self.machine._pickup_pitch = -pi/2

            self.machine._cube_lock.release()

            return State.Track()
            # return State.Move(position, self.machine._pickup_pitch, State.PickUp())

class Move(State):

    def __init__(
            self,
            position : ndarray,
            pitch : float,
            after_state : State
        ):
        self._position = asarray(position).flatten()
        self._pitch = pitch
        self._after_state = after_state

    def main(self):
        ros.loginfo('Entered move state.')

        # Get the quarternion transformation matrix from the pitch.
        q = Rotation.from_euler('ZYX', [0, self._pitch, 0]).as_quat()

        end = Pose()
        end.position.x = self._position[0]
        end.position.y = self._position[1]
        end.position.z = self._position[2]
        end.orientation.x = q[0]
        end.orientation.y = q[1]
        end.orientation.z = q[2]
        end.orientation.w = q[3]

        self.machine._effector_desired_pub.publish(end)
        ros.loginfo(f'Moving to {tuple(self._position)}.')

        # Keep track of the most recent distances to the end effector.
        recent = deque(maxlen = 10)

        while not ros.is_shutdown():
            ros.sleep(0.2)

            self.machine._effector_lock.acquire()

            # If no end effector position then wait for one.
            if self.machine._effector_position is None:
                self.machine._effector_lock.release()
                continue

            distance = norm(self._position - self.machine._effector_position)
            self.machine._effector_lock.release()

            moved = distance < self.machine._movement_threshold

            # Fallback on consistently the same distance.
            recent.append(distance)
            moved |= len(recent) == recent.maxlen and all(d == recent[0] for d in recent)

            if moved:
                ros.loginfo(f'Moved to {tuple(self._position)}!')
                return self._after_state
            else:
                ros.loginfo(
                    f'End {distance} < {self.machine._movement_threshold} to desired'
                )
                # ros.loginfo(f'Desired: {tuple(self._position)}.') #Distance away is {distance}.')
                # ros.loginfo(f'Effector: {tuple(self.machine._effector_position)}.')

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

            self.machine._effector_lock.acquire()
            position = self.machine._effector_position + asarray([0, 0, -20])
            ros.loginfo(f'{position}')
            self.machine._effector_lock.release()

            return State.Move(position, self.machine._pickup_pitch, PickUp(stage = 1))

        if self._stage == 1:

            # Close the gripper and wait to close.
            ros.loginfo(f'Closing gripper.')
            percent.data = 0.0
            self.machine._gripper_pub.publish(percent)
            ros.sleep(0.5)

            self.machine._effector_lock.acquire()
            position = self.machine._effector_position + asarray([0, 0, 40])
            self.machine._effector_lock.release()

            # Move above the cube, move to the colour check position, and then
            # check the colour.
            return State.Move(
                position,
                self.machine._pickup_pitch,
                State.Move(
                    self.machine._colour_check_position,
                    0,
                    State.ColourCheck()
                )
            )

class ColourCheck(State):

    def main(self):
        ros.loginfo('Entered colour checking state.')

        self.machine._colour_lock.acquire()
        colour = self.machine._colour
        self.machine._colour_lock.release()

        colour = 'red'

        position = self.machine._dropoff[colour]
        ros.loginfo(f'Cube colour is {colour}.')

        return State.Move(position, -pi/2, State.DropOff())

class DropOff(State):

    def main(self):
        """main loop"""
        ros.loginfo('Entered drop off state.')

        ros.loginfo(f'Opening gripper.')
        percent = Float32()
        percent.data = 1.0
        self.machine._gripper_pub.publish(percent)
        ros.sleep(0.5)

        self.machine._colour_lock.acquire()
        colour = None
        self.machine._colour_lock.release()
    
        self.machine._cube.pop(self.machine._best_cube)

        return State.Move(self.machine._home_position, pi/2, State.Search())

class Track(State):

    def main(self):
        ros.sleep(0.2)

        self.machine._cube_lock.acquire()
        position = self.machine._cube[self.machine._best_cube][0] + asarray([-10, 0, 30])
        # position = asarray([0, 200, 57])
        self.machine._cube_lock.release()

        if norm(position[0:2]) > carousel.L[1] + carousel.L[2]:
            pitch = -pi/4
        else:
            pitch = -pi/2

        theta = inverse_analytical_4R(position, carousel.L, pitch)
    
        self.machine._joint_pub.publish(
            JointState(
                header = Header(stamp = ros.Time.now()),
                name = ['joint_1', 'joint_2', 'joint_3', 'joint_4'],
                position = theta,
                velocity = [1.0, 1.0, 1.0, 1.0]
            ) 
        )

        return self

if __name__ == '__main__':
    ros.init_node('CarouselLogicNode')
    Carousel(State.Search()).spin()
    # Carousel(State.Search()).spin()
