#! /usr/bin/python3

from typing import List
from collections import deque
from threading import Lock

import rospy as ros

from numpy import ndarray, asarray, arctan2, pi, vstack, subtract, average, sqrt
from numpy import cos, sin
from numpy.linalg import norm
from scipy.spatial.transform import Rotation

from utility.robot import carousel
from utility.kinematics import angle_wrap, inverse_analytical_4R
from utility.topics import Topics, Pose, Float32, JointState, Header, String
from utility.statemachine import StateMachine, State

class Carousel(StateMachine):
    """The carousel logic controller node.

    Takes the locations of the cubes and the current end effector location, and
    outputs the desired end effector location and the gripper percentage open.
    """

    def __init__(
            self,
            first : State,
            throw = False,
            catch = False
        ):
        """Create a new carousel logic controller instance.

        Args:
            first: The first state of the carousel, usually State.Search().
            throw: Whether to throw the cubes to their locations.
            catch: Whether to attempt to catch the cubes as they rotate.
        """
        super().__init__(first)

        # Condition whether robot must throw block.
        self._throw = throw
        self._catch = catch

        # Dictionary of fiducial ID to ((x, y, z), yaw_to, yaw_of)
        self._cube : dict = {}
        self._cube_lock = Lock()

        # Fiducial id of the selected cube to pickup.
        self._best_cube = None

        # The current end effector position.
        self._effector_position = None
        self._effector_lock = Lock()

        # The currently detected colour.
        self._colour = None
        self._colour_lock = Lock()

        # The current end effector pitch to set.
        self._pitch = -pi/4

        # Orientation error threshold for choosing a cube.
        self._centre = asarray([0, 190, 0])
        self._orientation_threshold = 20 * pi / 180

        # Threshold to destination location before the end effector
        # has considered to be moved.
        self._joints = ('joint_1', 'joint_2', 'joint_3', 'joint_4')
        self._movement_threshold = 5

        # Distance threshold in mm per rotation_wait in seconds below
        # which is considered stopped.
        self._rotation_wait = 0.4
        self._rotation_threshold = 0.01 * pi / 180

        # Position to hold cube while checking its colour.
        self._colour_check_position = asarray([0, 155, 280])

        # The default home position of the arm.
        self._home_position = asarray([0, 10, 390])

        # Mapping of colours to their dropoff locations.
        self._dropoff = {
            'red': asarray([-150, -50, 40]),
            'green': asarray([-50, -150, 40]),
            'blue': asarray([50, -150, 40]),
            'yellow': asarray([150, -50, 40]),
        }

        # Input. The positions of the cubees and the end effector location.
        self._cube_sub = Topics.correct_frames.subscriber(self._cube_callback)
        self._effector_sub = Topics.effector.subscriber(self._end_effector_callback)
        self._colour_sub = Topics.block_colour.subscriber(self._colour_callback)

        # Output. The desired end effector location and gripper open percentage.
        self._effector_desired_pub = Topics.effector_desired.publisher()
        self._gripper_pub = Topics.gripper.publisher()
        self._joint_pub = Topics.desired_joint_states.publisher()

    def _end_effector_callback(self, data : Pose):
        """Callback on receiving a new end effector location.

        Args:
            data: The Pose message containing the end effector position.
        """
        self._effector_lock.acquire()

        p = data.position
        self._effector_position = asarray([(p.x, p.y, p.z)])

        self._effector_lock.release()

    def _cube_callback(self, data : List[Pose]):
        """Callback to record the current cube poses.

        Args:
            The poses of the current cubes detected.
        """
        self._cube_lock.acquire()

        for cube in data.transforms:

            # Get the fiducial id, position and rotation matrix.
            id = cube.fiducial_id
            p = cube.transform.translation
            R = cube.transform.rotation

            # Get the yaw of the cube in the space frame.
            yaw, _, _ = Rotation.from_quat(
                [R.x, R.y, R.z, R.w]
            ).as_euler('ZYX')

            # Set the cubes position, yaw from the origin and yaw of the cube.
            self._cube[id] = (
                asarray([p.x, p.y, p.z]), # x,y z position
                arctan2(p.y, p.x), # Yaw from robot to cube
                yaw # Yaw of cube itself.
            )

        self._cube_lock.release()

    def _colour_callback(self, msg : String):
        """Callback on receiving a new colour.
        
        Args:
            msg: A String message containing the colour as a string.
        """
        self._colour_lock.acquire()
        self._colour = msg.data
        self._colour_lock.release()

class Search(State):
    """The search state is responible for selecting a cube to pickup.
    
    Chooses cubes that have an orientation matching the end effector
    when attempting to pickup, and selects the cube furthest away
    from the cube cluster centre.
    """

    def _has_stopped(self): 
        """Waits until the carousel has stopped.

        Returns:
            If the platform has stopped rotating.
        """

        # Select the first cube to track its change in yaw.
        id = tuple(self.machine._cube.keys())[0]

        self.machine._cube_lock.acquire()
        yaw0 = self.machine._cube[id][2]
        self.machine._cube_lock.release()

        # Get the yaw of the cube at two different times.
        ros.sleep(self.machine._rotation_wait)

        self.machine._cube_lock.acquire()
        yaw1 = self.machine._cube[id][2]
        self.machine._cube_lock.release()

        # Get the difference in yaw.
        difference = angle_wrap(yaw0 - yaw1)

        ros.loginfo(f'{yaw0} - {yaw1} = {difference} < {self.machine._rotation_threshold}')

        # The platform has stopped if the yaw has approximately not changed.
        return difference <= self.machine._rotation_threshold

    def _desired_cube(self):
        """Selects the best cube to pickup.

        Returns:
            The position, yaw to the cube and yaw of the cube of
            the most ideal cube to pickup.
        """
        self.machine._cube_lock.acquire()
        ros.loginfo('Choosing cube.')

        # Get the fiducial cube identifiers.
        cubes = [key for key in self.machine._cube.keys()]

        # If there is only one cube then try and pick it up.
        if len(cubes) == 1:
            self.machine._cube_lock.release()
            return cubes[0]

        # Filter by orientation.
        for id in cubes:
            position = self.machine._cube[id][0]
            yaw_to = self.machine._cube[id][1]
            yaw_of = self.machine._cube[id][2]

            # Being a cube, any of the cardinal directions will work.
            error = min([
                abs(angle_wrap(yaw_to - yaw))
                for yaw in (yaw_of, yaw_of + pi/2, yaw_of + pi, yaw_of + 3*pi/2)
            ])

            # If the orientation error is greater than the threshold then
            # remove it from the available cubes.
            if error > self.machine._orientation_threshold:
                cubes.pop(cubes.index(id))
                ros.loginfo(f'Cube {id} eliminated for orientation error {error}.')
                continue

            # 
            if norm(position[0:2]) > carousel.L[1] + carousel.L[2] + carousel.L[2] / sqrt(2):
                cubes.pop(cubes.index(id))
                ros.loginfo(f'Cube {id} elinated for too far.')
                continue
    
        # If no cubes are grabbable then return no desired cubes.
        if not cubes:
            self.machine._cube_lock.release()
            return None

        # Calculate the distances away from the cube cluster centre.
        ps = vstack([self.machine._cube[id][0] for id in cubes])
        mu = average(ps, axis = 0).reshape([3, 1])
        distances = norm(subtract(ps, mu.reshape([3, 1]).T), axis = 0)

        # Get the cube furthest away from the cluster centre.
        best = cubes[distances.argmax()]
        self.machine._cube_lock.release()

        return best

    def main(self):
        """Waits for the conveyor to stop moving, selects the best cube
        and the transitions to the Move state to the cube and the Pickup
        state to pick it up."""

        ros.loginfo('Entered search state.')

        while not ros.is_shutdown():

            # Ensure no busy loop.
            ros.sleep(0.01)

            # Wait until there are cubes to pickup.
            self.machine._cube_lock.acquire()
            no_cubes = not self.machine._cube
            self.machine._cube_lock.release()

            if no_cubes:
                continue

            # If catching is enabled, then catch.
            if self.machine._catch:
                self.machine._cube_lock.acquire()
                self.machine._best_cube = list(self.machine._cube.keys())[0]
                self.machine._cube_lock.release()
                return State.Catch()

            # Wait until the carousel has stopped.
            if not self._has_stopped():
                continue

            ros.loginfo('Guessed conveyor stopped.')

            # Select the best cube.
            self.machine._best_cube = self._desired_cube()

            # If there are no good cubes to pick up then continue searching.
            if self.machine._best_cube is None:
                continue

            ros.loginfo(f'Selected cube {self.machine._best_cube}.')

            # Set the pickup position to 20mm above the cube.
            self.machine._cube_lock.acquire()
            position = self.machine._cube[self.machine._best_cube][0] 
            self.machine._cube_lock.release()

            ros.loginfo(f'Pickup position at {tuple(position)}.')

            # Tune adjustment factors for distance to cube.
            if norm(position[0:2]) > carousel.L[1] + carousel.L[2]:
                position = position + asarray([0, 0, 30])
                pitch = -pi/4
                show_pitch = pi/4
            else:
                position = position + asarray([0, -10, 25])
                pitch = -pi/2 + pi/40
                show_pitch = 0

            self.machine._cube_lock.acquire()
            yaw_to = self.machine._cube[self.machine._best_cube][1]
            self.machine._cube_lock.release()

            # Tuning rotaton adjustment factor for left side.
            if position[0] < 0:
                adj = 2 * pi / 180
                position[0] = position[0] * cos(adj) - position[1] * sin(adj)
                position[1] = position[0] * sin(adj) + position[1] * cos(adj)

            # Default go to the pickup location.
            return State.Move(
                position,
                State.PickUp(pitch = show_pitch),
                velocity = 0.05,
                pitch = pitch
            )

class Move(State):
    """State responsible for moving the end effector to a location and
    then transitioning to the state afterwards."""

    def __init__(
            self,
            position : ndarray,
            after_state : State,
            *,
            velocity : float = 1.0,
            pitch : float = None,
            wait : bool = True
        ):
        """Create the movement state temporary data.
        
        Args:
            position: The position to move to.
            after_state: The state to transition to after the move.
            velocity: The percentage of maximum speed to move at.
            pitch: The end effector pitch.
            wait: Whether to wait for the end effector to get to the desired location.
        """
        self._position = asarray(position).flatten()
        self._after_state = after_state
        self._velocity = velocity
        self._pitch = pitch
        self._wait = wait

    def main(self):
        return self.main_no_trajectory()

    def main_no_trajectory(self):
        """Sends the desired joint """
        ros.loginfo('Entered move state.')

        if self._pitch is not None:
            self.machine._pitch = self._pitch

        # Calculate the angles to get to the desired end effector position.
        theta = inverse_analytical_4R(self._position, carousel.L, self.machine._pitch)

        vel = [self._velocity] * len(self.machine._joints)
        ros.loginfo(f'Velocity: {vel}')

        self.machine._joint_pub.publish(
            JointState(
                header = Header(stamp = ros.Time.now()),
                name = self.machine._joints,
                position = theta,
                velocity = vel
            )
        )

        ros.sleep(0.1)

        # Skip the waiting if desired.
        if not self._wait:
            return self._after_state

        # Queue of recent differences in position.
        recent = deque(maxlen = 4)

        # Wait to get to the next position.
        while not ros.is_shutdown():
            ros.sleep(0.02)

            self.machine._effector_lock.acquire()

            # If no end effector position then wait for one.
            if self.machine._effector_position is None:
                self.machine._effector_lock.release()
                continue

            # Get the current end effector position.
            recent.append(self.machine._effector_position)
            self.machine._effector_lock.release()

            if len(recent) != recent.maxlen:
                continue

            moved = all(norm(d - recent[0]) < self.machine._movement_threshold for d in recent)
            moved &= norm(self.machine._effector_position - self._position) < 75

            # If all the recent positions are approximately equal.
            if moved:
                ros.loginfo(f'Moved to {tuple(self._position)}!')
                break

        return self._after_state

class Catch(State):

    def __init__(self, align = True):
        self._align = align
        self._timeout = ros.get_time() + 15

    def _get_position(self):

        # True for positive rotation, False for negative rotation.
        yaw = None

        # Recent rotation samples.
        recent = []

        # Calculate the angular direction of the conveyor.
        while not ros.is_shutdown():

            if ros.get_time() > self._timeout:
                return None

            # Get the position of the cube
            self.machine._cube_lock.acquire()
            position, _, yaw0 = self.machine._cube[self.machine._best_cube]
            self.machine._cube_lock.release()

            # Angular velocity of the cube.
            ros.sleep(0.5)

            self.machine._cube_lock.acquire()
            _, _, yaw1 = self.machine._cube[self.machine._best_cube]
            self.machine._cube_lock.release()

            if yaw0 > pi / 2 and yaw1 < -pi / 2:
                yaw1 += 2 * pi
            elif yaw0 < -pi / 2 and yaw1 > pi / 2:
                yaw1 -= 2 * pi

            ros.loginfo(f'{yaw0} > {yaw1}')

            # True if increasing in angle.
            recent.append(yaw0 > yaw1)

            if len(recent) >= 3:

                # Positive rotation around z.
                if all(x == True for x in recent):
                    yaw = pi

                # Negative rotation around z.
                elif all(x == False for x in recent):
                    yaw = 0

                # Undetermined.
                else:
                    recent.clear()
                    continue

                break

        # Position of the cube relative to the centre
        pos = position - self.machine._centre
        x, y, z = pos[0], pos[1], pos[2]

        # Radius away from the centre.
        r = min(sqrt(x **2 + y ** 2) + 15, 230 / 2)
        ros.loginfo(f'r: {r}')

        x = self.machine._centre[0] + r * cos(yaw)
        y = self.machine._centre[1] + 20
        z += 20

        return asarray([x, y, z])

    def _is_clear(self, left : bool):
        """Check if the front left or right has no cubes.

        Args:
            left: True to check the left side or False for the right.
        
        Returns:
            True if the area is clear.
        """
        self.machine._cube_lock.acquire()

        for _, cube in self.machine._cube.items():
            x, y = cube[0][0], cube[0][1]

            if y > self.machine._centre[1] + 30:
                continue

            if left and x > 20:
                continue
            elif not left and x < -20:
                continue

            self.machine._cube_lock.release()
            return False

        self.machine._cube_lock.release()
        return True

    def _on_timeout(self):
        # Remove the tracked cube.
        self.machine._cube_lock.acquire()
        self.machine._cube.pop(self.machine._best_cube)
        self.machine._cube_lock.release()

        # Reset the home position.
        return State.Move(
            self.machine._home_position,
            State.Search(),
            velocity = 0.5,
            pitch = pi/2
        )

    def main(self):

        if self._align:

            self.machine._cube_lock.acquire()
            position, _, _ = self.machine._cube[self.machine._best_cube]
            self.machine._cube_lock.release()

            # If around the centre then just pick it up.
            if abs(position[0] - self.machine._centre[0]) < 35:
                if abs(position[1] - self.machine._centre[1]) < 35:
                    return State.Move(
                        position + asarray([0, 0, 20]),
                        State.PickUp(pitch = 0),
                        velocity = 0.05,
                        pitch = -pi/2 + pi/20
                    )

            # Get the position to catch the cube at. Timeout may occur
            # when sampling rotation direction.
            catch_position = self._get_position()
            if catch_position is None:
                return self._on_timeout()

            # Left if the position has a negative x coordinate.
            left = catch_position[0] < 0

            while not ros.is_shutdown():
                ros.sleep(0.1)

                # If timed out waiting for area to clear then timeout.
                if ros.get_time() > self._timeout:
                    return self._on_timeout()

                # If not clear then wait to be clear.
                if not self._is_clear(left):
                    continue

                # Cleared, move to the catch position.
                return State.Move(
                    catch_position,
                    State.Catch(align = False),
                    pitch = 0
                )

        # Queue of recent differences in position.
        recent = deque(maxlen = 4)

        # Wait to get to the next position.
        while not ros.is_shutdown():
            ros.sleep(0.1)

            if ros.get_time() > self._timeout:
                return self._on_timeout()

            self.machine._cube_lock.acquire()
            position, _, _ = self.machine._cube[self.machine._best_cube]
            self.machine._cube_lock.release()

            # Get the current end effector position.
            recent.append(position)

            if len(recent) != recent.maxlen:
                continue

            moved = all(norm(d - recent[0]) < self.machine._movement_threshold for d in recent)
            moved &= norm(self.machine._effector_position - position) < 30

            # If all the recent positions are approximately equal.
            if moved:
                return State.PickUp(pitch = pi/2)

class PickUp(State):
    """State responsible for picking up the cube."""

    def __init__(self, pitch = pi / 4):
        self._pitch = pitch

    def main(self):
        """main loop"""
        ros.loginfo(f'Entered pickup state.')

        percent = Float32()
        percent.data = 0.0

        ros.loginfo(f'Closing gripper.')
        self.machine._gripper_pub.publish(percent)
        ros.sleep(0.15)

        return State.Move(
            self.machine._colour_check_position,
            State.ColourCheck(),
            velocity = 0.5,
            pitch = self._pitch
        )

class ColourCheck(State):

    def main(self):
        ros.loginfo('Entered colour checking state.')

        timeout = ros.get_time() + 1

        while not ros.is_shutdown():
            ros.sleep(0.01)

            if ros.get_time() > timeout:
                ros.loginfo('Timed out finding colour.')

                ros.loginfo(f'Opening gripper.')
                percent = Float32()
                percent.data = 1.0
                self.machine._gripper_pub.publish(percent)

                # Don't go for a cube that dones't exit.
                self.machine._colour_lock.acquire()
                colour = None
                self.machine._colour_lock.release()

                self.machine._cube.pop(self.machine._best_cube)

                return State.Move(
                    self.machine._home_position,
                    State.Search(),
                    velocity = 0.5,
                    pitch = pi/2
                )

            self.machine._colour_lock.acquire()
            colour = self.machine._colour
            self.machine._colour_lock.release()

            if colour not in self.machine._dropoff.keys():
                continue

            ros.loginfo(f'Cube colour is {colour}.')
            position = self.machine._dropoff[colour]

            if self.machine._throw == True:
                return State.Move(
                    position,
                    State.Throw(),
                    velocity = 0.6,
                    pitch = -pi/2,
                    wait = False
                )

            return State.Move(
                position,
                State.DropOff(),
                velocity = 0.5,
                pitch = -pi/2
            )

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

        return State.Move(
            self.machine._home_position,
            State.Search(),
            velocity = 0.5,
            pitch = pi/2
        )

class Throw(State):

    def main(self):
        """main loop"""
        ros.loginfo('Entered throw state.')

        ros.sleep(0.2)
        ros.loginfo('Opening gripper.')
        percent = Float32()
        percent.data = 1.0
        self.machine._gripper_pub.publish(percent)

        ros.sleep(0.2)

        self.machine._cube_lock.acquire()
        self.machine._cube.pop(self.machine._best_cube)
        self.machine._cube_lock.release()

        return State.Move(
            self.machine._home_position,
            State.Search(),
            velocity = 0.5,
            pitch = pi/2
        )

if __name__ == '__main__':
    ros.init_node('CarouselLogicNode')

    # Get whether the carousel will throw the cube.
    throw = ros.get_param('throw', False)
    catch = ros.get_param('catch', False)

    ros.loginfo(f'Throwing is {throw}.')
    ros.loginfo(f'Catching is {catch}.')

    Carousel(
        State.Search(),
        throw = throw,
        catch = catch
    ).spin()
