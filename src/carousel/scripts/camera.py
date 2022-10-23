#!/usr/bin/python

import rospy as ros

from typing import Tuple
from numpy import asarray

from fiducial_msgs.msg import FiducialTransformArray
from utility.topics import Topics

class Camera:
    """The camera node subscribes to the fiducial transformations and converts
    them to the robot frame of reference.

    Expects the fiducial transforms to be in metres.

    Fixes the z offset of all transformed fiducials to the z component of the
    cube offset.

    Expects the x and z axes to be alined, and the y axis to be flipped from
    the camera reference frame to the robot reference frame.
    """

    def __init__(
            self,
            camera_offset : Tuple[float, float, float],
            cube_offset : Tuple[float, float, float]
        ):
        """Create a new camera fiducial offset.

        Args:
            camera_offset: The offset vector as (dx, dy, dz) from the camera to
                the robot frame of reference, in millimetres.
            cube_offset: Offset (dx, dy, dz) from a cube (x, y, z) to the centre
                of the top surface of the cube, in millimetres.
        """
        self._camera_offset = asarray(camera_offset)
        self._cube_offset = asarray(cube_offset)

        self._fiducial_pub = Topics.correct_frames.publisher()
        self._fiducial_sub = Topics.fiducial_transforms.subscriber(self._callback)

    def _callback(self, msg : FiducialTransformArray):
        """Callback on fiducial transform data that transforms fiducial
        transforms to the robot frame of reference.

        Args:
            msg: The message containing a sequence of fiducial transforms.
        """

        # Get the fiducial transforms and do nothing if there are none.
        transforms = list(msg.transforms)
        if not transforms:
            return

        for fiducial in transforms:

            # Convert to mm and add the camera and cube offset.
            # Flip the y axis from the camera to the robot axis.
            p = asarray([
                fiducial.transform.translation.x * 1000,
                fiducial.transform.translation.y * -1000,
                fiducial.transform.translation.z * 1000
            ]) + self._camera_offset + self._cube_offset

            # Update the translation of the cube. Fix the z offset.
            fiducial.transform.translation.x = p[0]
            fiducial.transform.translation.y = p[1]
            fiducial.transform.translation.z = self._cube_offset[2]

        # Publish the updated transform array.
        self._fiducial_pub.publish(
            FiducialTransformArray(transforms = transforms)
        )

    def main(self):
        """Process fiducial transform messages and publish them in millimetres
        in the robot frame of reference.
        """
        ros.spin()

if __name__ == "__main__":
    ros.init_node('CarouselCameraTransformNode', anonymous=True)

    Camera(
        camera_offset = [0, 200, -588],
        cube_offset = [14, 14, 32]
    ).main()
