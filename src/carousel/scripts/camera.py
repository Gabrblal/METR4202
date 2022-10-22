#!/usr/bin/python

import numpy as np
import rospy as ros

from fiducial_msgs.msg import FiducialTransformArray

from utility.topics import Topics, Bool

class Camera:

    def __init__(self):
        
        # Offset from the the camera to the base of the robot.
        self._camera_offset = np.asarray([0, 200, -588])

        self._cube_offset = np.asarray([14, 14, 32])

        self._fiducial_pub = Topics.correct_frames.publisher()
        self._fiducial_sub = Topics.fiducial_transforms.subscriber(self._callback)

    def _callback(self, data):

        # Get the fiduicial transformation matricies. Do nothing if there are
        # none.
        transforms = list(data.transforms)
        if not transforms:
            return

        for fiducial in transforms:

            # Convert to mm and add the camera offset. Flip the y axis from the
            # camera to the robot axis.
            p = np.asarray([
                fiducial.transform.translation.x * 1000,
                fiducial.transform.translation.y * -1000,
                fiducial.transform.translation.z * 1000
            ]) + self._camera_offset + self._cube_offset

            # Update the translation of the cube.
            fiducial.transform.translation.x = p[0]
            fiducial.transform.translation.y = p[1]
            fiducial.transform.translation.z = 32 # p[2]

        self._fiducial_pub.publish(
            FiducialTransformArray(transforms = transforms)
        )

    def main(self):
        ros.spin()

if __name__ == "__main__":
    ros.init_node('CarouselCameraTransformNode', anonymous=True)
    Camera().main()
