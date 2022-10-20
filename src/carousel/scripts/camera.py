#!/usr/bin/python

import numpy as np
import rospy as ros

from fiducial_msgs.msg import FiducialTransformArray
from scipy.spatial.transform import Rotation
from modern_robotics import RpToTrans, TransToRp

from utility.topics import Topics

class Camera:

    def __init__(self):
        self._cube_offset = np.asarray([0, 16, 0])
        self._offset = np.asarray([0, 80, 7]) + self._cube_offset
        self._T_ch = None
        self._home_id = 2

        self._fiducial_pub = Topics.correct_frames.publisher()
        self._fiducial_sub = Topics.fiducial_transforms.subscriber(self._callback)

    def _callback(self, data):

        transforms = list(data.transforms)

        if not transforms:
            return

        transforms.sort(key = lambda x : x.fiducial_id)

        if transforms[0].fiducial_id == self._home_id:
            if self._T_ch is None:
                self._T_ch = self._converter(transforms[0])
                ros.loginfo('Registered home position')
            transforms.pop(0)
        elif self._T_ch is None:
            return

        for fiducial in transforms:

            # get data for other id
            T_cb = self._converter(fiducial)
    
            # frame transformation
            T_hb = np.linalg.inv(self._T_ch) @ T_cb
            # print(f'T_hb: {T_hb}')
            # T_list.append(T_hb)
            R, (x,y,z) = TransToRp(T_hb)
            x += self._offset[0]
            y += self._offset[1]
            z += self._offset[2]

            r = fiducial.transform.rotation
            p = fiducial.transform.translation
            # ros.loginfo(f'    ({p.x}, {p.y}, {p.z}) -> {x}, {y}, {z})')

            fiducial.transform.translation.x = x
            fiducial.transform.translation.y = y
            fiducial.transform.translation.z = z

            x,y,z,w = Rotation.as_quat(Rotation.from_matrix(R))
            fiducial.transform.rotation.x = x
            fiducial.transform.rotation.y = y
            fiducial.transform.rotation.z = z
            fiducial.transform.rotation.w = w                       

        self._fiducial_pub.publish(
            FiducialTransformArray(transforms = transforms)
        )

    def _converter(self, fiducial_info):
        # need to add an offset so that transform is in the middle of 
        p = [fiducial_info.transform.translation.x*1000,
            fiducial_info.transform.translation.y*1000,
            fiducial_info.transform.translation.z*1000]
        R = Rotation.from_quat([
                fiducial_info.transform.rotation.x,
                fiducial_info.transform.rotation.y,
                fiducial_info.transform.rotation.z,
                fiducial_info.transform.rotation.w
        ])
        R = R.as_matrix()
        T = RpToTrans(R, p)
        # print(f'position: {p} \n')
        # print(f'rotation: {R} \n')
        # print(f'frame: {T}')
        return T

    def main(self):
        ros.spin()

if __name__ == "__main__":
    ros.init_node('CarouselCameraTransformNode', anonymous=True)
    Camera().main()
