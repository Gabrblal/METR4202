#!/usr/bin/python

import rospy
import tf
import geometry_msgs.msg
from std_msgs.msg import String
from fiducial_msgs.msg import FiducialTransformArray
from scipy.spatial.transform import Rotation
from modern_robotics import RpToTrans
import numpy as np

def callback(data):
    home_id = 10
    home_T = None
    T_ch = None
    rospy.loginfo(rospy.get_name() + " -> ")
    transforms = [info for info in data.transforms]
    transforms.sort(key=lambda x : x.fiducial_id)
    T_list = []

    for fiducial in transforms:
        # print(fiducial.fiducial_id)
        # check if current id is the home id
        if fiducial.fiducial_id == home_id:
            T_ch = converter(fiducial)
            T_list.append(T_ch)
            continue

        # get data for other id
        T_cb = converter(fiducial)
        if len(transforms) > 1:
            # frame transformation
            T_hb = np.linalg.inv(T_ch) @ T_cb
            # print(f'T_hb: {T_hb}')
            T_list.append(T_hb)

def get_fiducial():
    rospy.init_node('luggage_info', anonymous=True)
    rospy.Subscriber('fiducial_transforms', FiducialTransformArray, callback)
    rospy.spin()

def give_frames():
    pass

def converter(fiducial_info):
    # need to add an offset so that transform is in the middle of 
    p = [fiducial_info.transform.translation.x*1000,
         fiducial_info.transform.translation.y*1000,
         fiducial_info.transform.translation.z*1000]
    R = Rotation.from_quat([fiducial_info.transform.rotation.x,
               fiducial_info.transform.rotation.y,
               fiducial_info.transform.rotation.z,
               fiducial_info.transform.rotation.w])
    R = R.as_matrix()
    T = RpToTrans(R, p)
    # print(f'position: {p} \n')
    # print(f'rotation: {R} \n')
    # print(f'frame: {T}')
    return T

if __name__ == "__main__":
    get_fiducial()