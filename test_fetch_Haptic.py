import time
import struct
import sys
import copy

import rospy
import rospkg

from mujoco_py import load_model_from_path, MjSim, MjViewer
import os

import numpy as np
from transforms3d.quaternions import (
    mat2quat,
    quat2mat
    )
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion
)
from std_msgs.msg import (
    Header,
    String,
    Float32MultiArray
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest
)
from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker


def scale_x(x):
    scale_mat = np.diag([1.0/160,1.0/70,1.0/200])
    #scale_mat = np.eye(3)
    return np.matmul(scale_mat,x)

from baxter_interface import CHECK_VERSION
from time import sleep

baxter_transform = np.asarray([
                        [0,0,1],
                        [1,0,0],
                        [0,1,0],
                        ])

delta_eps = 0.1

class haptic_pos:
    def __init__(self):
        self.hd_vel = np.zeros((3,1))
        self.hd_ang_vel = np.zeros((3,1))
        self.hd_transform = np.eye(4)
        self.hd_position = np.zeros((3,1))
        self.baxter_transform = np.asarray([
                                [0,0,1,0],
                                [1,0,0,0],
                                [0,1,0,0],
                                [0,0,0,1]
                                ])
        self.hd_button1 = False
        self.hd_button2 = False
        rospy.Subscriber('pose_msg',Float32MultiArray,self.callback)
    def callback(self,data_stream):
        self.hd_transform = np.reshape(data_stream.data[0:16],(4,4),order='F')
        #self.hd_transform = np.matmul(self.baxter_transform,self.hd_transform)
        self.hd_vel = np.asarray(data_stream.data[16:19])
        #self.hd_vel = np.matmul(self.baxter_transform[0:3,0:3],self.hd_vel)
        self.hd_ang_vel = np.asarray(data_stream.data[19:22])
        #self.hd_ang_vel = np.matmul(self.baxter_transform[0:3,0:3],self.hd_ang_vel)
        #self.hd_position = np.asarray(data_stream.data[22:25])
        if data_stream.data[22]==1:
            self.hd_button1 = True
        else:
            self.hd_button1 = False
        if data_stream.data[23]==1:
            self.hd_button2 = True
        else:
            self.hd_button2 = False

def R_offset(hd_transform_0,b_transform_0):
    return np.matmul(hd_transform_0.T,b_transform_0)

def main():
    log_joint_angles = []
    log_pos = []
    log_haptic = []
    rospy.init_node('Control_fetch')

    phantom = haptic_pos()

    #Load robot model
    model = load_model_from_path("/home/arclab/Documents/mujoco-py/xmls/fetch/main.xml")

    #Communication rate - 1kHz
    rate = rospy.Rate(100)
    def reset_env():
        pass
    x = 0
    hd_transform_0 = np.matmul(baxter_transform,phantom.hd_transform[0:3,0:3])
    b_ori_q = limb.endpoint_pose()['orientation']
    b_transform_0 = quat2mat([b_ori_q.w,b_ori_q.x,b_ori_q.y,b_ori_q.z])

    R_off = R_offset(hd_transform_0,b_transform_0)

    b_x = np.asarray([x_i for x_i in limb.endpoint_pose()['position']])
    hd_x = scale_x(np.matmul(baxter_transform,phantom.hd_transform[0:3,3]))
    x_off = b_x - hd_x

    rospy.on_shutdown(reset_baxter)

    while not rospy.is_shutdown():
        cur_joint_angles = limb.joint_angles()
        cur_pos = limb.endpoint_pose()['position']
        hd_ori = np.matmul(np.matmul(baxter_transform,phantom.hd_transform[0:3,0:3]),R_off)

        ori = mat2quat(hd_ori)
        cur_ori = limb.endpoint_pose()['orientation']
        #Get increment from haptic device
        alpha = 0.001
        delta_pos = alpha*np.matmul(baxter_transform,phantom.hd_vel)
        hd_x = scale_x(np.matmul(baxter_transform,phantom.hd_transform[0:3,3]))
        #print(delta_pos)
        #log_haptic.append([delta_pos[0],delta_pos[1],delta_pos[2]])


        if not phantom.hd_button1:
            joint_angles = np.asarray([limb.joint_angle(joint) for joint in limb.joint_names()])
            des_x = hd_x + x_off
            des_ori  = baxter_interface.Limb.Quaternion(w=ori[0],x=ori[1],y=ori[2],z=ori[3])

            p_pos = Point(des_x[0],des_x[1],des_x[2])
            p_pose= PoseStamped(header=hdr_p,pose=Pose(position= p_pos,orientation= des_ori))
            rviz_pub.publish(p_pose)
            #Baxter's new position
            #new_pos = Point(cur_pos.x+delta_pos[0],cur_pos.y+delta_pos[1],cur_pos.z+delta_pos[2])
            des_pos = Point(des_x[0],des_x[1],des_x[2])

            #Control fetch

        else:
            while phantom.hd_button1:
                pass

            #Reset R_offset
            hd_transform_0 = np.matmul(baxter_transform,phantom.hd_transform[0:3,0:3])
            b_ori_q = limb.endpoint_pose()['orientation']
            b_transform_0 = quat2mat([b_ori_q.w,b_ori_q.x,b_ori_q.y,b_ori_q.z])
            R_off = R_offset(hd_transform_0,b_transform_0)

            b_x = np.asarray([x_i for x_i in limb.endpoint_pose()['position']])
            hd_x = scale_x(np.matmul(baxter_transform,phantom.hd_transform[0:3,3]))
            x_off = b_x - hd_x


        rate.sleep()


if __name__ == "__main__":
    main()
