#!/usr/bin/env python3


import yaml

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from ament_index_python import get_package_share_directory
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Transform
from std_msgs.msg import Int32, String

import xacro
from urdf_parser_py.urdf import URDF

import numpy as np
from scipy.spatial.transform import Rotation

from marine_presenter.srv import ButtonPress

import os
import sys
from copy import deepcopy

# rotation shortcuts
def Rot2mat(R):
    try:
        return R.as_dcm()
    except:
        return R.as_matrix()
def mat2Rot(R):
    try:
        return Rotation.from_dcm(R)
    except:
        return Rotation.from_matrix(R)

def euler2mat(r, p, y):  
    return Rot2mat(Rotation.from_euler('xyz',(r,p,y)))    
def axangle2mat(r):
    return Rot2mat(Rotation.from_rotvec(r))

def mat2axangle(R):
    return mat2Rot(R).as_rotvec()
def mat2quat(R,q):
    q.x,q.y,q.z,q.w = mat2Rot(R).as_quat()    

def HomogeneousFrom(t, R):
    return np.matrix(np.vstack((np.hstack((R, t)), [0,0,0,1])))
    
def Homogeneous(pose):
    R = euler2mat(pose[3], pose[4], pose[5])
    t = np.array(pose[:3]).reshape(3,1)
    M = HomogeneousFrom(t, R)
    for i in range(3):
        for j in range(3):
            for v in [-1,0,1]:
                if abs(M[i,j]-v) < 1e-3:
                    M[i,j] = v
    return M

def HomogeneousInverse(M):
    Minv = M.copy()
    Minv[:3,:3] = M[:3,:3].T
    Minv[:3,[3]] = -Minv[:3,:3]*M[:3,[3]]
    return Minv

def mat2tf(M, tr):
    tr.translation.x = M[0,3]
    tr.translation.y = M[1,3]
    tr.translation.z = M[2,3]    
    mat2quat(M[:3,:3], tr.rotation)        
        

dt = 0.05

class Demo(Node):
    def __init__(self, name):

        super().__init__(name,
                allow_undeclared_parameters=True,
                automatically_declare_parameters_from_overrides=True)

        self.js_pub = self.create_publisher(JointState, '/farm/joint_states', 1)
        self.joints = JointState()
        self.joints.name = ['turbine_1_1']
        self.joints.position = [0.]


        self.br = TransformBroadcaster(self)
        self.tr = TransformStamped()
        self.tr.header.frame_id = 'world'
        self.tr.child_frame_id = 'bluerov2/base_link'
        self.tr.transform.translation.y = 0.

        self.timer = self.create_timer(dt, self.update)
        
    def update(self):

        now = self.get_clock().now()
        t = now.nanoseconds/1000000000.

        self.joints.header.stamp = now.to_msg()
        self.joints.position[0] += 0.1
        self.js_pub.publish(self.joints)

        self.tr.header.stamp = now.to_msg()


        self.tr.transform.translation.z = -3 + 0.5*np.cos(.1*t)
        self.tr.transform.translation.x = -20 + 1.3*np.sin(0.3*t)

        vy = 0.7*np.sin(0.1*t)
        vmax = 1.5
        if abs(vy) > vmax:
            vy *= vmax/abs(vy)

        self.tr.transform.translation.y += vy*dt

        print(f'y: {self.tr.transform.translation.y}')

        mat2quat(euler2mat(0, 0, 0), self.tr.transform.rotation)




        self.br.sendTransform(self.tr)

rclpy.init()    

executor = SingleThreadedExecutor()

executor.add_node(Demo('presenter'))

executor.spin()

#processes.stop()
#presenter.destroy_node()
#rclpy.shutdown()
   
