'''
//|    Copyright (C) 2024 Cybernitic Laboratory (CynLr) 
//|    website:  http://www.cynlr.com
//|
'''

import numpy as np
import pinocchio
from sys import argv
from os.path import dirname, join, abspath
from scipy.spatial.transform import Rotation as rot
#import utils

class ModelWrapper:
    def __init__(self, urdf_filename, joint_id, frame_name, n_dof, world_H_base, offset_ee):
        """
        Initialize the robot variables, which is like a wrapper around pinocchio for robot's variables
        relevant for inverse kinematics and dynamics

        Parameters:
        - urdf_filename : string of file to the urdf file of the robot
        - joint_id : integer of the joint id for which inverse kinematics is computed
        - frame_name : string of the frame name for which inverse kinematics is computed
        - n_dof : integer of the number of degrees of freedom
        - world_H_base : numpy array of 4x4 matrix representing the transformation of the base frame with respect to the world
        - offset_ee : numpy array of 3x1 representing the offset of the end-effector frame with respect to the link frame

        """
        self.urdf_filename = urdf_filename
        self.n_dof = n_dof
        # self.n_effectors = n_effectors  # number of effectors to be considered for the kinematics

        # Load the urdf model
        self.robot_model = pinocchio.buildModelFromUrdf(self.urdf_filename)
        # Create data required by the robot model algorithms
        self.robot_data = self.robot_model.createData()
        # Sample a random configuration
        self.q_m = pinocchio.randomConfiguration(self.robot_model)      # joint_pos model        
        # self.dq_m = np.zeros([self.q_m.size, 1])                      # joint_vel model
        self.dq_m = np.zeros(self.q_m.size)                             # joint_vel model

        # Extract the joint limits
        self.q_min  =  self.robot_model.lowerPositionLimit
        self.q_max  =  self.robot_model.upperPositionLimit
        self.dq_min = -self.robot_model.velocityLimit
        self.dq_max =  self.robot_model.velocityLimit
        self.t_min  = -self.robot_model.effortLimit
        self.t_max  =  self.robot_model.effortLimit

        # Set the robot's base transformation
        self.world_H_base = world_H_base                                # homogeneous matrix of the robot base wrt. the world frame

        # set the specified joint id
        self.joint_id = joint_id
        # set the specified frame name
        self.frame_name = self.robot_model.frames[-1].name if frame_name == "" else frame_name
        self.frame_id = self.robot_model.getFrameId(self.frame_name)
        # Set the ee offset
        self.offset_ee = offset_ee
        # end-effector position and orientation wrt the base frame
        self.pos_ee_base   = np.zeros([3,1])     
        self.rot_ee_in_base = np.eye(3,3)
        # end-effector position and orientation wrt he world frame
        self.pos_ee_world = np.zeros([3,1])
        self.rot_ee_world = self.world_H_base[:3, :3]
        self.rot_mx_6x6_ee  = np.eye(6,6)
        # End-effector velocity kinematics variables
        self.Jac_world  = np.zeros([6, self.robot_model.nv])            # Jacobian (Geometric) Matrix of the robot EE
        self.dJac_world = np.zeros([6, self.robot_model.nv])            # Derivative of the Jacobian matrix EE
        self.dJacdq = np.zeros([6, 1])                                  # dJac * dq 

        # velocity twist transformation
        self.ee_W6x6_o = self.get_twist_transformation_mx(self.offset_ee)


    def updateRobotModel(self, q, dq):      
        self.q_m[:self.n_dof] = q.reshape(-1,)
        self.dq_m[:self.n_dof] = dq.reshape(-1,)

        # forward kinematics
        pinocchio.forwardKinematics(self.robot_model, self.robot_data, self.q_m)

        # update the frame placement for specified ID
        pinocchio.updateFramePlacement(self.robot_model, self.robot_data, self.frame_id)       
        # EE state in base (rotation and translation of EE in the robot's base frame)
        self.rot_ee_in_base = self.robot_data.oMf[self.frame_id].rotation
        self.pos_ee_base = self.rot_ee_in_base @ self.offset_ee + self.robot_data.oMf[self.frame_id].translation.reshape(-1,1)
        
        # EE state in world frame (rotation and translation of EE wrt. world frame)
        self.rot_ee_world = self.world_H_base[:3, :3] @ self.rot_ee_in_base
        self.pos_ee_world = self.world_H_base[:3, :3] @ self.pos_ee_base + self.world_H_base[:3, 3:]

        # 6x6 Rotation matrix of the EE wrt. the world
        self.rot_mx_6x6_ee[:3, :3]   = self.rot_ee_world
        self.rot_mx_6x6_ee[3:6, 3:6] = self.rot_ee_world

        # Jacobian of EE in base and world frame
        Jac = pinocchio.computeFrameJacobian(self.robot_model, self.robot_data, self.q_m, self.frame_id, pinocchio.LOCAL)
        self.Jac_world = self.rot_mx_6x6_ee @  self.ee_W6x6_o @ Jac[:, :self.n_dof]

        # EE velocity in world frame
        self.vel_ee_world = self.Jac_world @ np.array([self.dq_m[:self.n_dof]]).reshape(-1,1) #dq


    def getMassMatrix(self):
        # Mass Matrix
        self.mass_matrix = pinocchio.crba(self.robot_model, self.robot_data, self.q_m)[:self.n_dof, :self.n_dof] 


    def getJacobianTimeDrivative(self):
        pinocchio.updateFramePlacement(self.robot_model, self.robot_data, self.frame_id) 
        pinocchio.computeJointJacobiansTimeVariation(self.robot_model, self.robot_data, self.q_m, self.dq_m)
        dJac = pinocchio.getFrameJacobianTimeVariation(self.robot_model, self.robot_data, self.frame_id, pinocchio.LOCAL)
        self.dJac_world = self.rot_mx_6x6_ee @  self.ee_W6x6_o @ dJac[:, :self.n_dof]
        self.dJdq = self.dJac_world @ self.dq_m.reshape(-1,1)

    
    def get_twist_transformation_mx(self, toe):
    #
    # toe : relative position in reference frame
    # skew symmetric matrix associated with relative position
        skew_mx_oe = np.array([ [          0.0, -toe[2,:1].item(),  toe[1,:1].item()], 
                                [ toe[2,:1].item(),           0.0, -toe[0,:1].item()], 
                                [-toe[1,:1].item(),  toe[0,:1].item(),           0.0]])
        # twist transformation matrix
        ee_W6x6_o   = np.eye(6,6)
        ee_W6x6_o[:3, 3:6]  = skew_mx_oe
        #
        return ee_W6x6_o
    

    def positionEE(self):
        return self.pos_ee_world
    
    def rotationEE(self):
        return self.rot_ee_world
    
    def transformMatrixEE(self):
        # create a homogeneous transformation matrix
        wHEE = np.eye(4,4)
        wHEE[:3, 3:] = self.positionEE()
        wHEE[:3, :3] = self.rotationEE()
        return wHEE

    def velocity_ee(self):
        return self.vel_ee_world

    def jacobian_ee(self):
        return self.Jac_world
    
    def mass_matrix(self):
        return self.mass_matrix







