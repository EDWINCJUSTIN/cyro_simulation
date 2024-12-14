'''
//|    Copyright (C) 2024 Cybernitic Laboratory (CynLr) 
//|    website:  http://www.cynlr.com
//|
'''

import numpy as np
from numpy.linalg import norm, solve
import time
from sys import argv
from os.path import dirname, join, abspath
from scipy.spatial.transform import Rotation as rot
from helper.model_wrapper import ModelWrapper
from qpsolvers import solve_qp

class ParamIK:
    def __init__(self):
        self.step = 0.100
        self.maxIter = 120 #120
        self.tolTrans = 0.0001 #0.01
        self.tolOrient = 0.01 #0.09
        self.vLinMax = 1.5
        self.vAngMax = 3.5
        self.aFilter = 0.9
        self.scaleReg = 0.00001 #0.001
        self.scaleWq0 = 0.00001
        self.scaleWddq = 0.0015
        self.scaleKp = 0.5
        self.scaleMm = 0.005
        self.scaleVelLim = 5.0
        self.weightTask = np.diag(np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0]))
        self.gainPosK = 3.6 * np.eye(3,3)
        self.gainPosD = 2.0 * np.sqrt(self.gainPosK)
        self.gainOriK = 3.2 * np.eye(3,3)
        self.gainOriD = 2.0 * np.sqrt(self.gainOriK)



class InvKinCL:
    def __init__(self, urdf_filename, joint_id, frame_name, n_dof, world_H_base, offset_ee):

        self.param_ik = ParamIK()
        self.robotVar = ModelWrapper(urdf_filename, joint_id, frame_name, n_dof, world_H_base, offset_ee)
        self.des_vel_twist_filt = np.zeros([6,1])
        # Hessian and gradient vector for the QP solver
        self.hessian_mx_pgen = np.zeros([self.robotVar.n_dof, self.robotVar.n_dof])
        self.gradient_vec_pgen = np.zeros([self.robotVar.n_dof])
        # Constraint matrix and vector for the QP solver
        self.cons_mx_state = np.zeros([4*self.robotVar.n_dof, self.robotVar.n_dof])
        self.cons_vec_state = np.zeros([4*self.robotVar.n_dof, 1])
        # QP solution vector
        self.joint_pos_final = np.zeros([self.robotVar.n_dof, 1])
        self.joint_pos_iter = np.zeros([self.robotVar.n_dof, 1])
        self.joint_vel_iter = np.zeros([self.robotVar.n_dof, 1])
        self.joint_vel_n1 = np.zeros([self.robotVar.n_dof, 1])
        self.pose_error_iter = np.zeros([6,1])
        self.wHEE_final = np.eye(4,4)

    
    def compute_orientation_error(self, current_rot_mat, desired_rot_mat):
        #  get the relative rotation in wrt. reference frame
        # relative_rot_mat = np.matmul(current_rot_mat, desired_rot_mat.transpose())
        relative_rot_mat = current_rot_mat @ desired_rot_mat.transpose()
        # compute the rotation oject (scipy transform) and return the axis angle orientation error (wrt. reference frame)
        # print(f'relative_rot_mat is : \n {relative_rot_mat}')
        return rot.from_matrix(relative_rot_mat).as_rotvec()
        # # --------------------------------------------------------------------------------
 

    def compute_pose_error_in_ref(self, cur_transf_mx_ee, des_transf_mx_ee):
        # position error between current and desired position all in the same frame
        pos_error = (cur_transf_mx_ee[:3, 3:] - des_transf_mx_ee[:3, 3:]).reshape(3,1)
        # orientation error using axis/angle representation
        ori_error = self.compute_orientation_error(cur_transf_mx_ee[:3, :3], des_transf_mx_ee[:3, :3]).reshape(3,1)

        return pos_error, ori_error

    def generate_task_motion_twist(self, cur_transf_mx_ee, des_transf_mx_ee, gain_pos, gain_ori):
        #
        pos_error, ori_error = self.compute_pose_error_in_ref(cur_transf_mx_ee, des_transf_mx_ee)

        des_vel_lin = -gain_pos @ pos_error
        des_vel_ang = -gain_ori @ ori_error

        # apply saturation and return the twist velocity
        # putting together the pose error and velocity twist vector
        pose_error = np.array([pos_error, ori_error]).reshape(-1,1)
        des_vel_twist = np.array([des_vel_lin, des_vel_ang]).reshape(-1,1)

        return des_vel_twist, pose_error
        # return sat_vel_lin, sat_vel_ang, pos_error, ori_error 
        
        
    def computeDesiredTaskVariation(self, wHEE, wHEEDes):
        # for now, compute the desired velocity twsit using a forst order dynmics
        des_vel_twist, pose_error = self.generate_task_motion_twist(wHEE, wHEEDes, self.param_ik.gainPosK, self.param_ik.gainOriK)
        #
        # apply velocity limits to linear and angular motion
        if(np.linalg.norm(des_vel_twist[:3, :]) > self.param_ik.vLinMax ):
            des_vel_twist[:3, :] = des_vel_twist[:3, :]/(np.linalg.norm(des_vel_twist[:3, :]) + 1e-10) * self.param_ik.vLinMax
        if(np.linalg.norm(des_vel_twist[3:, :]) > self.param_ik.vAngMax ):
            des_vel_twist[3:, :] = des_vel_twist[3:, :]/(np.linalg.norm(des_vel_twist[3:, :]) + 1e-10) *  self.param_ik.vAngMax
        
        return des_vel_twist, pose_error

    def initializeSolver(self):
        self.joint_vel_iter = solve_qp(P=self.hessian_mx_pgen, q=self.gradient_vec_pgen.T, G=self.cons_mx_state, h=self.cons_vec_state, solver="proxqp").reshape(-1,1)

    def updateModelFromDq(self, joint_pos, joint_vel):
        # update of the joint position 
        # q = joint_pos + joint_vel * self.param_ik.step
        q = joint_pos + 0.5*(self.joint_vel_n1 + joint_vel) * self.param_ik.step

        # enforcing joint position limits through (clamping within the range)
        for i in range(len(q)):
            if(q[i] > self.robotVar.q_max[i]):
                q[i] = self.robotVar.q_max[i]
            elif(q[i] < self.robotVar.q_min[i]):
                q[i] = self.robotVar.q_min[i]

        # create and set the joint velocity vector at zero (to update only configuration)
        dq = np.zeros(q.size).reshape(-1,1)

        # update the robot model
        self.robotVar.updateRobotModel(q, dq)
        joint_pos = q
        self.joint_vel_n1 = joint_vel
        return joint_pos, joint_vel


    def updateQPVariables(self, wHEEDes, joint_pos_0, joint_pos, joint_vel, position_only=False):

        # get the current end-effector transformation matrix of the model
        wHEE = self.robotVar.transformMatrixEE()

        # get the desired task variation
        des_vel_twist, pose_error = self.computeDesiredTaskVariation(wHEE, wHEEDes)
        # filter the desired task velocity twist
        self.des_vel_twist_filt = (1.0 - self.param_ik.aFilter) * self.des_vel_twist_filt + self.param_ik.aFilter * des_vel_twist
        self.pose_error_iter = pose_error

        # weight of desired task
        weightTask = self.param_ik.weightTask.copy()
        if(position_only):
            weightTask[:3, :3] = 100.0 * self.param_ik.weightTask[:3, :3]
      
        # Hessian matrix and gradient vector
        # ------------------------------------
        # desired posture
        joint_vel_0 = -self.param_ik.scaleKp * (joint_pos - joint_pos_0.reshape(-1, 1))
        weight_posture_mx = self.param_ik.scaleWq0 * np.eye(self.robotVar.n_dof, self.robotVar.n_dof)

        if(position_only):
            # QP Hessian
            self.hessian_mx_pgen = (self.robotVar.jacobian_ee()[:3,:].T @ weightTask[:3, :3] @ self.robotVar.jacobian_ee()[:3,:]) \
                + 0.1*self.param_ik.scaleReg * np.eye(self.robotVar.n_dof, self.robotVar.n_dof) + 0.0*weight_posture_mx 
            
            # QP gradient
            self.gradient_vec_pgen = - (self.robotVar.jacobian_ee()[:3,:].T @ weightTask[:3, :3] @ self.des_vel_twist_filt[:3, :]) \
                - 0.0*weight_posture_mx @ joint_vel_0 
        else:
            # QP Hessian
            self.hessian_mx_pgen = (self.robotVar.jacobian_ee().T @ weightTask @ self.robotVar.jacobian_ee()) \
                + 0.1*self.param_ik.scaleReg * np.eye(self.robotVar.n_dof, self.robotVar.n_dof) + 0.0*weight_posture_mx 
            
            # QP gradient
            self.gradient_vec_pgen = - (self.robotVar.jacobian_ee().T @ weightTask @ self.des_vel_twist_filt) \
                - 0.0*weight_posture_mx @ joint_vel_0 
        

        # Joint position constraints
        # ---------------------------
        # constraint matrix
        cons_mx_joint_pos = np.zeros([2*self.robotVar.n_dof, self.robotVar.n_dof])
        cons_mx_joint_pos[:self.robotVar.n_dof, :self.robotVar.n_dof] = - np.eye(self.robotVar.n_dof, self.robotVar.n_dof)
        cons_mx_joint_pos[self.robotVar.n_dof:2*self.robotVar.n_dof, :self.robotVar.n_dof] = np.eye(self.robotVar.n_dof, self.robotVar.n_dof)

        # constraint vector
        cons_vec_joint_pos = np.zeros([2*self.robotVar.n_dof, 1])
        cons_vec_joint_pos[0:self.robotVar.n_dof, :1] = - (self.robotVar.q_min.reshape(-1, 1) - joint_pos)
        cons_vec_joint_pos[self.robotVar.n_dof: 2*self.robotVar.n_dof, :1] = (self.robotVar.q_max.reshape(-1, 1) - joint_pos)

        # Joint velocity constraints
        # ---------------------------
        cons_mx_joint_vel = np.zeros([2*self.robotVar.n_dof, self.robotVar.n_dof])
        cons_mx_joint_vel[:self.robotVar.n_dof, :self.robotVar.n_dof] = - np.eye(self.robotVar.n_dof, self.robotVar.n_dof)
        cons_mx_joint_vel[self.robotVar.n_dof:2*self.robotVar.n_dof, :self.robotVar.n_dof] = np.eye(self.robotVar.n_dof, self.robotVar.n_dof)

        # constraint vector
        cons_vec_joint_vel = np.zeros([2*self.robotVar.n_dof, 1])
        cons_vec_joint_vel[0:self.robotVar.n_dof, :1] = - self.param_ik.scaleVelLim * self.robotVar.dq_min.reshape(-1, 1)
        cons_vec_joint_vel[self.robotVar.n_dof:2*self.robotVar.n_dof, :1] = self.param_ik.scaleVelLim * self.robotVar.dq_max.reshape(-1, 1)

        # Overall constraints vectors
        # ---------------------------
        self.cons_mx_state[:2*self.robotVar.n_dof, :self.robotVar.n_dof] = cons_mx_joint_pos
        self.cons_mx_state[2*self.robotVar.n_dof:4*self.robotVar.n_dof, :self.robotVar.n_dof] = cons_mx_joint_vel

        self.cons_vec_state[:2*self.robotVar.n_dof, :1] = cons_vec_joint_pos
        self.cons_vec_state[2*self.robotVar.n_dof:4*self.robotVar.n_dof, :1] = cons_vec_joint_vel

    def computeInverseKinematics(self, targetPosition, targetRotation = None, targetJointPosture = None, jointPosition = None):
        '''
        Compute the inverse kinematics for the robotic system given the desired position, rotation, and joint posture.

        Parameters:
        targetPosition : np.array of shape ([3,1]) : desired EE position
        targetRotation: rotation matrix (np.array([3,3])
        targetJointPosture: np.array of shape (1, n_joint)
        joint_pos : np.array of shape ([n_joint,1])
        position_only: boolean with defaut value at False

        '''

        flag = True

        if(not np.any(jointPosition)):
            jointPosition = 0.5 * (self.robotVar.q_min + self.robotVar.q_max).reshape(-1,1)
            self.joint_pos_iter = jointPosition
        self.joint_vel_iter = np.zeros([self.robotVar.n_dof, 1])

        if(not np.any(targetRotation)):
            targetRotation = np.eye(3,3)
            position_only = True
        else:
            position_only = False

        if(not np.any(targetJointPosture)):
            targetJointPosture = 0.5 * (self.robotVar.q_min + self.robotVar.q_max)

        #
        ik_stop = False
        count = 0

        self.joint_pos_iter, self.joint_vel_iter = self.updateModelFromDq(self.joint_pos_iter, self.joint_vel_iter)
        wHEE = self.robotVar.transformMatrixEE()
        wHEEDes = wHEE
        wHEEDes[:3, 3:] = targetPosition
        if(not position_only):
            wHEEDes[:3, :3] = targetRotation
        #
        while(not ik_stop):
            #
            count +=1
            #
            self.joint_pos_iter, self.joint_vel_iter = self.updateModelFromDq(self.joint_pos_iter, self.joint_vel_iter)
            #
            self.updateQPVariables(wHEEDes, targetJointPosture, self.joint_pos_iter, self.joint_vel_iter, position_only)
            

            # get the QP solution
            qp_sol = solve_qp(P=self.hessian_mx_pgen, q=self.gradient_vec_pgen.T, G=self.cons_mx_state, h=self.cons_vec_state, solver="proxqp")
            self.joint_vel_iter = np.array([qp_sol]).reshape(-1,1)

            if(len(self.joint_vel_iter) < self.robotVar.n_dof):
                self.joint_vel_iter = - np.linalg.inv(self.hessian_mx_pgen) @ self.gradient_vec_pgen
                print(f'- self.joint_vel_iter PINV--- is \t {self.joint_vel_iter.T}')
           
            # stop condition
            if(position_only):
                ik_stop = (count >= self.param_ik.maxIter) or (np.linalg.norm(self.pose_error_iter[:3, :]) < self.param_ik.tolTrans)
            else:
                ik_stop = (count >= self.param_ik.maxIter) or (np.linalg.norm(self.pose_error_iter[:3, :]) < self.param_ik.tolTrans and \
                                            np.linalg.norm(self.pose_error_iter[3:, :]) < self.param_ik.tolOrient)
                
            if(count >= self.param_ik.maxIter):
                print(f'no satisfactory solution found after \t {count} iterations')
                flag = False

        # Extracting the solution after convergence  (final pose and posture)
        self.joint_pos_final = self.joint_pos_iter
        self.wHEE_final = self.robotVar.transformMatrixEE()

        print(f'-- wHEE desired --- --  is \n {wHEEDes}')
        print(f'-- wHEE computed --- --  is \n {self.wHEE_final}')

        return self.joint_pos_final, flag


    def getJointSpacePosture(self):
        return self.joint_pos_final

    def getEndEffectorTransform(self):
        return self.wHEE_final