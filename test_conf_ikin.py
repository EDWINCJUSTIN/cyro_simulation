'''
//|    Copyright (C) 2024 Cybernitic Laboratory (CynLr) 
//|    website:  http://www.cynlr.com
//|
'''

import numpy as np
import pinocchio
import time
from sys import argv
from os.path import dirname, join, abspath
from scipy.spatial.transform import Rotation as rot
#import utils
from cl_inverse_kinematics import InvKinCL



# test of the robot variable class
if __name__ =="__main__":

	np.set_printoptions(precision=6, suppress=True)
	urdf_filename = 'C:/Users/EdwinJustin/CynLr/Design & Dev - Dev/3.User Repos/Edwin/Pedestal configuration/Simulations/V2-09-11-24/urdf_code/m0609.urdf'
	frame_name = "link6"
	joint_id = 6
	n_dof = 6
	world_H_base = np.eye(4,4)
	offset_ee = np.array([0.0, 0.0, 0.0]).reshape(-1,1)

	# create ikin object
	ikin = InvKinCL(urdf_filename, joint_id, frame_name, n_dof, world_H_base, offset_ee)
	
	#
	joint_pos =0.5 * (ikin.robotVar.q_min + ikin.robotVar.q_max).reshape(-1,1)
	joint_vel = np.zeros([joint_pos.size]).reshape(-1,1)    # joint_vel model
	joint_pos_0 = 0.5 * (ikin.robotVar.q_min + ikin.robotVar.q_max)

	# define the desired frame
	ratio = np.random.uniform(0.0, 1.0)
	joint_pos_des = np.array(ratio * ikin.robotVar.q_min + (1.0 - ratio) * ikin.robotVar.q_max).reshape(-1,1)
	ikin.updateModelFromDq(joint_pos_des, joint_vel)
	wHEEDes = ikin.robotVar.transformMatrixEE()


	# get the Jacobian Matrix (after a call of  the updateRobotModel method)
	Jacobian_tcp = ikin.robotVar.jacobian_ee()

	print(f'----- Jacobian_tcp -----  is \n {Jacobian_tcp}')


	# compute IK
	position_only = True

	if(position_only):
		ikin.computeInverseKinematics(	targetPosition = wHEEDes[:3, 3:], 
										targetJointPosture = joint_pos_0, 
										jointPosition = joint_pos)
	else:
		joint_pos =0.5 * (ikin.robotVar.q_min + ikin.robotVar.q_max).reshape(-1,1)
		ikin.computeInverseKinematics(	targetPosition = wHEEDes[:3, 3:], 
										targetRotation = wHEEDes[:3, :3], 
										targetJointPosture= joint_pos_0, 
										jointPosition = joint_pos)


