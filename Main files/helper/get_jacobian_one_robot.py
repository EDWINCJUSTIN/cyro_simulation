import cl_inverse_kinematics as clik
import pybullet as p
import pandas as pd
import pybullet_data
import numpy as np
import time
import com_man_index as mi
import math

physicsClient = p.connect(p.GUI)
#p.setAdditionalSearchPath("C:/Users/EdwinJustin/anaconda3/envs/pin_env/Lib/site-packages/pybullet_data/kuka_iiwa")
p.setAdditionalSearchPath(pybullet_data.getDataPath())
#planeId = p.loadURDF("plane.urdf")
p.setGravity(0,0,0)

def load_robot():
    robotStartPos = [0, 0, 1]
    robotStartOrientation = p.getQuaternionFromEuler([0, math.radians(45), 0])
    """ print(robotStartPos)
    print(np.array(p.getMatrixFromQuaternion(robotStartOrientation)).reshape(3,3)) """
    p.setAdditionalSearchPath("C:/Users/EdwinJustin/CynLr/Design & Dev - Dev/3.User Repos/Edwin/Pedestal configuration/Simulations/V2-09-11-24/urdf_code")
    robotId = p.loadURDF("a0509.urdf", robotStartPos, robotStartOrientation)
    #euler_angles = input("Euler angles (degrees): ")
    #euler_angles = [float(angle) for angle in euler_angles.strip("[]").split(",")]
    #base_orientation_quarternion = p.getQuaternionFromEuler(euler_angles)
    #p.resetBasePositionAndOrientation(robotId, [0,0,0],base_orientation_quarternion )
    return robotId, robotStartPos, robotStartOrientation

def create_target():
    target_position = [0, 0.3, 0.4]
    target_visual = p.createVisualShape(p.GEOM_SPHERE, radius = 0.02, rgbaColor = [1,0,0,1])
    target_body = p.createMultiBody(baseMass=0, baseVisualShapeIndex=target_visual, basePosition=target_position)
    return target_position

def reach_target(ikin, robotId, target):
    #wHEEDes = ikin.robotVar.transformMatrixEE()
    joint_pos =0.5 * (ikin.robotVar.q_min + ikin.robotVar.q_max).reshape(-1,1)
    joint_pos_0 = 0.5 * (ikin.robotVar.q_min + ikin.robotVar.q_max)
    joint_positions = ikin.computeInverseKinematics(  targetPosition = np.array(target).reshape(3, 1), 
										            targetJointPosture = joint_pos_0, 
										            jointPosition = joint_pos)
    print("target position ", np.array(target).reshape(3, 1))
    print("joint_pos_0", joint_pos_0)
    print("joint_pos", joint_pos)
    print("joint positions", np.degrees(joint_positions))
    """ for i in range (2000):
        for joint_index in range(len(joint_positions)):
            p.setJointMotorControl2(robotId, joint_index, p.POSITION_CONTROL, joint_positions[joint_index])
        p.stepSimulation()
        time.sleep(1/240) """
    for i in range (2000):
        for joint_index in range(len(joint_positions)):
            p.resetJointState(robotId, jointIndex = joint_index, targetValue = joint_positions[joint_index])
        p.stepSimulation()
        time.sleep(1/240)
    tcp_position, tcp_orientation = p.getLinkState(robotId, 6)[4:6]
    print(tcp_position)
    print(p.getEulerFromQuaternion(tcp_orientation))

def get_jacobian(ikin):
    Jacobian_tcp = ikin.robotVar.jacobian_ee()
    print(f'----- Jacobian_tcp -----  is \n {Jacobian_tcp}')
    return Jacobian_tcp


if __name__ == "__main__":
    np.set_printoptions(precision=6, suppress=True)
    robotId, robotStartPos, robotStartOrientation = load_robot()
    urdf_filename = 'C:/Users/EdwinJustin/CynLr/Design & Dev - Dev/3.User Repos/Edwin/Pedestal configuration/Simulations/V2-09-11-24/urdf_code/m0609.urdf'
    frame_name = "link6"
    joint_id = 6
    n_dof = 6
    world_H_base = np.eye(4,4)
    world_H_base[:3,:3] = np.array(p.getMatrixFromQuaternion(robotStartOrientation)).reshape(3,3)
    world_H_base[:3,3] = np.array(robotStartPos).T
    print(world_H_base)
    offset_ee = np.array([0.0, 0.0, 0.0]).reshape(-1,1)
    ikin = clik.InvKinCL(urdf_filename, joint_id, frame_name, n_dof, world_H_base, offset_ee)
    target = create_target()
    reach_target(ikin, robotId, target)
    jacobian = get_jacobian(ikin)
    manIndex = mi.find_manIndex(jacobian)
    print(manIndex)
    p.disconnect()