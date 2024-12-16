import pybullet as p
import pybullet_data
import math
import numpy as np
import pandas as pd
import time
import helper.cl_inverse_kinematics as clik
import trimesh

physicsClient = p.connect(p.GUI)
p.setGravity(0,0,0)

def load_robot(robotStartPos, robotStartOrientation):
    p.setAdditionalSearchPath("C:/Users/EdwinJustin/CynLr/Design & Dev - Dev/3.User Repos/Edwin/Pedestal configuration/Simulations/V2-09-11-24/urdf_code")
    robotId = p.loadURDF("m0609.urdf", robotStartPos, robotStartOrientation)
    return robotId

def create_sphere(pose):        #function to create sphere
    sphere = trimesh.creation.icosphere(radius=900, subdivisions=8)         #set the radius of the sphere here. for doosan M0609, it is taken as 900mm, equal to its reach
    sphere.apply_translation(pose)                                          #increase the subdivisions to get more accurate results. However, this significantly increases computation time. 
    return sphere

if __name__ == "__main__":
    np.set_printoptions(precision=6, suppress=True)

    robotStartPosleft = [-0.285,0,0]
    robotStartOrientationleft = p.getQuaternionFromEuler([0, 0, 0])
    robotStartPosright = [0.285,0,0]
    robotStartOrientationright = p.getQuaternionFromEuler([0, 0, 0])
    robotIdleft = load_robot(robotStartPosleft, robotStartOrientationleft)
    robotIdright = load_robot(robotStartPosright, robotStartOrientationright)

    num_joints_left = p.getNumJoints(robotIdleft)
    num_joints_right = p.getNumJoints(robotIdright)
    for link_a in range(0, num_joints_left):  
        for link_b in range(0, num_joints_right):  
            p.setCollisionFilterPair(robotIdleft, robotIdright, link_a, link_b, enableCollision=False)
    
    for x in range(285,451,50):
        position_left0, orientation_left0 = p.getBasePositionAndOrientation(robotIdleft)
        position_right0, orientation_right0 = p.getBasePositionAndOrientation(robotIdright)

        for theta in range(0,91,45):
            position_left1, orientation_left1 = p.getBasePositionAndOrientation(robotIdleft)
            position_right1, orientation_right1 = p.getBasePositionAndOrientation(robotIdright)

            for phi in range(0,90,45):
                position_left2, orientation_left2 = p.getBasePositionAndOrientation(robotIdleft)
                position_right2, orientation_right2 = p.getBasePositionAndOrientation(robotIdright)
                
                for z_angle in range(0,91,45):
                    position_left3, orientation_left3 = p.getBasePositionAndOrientation(robotIdleft)
                    position_right3, orientation_right3 = p.getBasePositionAndOrientation(robotIdright)

                    urdf_filename = 'C:/Users/EdwinJustin/CynLr/Design & Dev - Dev/3.User Repos/Edwin/Pedestal configuration/Simulations/V2-09-11-24/urdf_code/m0609.urdf'
                    frame_name = "link6"
                    joint_id = 6
                    n_dof = 6
                    world_H_baseleft = np.eye(4,4)
                    world_H_baseleft[:3,:3] = np.array(p.getMatrixFromQuaternion(orientation_left2)).reshape(3,3)
                    world_H_baseleft[:3,3] = np.array(position_left2).T
                    world_H_baseright = np.eye(4,4)
                    world_H_baseright[:3,:3] = np.array(p.getMatrixFromQuaternion(orientation_right2)).reshape(3,3)
                    world_H_baseright[:3,3] = np.array(position_right2).T
                    offset_ee = np.array([0.0, 0.0, 0.0]).reshape(-1,1)
                    ikinleft = clik.InvKinCL(urdf_filename, joint_id, frame_name, n_dof, world_H_baseleft, offset_ee)
                    ikinright = clik.InvKinCL(urdf_filename, joint_id, frame_name, n_dof, world_H_baseright, offset_ee)

                    data["Left_robot"].append(left_robot_count)
                    data["Right_robot"].append(right_robot_count)
                    data["Both_robots"].append(both_robot_count)
                    orientation.append(f"x : {x}, theta : {theta}, phi : {phi}, z_axiz_rot : {z_angle}")
                    a, b, c = p.getEulerFromQuaternion(orientation_left3)
                    new_orientation_left3 = p.getQuaternionFromEuler([a , b, c+math.radians(-45) ])
                    a, b, c = p.getEulerFromQuaternion(orientation_right3)
                    new_orientation_right3 = p.getQuaternionFromEuler([a , b, c +math.radians(45)])
                    p.resetBasePositionAndOrientation(robotIdleft, position_left3, new_orientation_left3)
                    p.resetBasePositionAndOrientation(robotIdright, position_right3, new_orientation_right3)
                    for i in range(3):
                        p.stepSimulation
                        time.sleep(1/240)
                
                p.resetBasePositionAndOrientation(robotIdleft, position_left2, orientation_left2)
                p.resetBasePositionAndOrientation(robotIdright, position_right2, orientation_right2)
                a, b, c = p.getEulerFromQuaternion(orientation_left2)
                new_orientation_left2 = p.getQuaternionFromEuler([a +math.radians(-45), b, c ])
                a, b, c = p.getEulerFromQuaternion(orientation_right2)
                new_orientation_right2 = p.getQuaternionFromEuler([a +math.radians(-45), b, c ])
                p.resetBasePositionAndOrientation(robotIdleft, position_left2, new_orientation_left2)
                p.resetBasePositionAndOrientation(robotIdright, position_right2, new_orientation_right2)
                
            p.resetBasePositionAndOrientation(robotIdleft, position_left1, orientation_left1)
            p.resetBasePositionAndOrientation(robotIdright, position_right1, orientation_right1)
            a, b, c = p.getEulerFromQuaternion(orientation_left1)
            new_orientation_left1 = p.getQuaternionFromEuler([a, b+math.radians(-45), c])
            a, b, c = p.getEulerFromQuaternion(orientation_right1)
            new_orientation_right1 = p.getQuaternionFromEuler([a, b+math.radians(45), c])
            p.resetBasePositionAndOrientation(robotIdleft, position_left1, new_orientation_left1)
            p.resetBasePositionAndOrientation(robotIdright, position_right1, new_orientation_right1)
        p.resetBasePositionAndOrientation(robotIdleft, position_left0, orientation_left0)
        p.resetBasePositionAndOrientation(robotIdright, position_right0, orientation_right0)
        new_position_left = [position_left0[0] - 0.05, position_left0[1], position_left0[2]]
        new_position_right = [position_right0[0] + 0.05, position_right0[1], position_right0[2]]
        print("left pos : ", new_position_left)
        print("right pos : ", new_position_right)
        p.resetBasePositionAndOrientation(robotIdleft, new_position_left, orientation_left0)
        p.resetBasePositionAndOrientation(robotIdright, new_position_right, orientation_right0)