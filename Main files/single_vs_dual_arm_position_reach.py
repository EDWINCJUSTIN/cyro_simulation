import pybullet as p
import pybullet_data
import math
import numpy as np
import pandas as pd
import time
import cl_inverse_kinematics as clik

physicsClient = p.connect(p.GUI)
p.setGravity(0,0,0)

def load_robot(robotStartPos, robotStartOrientation):
    p.setAdditionalSearchPath("C:/Users/EdwinJustin/CynLr/Design & Dev - Dev/3.User Repos/Edwin/Pedestal configuration/Simulations/V2-09-11-24/urdf_code")
    robotId = p.loadURDF("m0609.urdf", robotStartPos, robotStartOrientation)
    return robotId

def create_grid(x_range, y_range, z_range, grid_space):
    x = np.arange(x_range[0], x_range[1], grid_space)
    y = np.arange(y_range[0], y_range[1], grid_space)
    z = np.arange(z_range[0], z_range[1], grid_space)
    #print(x, y, z)
    grid = np.array(np.meshgrid(x, y, z)).T.reshape(-1, 3)
    grid_dict = {
        f"point_{px}_{py}_{pz}": (px, py, pz)
        for px, py, pz in grid
    }
    return grid_dict

def visual_grid(grid_dict):
    for point in grid_dict.values():
        print(point)
        target_position = point
        target_visual = p.createVisualShape(p.GEOM_SPHERE, radius = 0.01, rgbaColor = [1,0,0,1])
        target_body = p.createMultiBody(baseMass=0, baseVisualShapeIndex=target_visual, basePosition=target_position)

def reach_target(ikin, robotId, target):
    joint_pos =0.5 * (ikin.robotVar.q_min + ikin.robotVar.q_max).reshape(-1,1)
    joint_pos_0 = 0.5 * (ikin.robotVar.q_min + ikin.robotVar.q_max)
    joint_positions = ikin.computeInverseKinematics(  targetPosition = np.array(target).reshape(3, 1), 
										            targetJointPosture = joint_pos_0, 
										            jointPosition = joint_pos)
    """ print("target position ", np.array(target).reshape(3, 1))
    print("joint positions", np.degrees(joint_positions)) """
    for i in range (2):
        for joint_index in range(1,7):
            p.resetJointState(robotId, jointIndex = joint_index, targetValue = joint_positions[joint_index-1])
        p.stepSimulation()
        time.sleep(1/240)
    tcp_position, tcp_orientation = p.getLinkState(robotId, 6)[4:6]
    """ print(tcp_position)
    print(p.getEulerFromQuaternion(tcp_orientation)) """
    offset = math.sqrt((tcp_position[0]-target[0])**2 + (tcp_position[1]-target[1])**2 + (tcp_position[2]-target[2])**2)
    print("offset of tcp from target : ", offset)
    if offset < 0.0002:
        print(f"target_{target}_reachable")
        return True
    else:
        return False
    
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

    grid_dict = create_grid([-0.5,0.5], [0,1], [-0.865,1.135], 0.1)
    visual_grid(grid_dict)

    urdf_filename = 'C:/Users/EdwinJustin/CynLr/Design & Dev - Dev/3.User Repos/Edwin/Pedestal configuration/Simulations/V2-09-11-24/urdf_code/m0609.urdf'
    frame_name = "link6"
    joint_id = 6
    n_dof = 6
    world_H_baseleft = np.eye(4,4)
    world_H_baseleft[:3,:3] = np.array(p.getMatrixFromQuaternion(robotStartOrientationleft)).reshape(3,3)
    world_H_baseleft[:3,3] = np.array(robotStartPosleft).T
    world_H_baseright = np.eye(4,4)
    world_H_baseright[:3,:3] = np.array(p.getMatrixFromQuaternion(robotStartOrientationright)).reshape(3,3)
    world_H_baseright[:3,3] = np.array(robotStartPosright).T
    offset_ee = np.array([0.0, 0.0, 0.0]).reshape(-1,1)
    ikinleft = clik.InvKinCL(urdf_filename, joint_id, frame_name, n_dof, world_H_baseleft, offset_ee)
    ikinright = clik.InvKinCL(urdf_filename, joint_id, frame_name, n_dof, world_H_baseright, offset_ee)

    data = {'Left_robot' : [], 'Right_robot' : [], 'Both_robots' : []}
    orientation = []

    for x in range(285,451,50):
        position_left0, orientation_left0 = p.getBasePositionAndOrientation(robotIdleft)
        position_right0, orientation_right0 = p.getBasePositionAndOrientation(robotIdright)
        for theta in range(0,91,45):
            print("theta ; ", theta)
            position_left1, orientation_left1 = p.getBasePositionAndOrientation(robotIdleft)
            position_right1, orientation_right1 = p.getBasePositionAndOrientation(robotIdright)
            for phi in range(0,90,45):

                position_left2, orientation_left2 = p.getBasePositionAndOrientation(robotIdleft)
                position_right2, orientation_right2 = p.getBasePositionAndOrientation(robotIdright)

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

                left_robot_count = 0
                right_robot_count = 0
                both_robot_count = 0
                for target_key, target in grid_dict.items():
                    left_robot = reach_target(ikinleft, robotIdleft, target)
                    right_robot = reach_target(ikinright, robotIdright, target)
                    if left_robot:
                        left_robot_count +=1
                    if right_robot:
                        right_robot_count +=1
                    if left_robot and right_robot:
                        both_robot_count +=1
                data["Left_robot"].append(left_robot_count)
                data["Right_robot"].append(right_robot_count)
                data["Both_robots"].append(both_robot_count)
                orientation.append(f"x : {x}, theta : {theta}, phi : {phi}")
                
                a, b, c = p.getEulerFromQuaternion(orientation_left2)
                new_orientation_left2 = p.getQuaternionFromEuler([a +math.radians(-45), b, c ])
                a, b, c = p.getEulerFromQuaternion(orientation_right2)
                new_orientation_right2 = p.getQuaternionFromEuler([a +math.radians(-45), b, c ])
                p.resetBasePositionAndOrientation(robotIdleft, position_left2, new_orientation_left2)
                p.resetBasePositionAndOrientation(robotIdright, position_right2, new_orientation_right2)
                for i in range(3):
                    p.stepSimulation
                    time.sleep(1/240)
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

    excel_file_path = "C:/Users/EdwinJustin/CynLr/Design & Dev - Dev/3.User Repos/Edwin/Pedestal configuration/Simulations/V2-09-11-24/Trial exports/single_vs_dual_arm_reach.xlsx"
    df = pd.DataFrame(data, index=orientation)
    df.to_excel(excel_file_path)

    p.disconnect()
