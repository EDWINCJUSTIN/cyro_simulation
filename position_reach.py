import pybullet as p
import pybullet_data
import math
import numpy as np
import pandas as pd
import time
import cl_inverse_kinematics as clik
import trimesh
import rtree

physicsClient = p.connect(p.GUI)
p.setGravity(0,0,0)

def get_jacobian(ikin):
    Jacobian_tcp = ikin.robotVar.jacobian_ee()
    print(f'----- Jacobian_tcp -----  is \n {Jacobian_tcp}')
    return Jacobian_tcp

def find_manIndex(jacobian):
    print("jacobian : ", jacobian)
    jac_transpose = jacobian.T
    print("jacobian transpose", jac_transpose)
    determinant = np.linalg.det(jacobian*jac_transpose)
    print("determinant ; ", determinant)
    manIndex = math.sqrt(abs(determinant))
    print("man index : ", manIndex)
    return manIndex

def load_robot(robotStartPos, robotStartOrientation):
    p.setAdditionalSearchPath("C:/Users/EdwinJustin/CynLr/Design & Dev - Dev/3.User Repos/Edwin/Pedestal configuration/Simulations/V2-09-11-24/urdf_code")
    robotId = p.loadURDF("m0609.urdf", robotStartPos, robotStartOrientation)
    return robotId

def create_grid(x_range, y_range, z_range, grid_space):
    x = np.arange(x_range[0], x_range[1], grid_space)
    y = np.arange(y_range[0], y_range[1], grid_space)
    z = np.arange(z_range[0], z_range[1], grid_space)
    print(x, y, z)
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
    print("target position ", np.array(target).reshape(3, 1))
    print("joint positions", np.degrees(joint_positions))
    for i in range (20):
        for joint_index in range(1,7):
            p.resetJointState(robotId, jointIndex = joint_index, targetValue = joint_positions[joint_index-1])
        p.stepSimulation()
        time.sleep(1/240)
    tcp_position, tcp_orientation = p.getLinkState(robotId, 6)[4:6]
    print(tcp_position)
    print(p.getEulerFromQuaternion(tcp_orientation))
    offset = math.sqrt((tcp_position[0]-target[0])**2 + (tcp_position[1]-target[1])**2 + (tcp_position[2]-target[2])**2)
    print("offset of tcp from target : ", offset)
    if offset < 0.0002:
        print(f"target_{target}_reachable")
        jacobian = get_jacobian(ikin)
        manIndex = find_manIndex(jacobian)
        return True, manIndex, f"point_{target[0]}_{target[1]}_{target[2]}"
    else:
        manIndex = None
        return False, manIndex, f"point_{target[0]}_{target[1]}_{target[2]}"
    
def create_spheres(pose):
    sphere = trimesh.creation.icosphere(radius=900, subdivisions=5)
    sphere.apply_translation(pose)
    return sphere

def calc_scaled_avg_manIndex(dictionary):
    keys_to_delete = []
    for key_name in dictionary:
        if dictionary[key_name] == None:
            keys_to_delete.append(key_name)
            #del dictionary[key_name]
    for key in keys_to_delete:
        del dictionary[key]
    sum_dict = sum(dictionary.values())
    print("sum of man index : ", sum_dict)
    max_dict = max(dictionary.values())
    print("max of man index : ", max_dict)
    len_dict = len(dictionary)
    print("length of man index : ", len_dict)
    scaled_avg_manIndex = sum_dict/(max_dict*len_dict)
    return scaled_avg_manIndex

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
    left_sphere_pose = np.array(robotStartPosleft) + np.array([0,0,0.135])
    right_sphere_pose = np.array(robotStartPosright) + np.array([0,0,0.135])
    #left_sphere_pose.reshape(3,)
    #right_sphere_pose.reshape(3,)
    left_sphere = create_spheres(left_sphere_pose)
    right_sphere = create_spheres(right_sphere_pose)
    shared_space = left_sphere.intersection(right_sphere)
    common_target=[]
    for target_key, target in grid_dict.items():
        point = np.array(target)
        if shared_space.contains([target]):
            common_target.append(target_key)
            
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

    rows = []
    for target_key in grid_dict:
        rows.append(target_key)
    column = ['left_robot', 'right_robot']
    array = pd.DataFrame(index = rows, columns= column)

    man_index_left_dict = {}
    man_index_right_dict = {}

    for target_key, target in grid_dict.items():
        left_robot, man_index_left, key_name_left = reach_target(ikinleft, robotIdleft, target)
        man_index_left_dict[key_name_left] = man_index_left
        right_robot, man_index_right, key_name_right = reach_target(ikinright, robotIdright, target)
        man_index_right_dict[key_name_right] = man_index_right
        if left_robot:
            array.loc[target_key, 'left_robot'] = "reachable"
        else:
            array.loc[target_key, 'left_robot'] = "unreachable"
        if right_robot:
            array.loc[target_key, 'right_robot'] = "reachable"
        else:
            array.loc[target_key, 'right_robot'] = "unreachable"

    excel_file_path = "C:/Users/EdwinJustin/CynLr/Design & Dev - Dev/3.User Repos/Edwin/Pedestal configuration/Simulations/V2-09-11-24/Trial exports/position_reach.xlsx"
    with pd.ExcelWriter(excel_file_path, engine='openpyxl') as writer:
        array.to_excel(writer, sheet_name= "trial", index=True)

    common_man_index_left_dict = {}
    common_man_index_right_dict = {}

    for target_key in grid_dict:
        if target_key in common_target:
            common_man_index_left_dict[target_key] = man_index_left_dict[target_key]
            common_man_index_right_dict[target_key] = man_index_right_dict[target_key]

    scaled_avg_man_index_left = calc_scaled_avg_manIndex(common_man_index_left_dict)
    print("scaled avg man index left : ", scaled_avg_man_index_left)
    scaled_avg_man_index_right = calc_scaled_avg_manIndex(common_man_index_right_dict)
    print("scaled avg man index right : ", scaled_avg_man_index_right)

    intersection_avg_dual_arm_man_index = (scaled_avg_man_index_left + scaled_avg_man_index_right)/2
    print("intersection avg dual arm man index :", intersection_avg_dual_arm_man_index)

    shared_vol = shared_space.volume
    combined_vol = left_sphere.volume + right_sphere.volume - shared_vol
    print("shared vol : ", shared_vol)
    print("combined vol : ", combined_vol)

    dual_arm_configuration_man_index = (combined_vol/shared_vol) * intersection_avg_dual_arm_man_index
    print("dual arm configuration man index : ", dual_arm_configuration_man_index)

    p.disconnect()
        

