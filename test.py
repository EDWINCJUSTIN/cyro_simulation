import pybullet as p
import pybullet_data
import math
import numpy as np
import pandas as pd
import time
import cl_inverse_kinematics as clik
""" 
physicsClient = p.connect(p.GUI)
p.setGravity(0,0,0)

def load_robot(robotStartPos, robotStartOrientation):
    p.setAdditionalSearchPath("C:/Users/EdwinJustin/CynLr/Design & Dev - Dev/3.User Repos/Edwin/Pedestal configuration/Simulations/V2-09-11-24/urdf_code")
    robotId = p.loadURDF("m0609.urdf", robotStartPos, robotStartOrientation)
    return robotId

def reach_target(ikin, robotId, target):
    joint_pos =0.5 * (ikin.robotVar.q_min + ikin.robotVar.q_max).reshape(-1,1)
    joint_pos_0 = 0.5 * (ikin.robotVar.q_min + ikin.robotVar.q_max)
    for x in range(0,181,90):
        for y in range(0,361,90):
            print("x", x)
            print("y", y)

            print("before ikin \n\n")
            returned_values, flag = ikin.computeInverseKinematics(  targetPosition = np.array(target).reshape(3, 1),
                                                            targetRotation = np.array(p.getMatrixFromQuaternion(p.getQuaternionFromEuler([x,y,0]))).reshape(3,3), 
                                                            targetJointPosture = joint_pos_0, 
                                                            jointPosition = joint_pos)
            print("Values \n\n", returned_values)
            # return
            print(flag)
            joint_positions = returned_values[:6]
            #flag = returned_values[6]
            print("target position ", np.array(target).reshape(3, 1))
            print("joint positions", np.degrees(joint_positions))
            for i in range (100):
                for joint_index in range(1,7):
                    p.resetJointState(robotId, jointIndex = joint_index, targetValue = joint_positions[joint_index-1])
                p.stepSimulation()
                time.sleep(1/240)
            tcp_position, tcp_orientation = p.getLinkState(robotId, 6)[4:6]
            print(tcp_position)
            print(p.getEulerFromQuaternion(tcp_orientation))
            offset = math.sqrt((tcp_position[0]-target[0])**2 + (tcp_position[1]-target[1])**2 + (tcp_position[2]-target[2])**2)
            print("offset of tcp from target : ", offset)
            if flag:
                print(f"target_{target}_reachable")
                #return True
            else:
                print(f"target_{target}_unreachable")
                #return False
    
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
    target = [0,0.3,0.3]
    left_robot = reach_target(ikinleft, robotIdleft, target)
    right_robot = reach_target(ikinright, robotIdright, target)

    p.disconnect() """
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

if __name__ == "__main__":
    base_orientation = []
    targets = []
    orientations = []
    grid_dict = create_grid([-0.5,0.5], [0,1], [-0.865,1.135], 0.1)
    for x in range(285,451,50):
        for a in range (0,91,45):
            for b in range(0,90,45):
                for c in range(0,361,45):
                    base_orientation.append(f"{x}_{a}_{b}_{c}")
    for target_key, target in grid_dict.items():
        targets.append(target_key)
    for x in range(0,181,30):
        for y in range(0,361,30):
            orientations.append(f"{x}_{y}_0")
    """ print(base_orientation )
    print(targets)
    print(orientations) """
    indices = base_orientation
    headers = ['base_orientations'] + ['targets'] + orientations
    #data = {base_orientation, targets, }
    #print(headers)
    data = {header: [''] * len(indices) for header in headers}
    data['targets'] = targets

    df = pd.DataFrame(data, index= indices)

    print(df)