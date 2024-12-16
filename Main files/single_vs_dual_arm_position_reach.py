import pybullet as p
import pybullet_data
import math
import numpy as np
import pandas as pd
import time
import helper.cl_inverse_kinematics as clik                 #inports cynlr inverse kinematics code from folder helper

physicsClient = p.connect(p.GUI)                            #starts pybullet client
p.setGravity(0,0,0)                                         #no gravity

def load_robot(robotStartPos, robotStartOrientation):       #loads robot at the position and orientation mentioned
    p.setAdditionalSearchPath("C:/Users/EdwinJustin/CynLr/Design & Dev - Dev/3.User Repos/Edwin/Pedestal configuration/Simulations/V2-09-11-24/urdf_code")       #source file location of robot. change this path to load robot of your choice
    robotId = p.loadURDF("m0609.urdf", robotStartPos, robotStartOrientation)
    return robotId

def create_grid(x_range, y_range, z_range, grid_space):             #creates a grid of target points with reference to global origin.
    x = np.arange(x_range[0], x_range[1], grid_space)
    y = np.arange(y_range[0], y_range[1], grid_space)
    z = np.arange(z_range[0], z_range[1], grid_space)
    #print(x, y, z)
    grid = np.array(np.meshgrid(x, y, z)).T.reshape(-1, 3)          #returns grid coordinates in the form of a dictionary.
    grid_dict = {
        f"point_{px}_{py}_{pz}": (px, py, pz)
        for px, py, pz in grid
    }
    return grid_dict

def visual_grid(grid_dict):                                         #creates small red spheres to help visualize the grid
    for point in grid_dict.values():
        print(point)
        target_position = point
        target_visual = p.createVisualShape(p.GEOM_SPHERE, radius = 0.01, rgbaColor = [1,0,0,1])
        target_body = p.createMultiBody(baseMass=0, baseVisualShapeIndex=target_visual, basePosition=target_position)

def reach_target(ikin, robotId, target):                            #function to calculate inverse kinematics and move robot joints to required orientation.
    joint_pos =0.5 * (ikin.robotVar.q_min + ikin.robotVar.q_max).reshape(-1,1)
    joint_pos_0 = 0.5 * (ikin.robotVar.q_min + ikin.robotVar.q_max)
    joint_positions, flag = ikin.computeInverseKinematics(  targetPosition = np.array(target).reshape(3, 1),            #flag is true if ik is succesful. false if failed
										            targetJointPosture = joint_pos_0, 
										            jointPosition = joint_pos)

    for i in range (2):                                             #change range value to change simulation time
        for joint_index in range(1,7):
            p.resetJointState(robotId, jointIndex = joint_index, targetValue = joint_positions[joint_index-1])          #moves robot joints to required angles
        p.stepSimulation()
        time.sleep(1/240)

    if flag:
        print(f"target_{target}_reachable")                         #if the target is reachable, return true else false
        return True
    else:
        return False
    
if __name__ == "__main__":                                          #main function
    np.set_printoptions(precision=6, suppress=True)                 #sets precision of floating points

    robotStartPosleft = [-0.285,0,0]                                 #initial position and orientation of both robots
    robotStartOrientationleft = p.getQuaternionFromEuler([0, 0, 0])
    robotStartPosright = [0.285,0,0]
    robotStartOrientationright = p.getQuaternionFromEuler([0, 0, 0])
    robotIdleft = load_robot(robotStartPosleft, robotStartOrientationleft)              #loads robots
    robotIdright = load_robot(robotStartPosright, robotStartOrientationright)

    num_joints_left = p.getNumJoints(robotIdleft)                   #gets number of joints of each robots
    num_joints_right = p.getNumJoints(robotIdright)
    for link_a in range(0, num_joints_left):  
        for link_b in range(0, num_joints_right):  
            p.setCollisionFilterPair(robotIdleft, robotIdright, link_a, link_b, enableCollision=False)              #disables collision detection between robots

    grid_dict = create_grid([-0.5,0.5], [0,1], [-0.865,1.135], 0.1)             #creates target grid. change the parameters send to change the x,y and z limits of the grid. 0.2m is the current space 
    visual_grid(grid_dict)                                                      #between grid points. this can be changed as requird

    data = {'Left_robot' : [], 'Right_robot' : [], 'Both_robots' : []}          #array to store number of points reachable for left,right and both robots
    orientation = []                                                            #array to store orientations of robot base, since both robots are moved symmetrically, only one array is required

    for x in range(285,451,50):
        position_left0, orientation_left0 = p.getBasePositionAndOrientation(robotIdleft)                #loop to vary distance between robot bases. currently it goes from 285 to 451mm from origin in steps of 50mm
        position_right0, orientation_right0 = p.getBasePositionAndOrientation(robotIdright)
        for theta in range(0,91,45):                                                                    #loop to rotate robot base about y axis from 0 to 90degrees in steps of 45
            position_left1, orientation_left1 = p.getBasePositionAndOrientation(robotIdleft)
            position_right1, orientation_right1 = p.getBasePositionAndOrientation(robotIdright)
            for phi in range(0,90,45):                                                                  #loop to rotate robot base about x axis from 0 to 90 degrees in steps of 45
                position_left2, orientation_left2 = p.getBasePositionAndOrientation(robotIdleft)
                position_right2, orientation_right2 = p.getBasePositionAndOrientation(robotIdright)
                for z_angle in range(0,91,45):                                                          #loop to rotate robot base about z axis from 0 to 90 degrees in steps of 45
                    position_left3, orientation_left3 = p.getBasePositionAndOrientation(robotIdleft)
                    position_right3, orientation_right3 = p.getBasePositionAndOrientation(robotIdright)

                    urdf_filename = 'C:/Users/EdwinJustin/CynLr/Design & Dev - Dev/3.User Repos/Edwin/Pedestal configuration/Simulations/V2-09-11-24/urdf_code/m0609.urdf'          #change file path for different robot
                    frame_name = "link6"
                    joint_id = 6
                    n_dof = 6
                    world_H_baseleft = np.eye(4,4)
                    world_H_baseleft[:3,:3] = np.array(p.getMatrixFromQuaternion(orientation_left3)).reshape(3,3)
                    world_H_baseleft[:3,3] = np.array(position_left3).T                                                     #set the base of the robot for calculating ik
                    world_H_baseright = np.eye(4,4)
                    world_H_baseright[:3,:3] = np.array(p.getMatrixFromQuaternion(orientation_right3)).reshape(3,3)
                    world_H_baseright[:3,3] = np.array(position_right3).T
                    offset_ee = np.array([0.0, 0.0, 0.0]).reshape(-1,1)                                                     #tcp offset. set according to end effector used
                    ikinleft = clik.InvKinCL(urdf_filename, joint_id, frame_name, n_dof, world_H_baseleft, offset_ee)
                    ikinright = clik.InvKinCL(urdf_filename, joint_id, frame_name, n_dof, world_H_baseright, offset_ee)

                    left_robot_count = 0                                                                #keeps count of number of target poins reachable by left robot
                    right_robot_count = 0                                                               #right robot
                    both_robot_count = 0                                                                #both robots
                    for target_key, target in grid_dict.items():                                        #loops through all target points
                        left_robot = reach_target(ikinleft, robotIdleft, target)                        #checks reach of left robot
                        right_robot = reach_target(ikinright, robotIdright, target)                     #checks reach of right robot
                        if left_robot:
                            left_robot_count +=1                                                        #increments number of points reachable by left robot
                        if right_robot:
                            right_robot_count +=1                                                       #increments number of points reachable by right robot
                        if left_robot and right_robot:
                            both_robot_count +=1                                                        #increments number of points reachable by both robots
                    data["Left_robot"].append(left_robot_count)                                         #adds count to array
                    data["Right_robot"].append(right_robot_count)
                    data["Both_robots"].append(both_robot_count)
                    orientation.append(f"x : {x}, theta : {theta}, phi : {phi}, z_axiz_rot : {z_angle}")    #stores the robot base orientation in an array
                    a, b, c = p.getEulerFromQuaternion(orientation_left3)
                    new_orientation_left3 = p.getQuaternionFromEuler([a , b, c+math.radians(-45) ])         #changes robot base orientation about z axis
                    a, b, c = p.getEulerFromQuaternion(orientation_right3)
                    new_orientation_right3 = p.getQuaternionFromEuler([a , b, c +math.radians(45)])
                    p.resetBasePositionAndOrientation(robotIdleft, position_left3, new_orientation_left3)   #updates new orientation
                    p.resetBasePositionAndOrientation(robotIdright, position_right3, new_orientation_right3)
                    for i in range(3):
                        p.stepSimulation                                                                    #simulates the change in base orientation
                        time.sleep(1/240)
                
                p.resetBasePositionAndOrientation(robotIdleft, position_left2, orientation_left2)           #resets base to before rotation about z axis
                p.resetBasePositionAndOrientation(robotIdright, position_right2, orientation_right2)
                a, b, c = p.getEulerFromQuaternion(orientation_left2)
                new_orientation_left2 = p.getQuaternionFromEuler([a +math.radians(-45), b, c ])             #rotates robot base about x axis
                a, b, c = p.getEulerFromQuaternion(orientation_right2)
                new_orientation_right2 = p.getQuaternionFromEuler([a +math.radians(-45), b, c ])
                p.resetBasePositionAndOrientation(robotIdleft, position_left2, new_orientation_left2)       #updates new orientation
                p.resetBasePositionAndOrientation(robotIdright, position_right2, new_orientation_right2)
                
            p.resetBasePositionAndOrientation(robotIdleft, position_left1, orientation_left1)               #resets base to befor rotation about x axis
            p.resetBasePositionAndOrientation(robotIdright, position_right1, orientation_right1)
            a, b, c = p.getEulerFromQuaternion(orientation_left1)
            new_orientation_left1 = p.getQuaternionFromEuler([a, b+math.radians(-45), c])                   #rotates robot base about y axis
            a, b, c = p.getEulerFromQuaternion(orientation_right1)
            new_orientation_right1 = p.getQuaternionFromEuler([a, b+math.radians(45), c])
            p.resetBasePositionAndOrientation(robotIdleft, position_left1, new_orientation_left1)           #updates new orientation
            p.resetBasePositionAndOrientation(robotIdright, position_right1, new_orientation_right1)
        p.resetBasePositionAndOrientation(robotIdleft, position_left0, orientation_left0)                   #resets base to befor rotation about y axis
        p.resetBasePositionAndOrientation(robotIdright, position_right0, orientation_right0)
        new_position_left = [position_left0[0] - 0.05, position_left0[1], position_left0[2]]                #moves robot base in x axis
        new_position_right = [position_right0[0] + 0.05, position_right0[1], position_right0[2]]
        print("left pos : ", new_position_left)
        print("right pos : ", new_position_right)
        p.resetBasePositionAndOrientation(robotIdleft, new_position_left, orientation_left0)                #updates new orientation
        p.resetBasePositionAndOrientation(robotIdright, new_position_right, orientation_right0)

    excel_file_path = "C:/Users/EdwinJustin/CynLr/Design & Dev - Dev/3.User Repos/Edwin/Pedestal configuration/Simulations/V2-09-11-24/Trial exports/single_vs_dual_arm_reach.xlsx"                  #file path to export excel file
    df = pd.DataFrame(data, index=orientation)                                                              #converts array to dataframe
    df.to_excel(excel_file_path)                                                                            #writes data into excel file

    p.disconnect()                                                                                          #pybullet disconnects 

