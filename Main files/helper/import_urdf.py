import pybullet as p
import pybullet_data
import time
import os
import pandas as pd

#print(os.getcwd)
#print(pybullet_data.getDataPath())


physicsClient = p.connect(p.GUI)
#p.setAdditionalSearchPath("C:/Users/EdwinJustin/anaconda3/envs/pin_env/Lib/site-packages/pybullet_data/kuka_iiwa")
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")
p.setGravity(0,0,0)

def load_robot():
    robotStartPos = [0, 0, 1]
    robotStartOrientation = p.getQuaternionFromEuler([0, 0, 0])

    p.setAdditionalSearchPath("C:/Users/EdwinJustin/CynLr/Design & Dev - Dev/3.User Repos/Edwin/Pedestal configuration/Simulations/V2-09-11-24/urdf_code")
    robotId = p.loadURDF("m0609.urdf", robotStartPos, robotStartOrientation)
    euler_angles = input("Euler angles (degrees): ")
    euler_angles = [float(angle) for angle in euler_angles.strip("[]").split(",")]
    base_orientation_quarternion = p.getQuaternionFromEuler(euler_angles)
    p.resetBasePositionAndOrientation(robotId, [0,0,0],base_orientation_quarternion )
    return robotId

def create_target():
    target_position = [0, 0.5, 0.4]
    target_visual = p.createVisualShape(p.GEOM_SPHERE, radius = 0.02, rgbaColor = [1,0,0,1])
    target_body = p.createMultiBody(baseMass=0, baseVisualShapeIndex=target_visual, basePosition=target_position)
    return target_position

def reach_target(target, robotId):
    for i in range(1000):
        joint_positions = p.calculateInverseKinematics(robotId, 6, target)
        for joint_index in range(len(joint_positions)):
            p.setJointMotorControl2(robotId, joint_index, p.POSITION_CONTROL, joint_positions[joint_index])
        p.stepSimulation()
        time.sleep(1/240)

def get_jacobian(robotId ):
    joint_states = p.getJointStates(robotId, list(range(6)))
    joint_positions = [state[0] for state in joint_states]
    print("joint states : ", joint_states)
    print("joint positions : ", joint_positions)
    zero_vec = [0.0] * 6
    jac_t, jac_r = p.calculateJacobian(robotId, 6, [0,0,0], joint_positions, zero_vec, zero_vec)
    # Convert the Jacobian matrices to pandas DataFrames for exporting
    df_jac_t = pd.DataFrame(jac_t, columns=[f'Joint {i}' for i in range(len(joint_positions))])
    df_jac_r = pd.DataFrame(jac_r, columns=[f'Joint {i}' for i in range(len(joint_positions))])

    # Export to Excel
    with pd.ExcelWriter('C:/Users/EdwinJustin/CynLr/Design & Dev - Dev/3.User Repos/Edwin/Pedestal configuration/Simulations/V2-09-11-24/Trial exports/jacobian_matrices.xlsx', engine='openpyxl') as writer:
        df_jac_t.to_excel(writer, sheet_name='Linear Jacobian', index=False)
        df_jac_r.to_excel(writer, sheet_name='Angular Jacobian', index=False)
    return jac_t, jac_r

def main():
    robotId=load_robot()
    target = create_target()
    reach_target(target, robotId)
    jac_t, jac_r = get_jacobian(robotId)
    print("linear jacobian : ", jac_t)
    print("angular jacobian : ", jac_r)
    
    '''
    for step in range(5000):
        p.stepSimulation()  # Step the simulation forward
        pos, orn = p.getBasePositionAndOrientation(robotId)  # Get the robot's position and orientation
        print(f"Step {step}: Position {pos}, Orientation {orn}")
        time.sleep(1 / 240)
    '''
    p.disconnect()

main()