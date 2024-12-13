import pybullet as p
import pybullet_data
import pandas as pd
import numpy as np
import trimesh
import math
import time

physicsClient = p.connect(p.GUI)
p.setGravity(0,0,0)

def load_robot(robotStartPos, robotStartOrientation):
    p.setAdditionalSearchPath("C:/Users/EdwinJustin/CynLr/Design & Dev - Dev/3.User Repos/Edwin/Pedestal configuration/Simulations/V2-09-11-24/urdf_code")
    robotId = p.loadURDF("m0609.urdf", robotStartPos, robotStartOrientation)
    return robotId

if __name__ == "__main__":
    np.set_printoptions(precision=6, suppress=True)

    robotStartPosleft = [-0.285,0,0]
    robotStartOrientationleft = p.getQuaternionFromEuler([0, 0, 0])
    robotStartPosright = [0.285,0,0]
    robotStartOrientationright = p.getQuaternionFromEuler([0, 0, 0])
    robotIdleft = load_robot(robotStartPosleft, robotStartOrientationleft)
    robotIdright = load_robot(robotStartPosright, robotStartOrientationright)
    for x in range(285,451,50):
        position_left0, orientation_left0 = p.getBasePositionAndOrientation(robotIdleft)
        position_right0, orientation_right0 = p.getBasePositionAndOrientation(robotIdright)
        for theta in range(0,91,10):
            print("theta ; ", theta)
            position_left1, orientation_left1 = p.getBasePositionAndOrientation(robotIdleft)
            position_right1, orientation_right1 = p.getBasePositionAndOrientation(robotIdright)
            for phi in range(0,90,10):
                position_left2, orientation_left2 = p.getBasePositionAndOrientation(robotIdleft)
                position_right2, orientation_right2 = p.getBasePositionAndOrientation(robotIdright)
                a, b, c = p.getEulerFromQuaternion(orientation_left2)
                new_orientation_left2 = p.getQuaternionFromEuler([a +math.radians(-10), b, c ])
                a, b, c = p.getEulerFromQuaternion(orientation_right2)
                new_orientation_right2 = p.getQuaternionFromEuler([a +math.radians(-10), b, c ])
                p.resetBasePositionAndOrientation(robotIdleft, position_left2, new_orientation_left2)
                p.resetBasePositionAndOrientation(robotIdright, position_right2, new_orientation_right2)
                """ print("left robot : ", position_left2, p.getEulerFromQuaternion(orientation_left2))
                print("right robot", position_right2, p.getEulerFromQuaternion(orientation_right2)) """
                for i in range(10):
                    p.stepSimulation
                    time.sleep(1/240)
            p.resetBasePositionAndOrientation(robotIdleft, position_left1, orientation_left1)
            p.resetBasePositionAndOrientation(robotIdright, position_right1, orientation_right1)
            a, b, c = p.getEulerFromQuaternion(orientation_left1)
            new_orientation_left1 = p.getQuaternionFromEuler([a, b+math.radians(-10), c])
            a, b, c = p.getEulerFromQuaternion(orientation_right1)
            new_orientation_right1 = p.getQuaternionFromEuler([a, b+math.radians(10), c])
            p.resetBasePositionAndOrientation(robotIdleft, position_left1, new_orientation_left1)
            p.resetBasePositionAndOrientation(robotIdright, position_right1, new_orientation_right1)
            """ print("left robot : ", position_left1, p.getEulerFromQuaternion(orientation_left1))
            print("right robot", position_right1, p.getEulerFromQuaternion(orientation_right1)) """
        p.resetBasePositionAndOrientation(robotIdleft, position_left0, orientation_left0)
        p.resetBasePositionAndOrientation(robotIdright, position_right0, orientation_right0)
        new_position_left = [position_left0[0] - 0.05, position_left0[1], position_left0[2]]
        new_position_right = [position_right0[0] + 0.05, position_right0[1], position_right0[2]]
        print("left pos : ", new_position_left)
        print("right pos : ", new_position_right)
        p.resetBasePositionAndOrientation(robotIdleft, new_position_left, orientation_left0)
        p.resetBasePositionAndOrientation(robotIdright, new_position_right, orientation_right0)
        """  print("left robot : ", new_position_left, p.getEulerFromQuaternion(orientation_left0))
        print("right robot", new_position_right, p.getEulerFromQuaternion(orientation_right0)) """