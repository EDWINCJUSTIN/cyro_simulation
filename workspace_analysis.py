from robodk.robolink import*    
from robodk.robomath import*
import trimesh
import pyvista as pv
import numpy as np
import pandas as pd

RDK = Robolink()        #connect with RoboDK

def load_robots():      #adds two robots into the robodk environment, assigns a seperate base frame for each
    Robot_path = "C:/Users/EdwinJustin/CynLr/Design & Dev - Dev/3.User Repos/Edwin/Pedestal configuration/Simulations/V2-09-11-24/Doosan-Robotics-M0609.robot"      #file path of robot file
    Left_robot = RDK.AddFile(Robot_path)
    Right_robot = RDK.AddFile(Robot_path)
    Left_robot.setName('Left_robot')                #change robot name
    Right_robot.setName('Right_robot')              #change robot name
    if not Left_robot.Valid():                      #checks if robot loaded is valid
        raise Exception(f'Failed to load robot')
    if not Right_robot.Valid():
        raise Exception(f'Failed to load robot')
    Left_robot_frame = RDK.AddFrame('Left_robot_frame')         #Creates a new base frame
    Right_robot_frame = RDK.AddFrame('Right_robot_frame')
    Left_robot.setParent(Left_robot_frame)                      #Assigns the created base frame to each robot
    Right_robot.setParent(Right_robot_frame)
    Frames = RDK.ItemList(ITEM_TYPE_FRAME)
    for frame in Frames:
        if frame.Name() != 'Left_robot_frame' and frame.Name() != 'Right_robot_frame':
            frame.Delete()                                      #Deletes the preloaded frames that comes with the robots
    Origin = RDK.AddFrame('Origin')                             #creates an origin frame. all other frames are referenced to this origin. 
    Origin.setPose(transl(0,0,0))
    Left_robot_frame.setParent(Origin)
    Right_robot_frame.setParent(Origin)
    Left_robot_frame.setPose(transl(0,0,0))                     #sets an initial position for the robots
    Right_robot_frame.setPose(transl(0,0,0))

def define_workspace():         #function to create a reference frame for the workspace volume
    Left_robot_center = RDK.AddFrame('Left_robot_center')
    Right_robot_center = RDK.AddFrame('Right_robot_center')
    Left_robot_center.setParent(RDK.Item('Left_robot_frame', ITEM_TYPE_FRAME))
    Right_robot_center.setParent(RDK.Item('Right_robot_frame', ITEM_TYPE_FRAME))
    Left_robot_center.setPose(transl(0,0,135))              #sets the position of the workspace center with respect to the robot base frame
    Right_robot_center.setPose(transl(0,0,135))             #the current value is taken assuming that the center lies at the center of J1 and J2 axes
    #return Left_robot_center, Right_robot_center

def create_sphere(pose):        #function to create sphere
    sphere = trimesh.creation.icosphere(radius=900, subdivisions=8)         #set the radius of the sphere here. for doosan M0609, it is taken as 900mm, equal to its reach
    sphere.apply_translation(pose)                                          #increase the subdivisions to get more accurate results. However, this significantly increases computation time. 
    return sphere

if __name__ == "__main__":
    load_robots()
    define_workspace()
    initial_pos_left = transl(-285,0,0)                     #initial position from where we start the analysisi. this value is choosen such that J2 does not collide when rotated 360 degree
    initial_pos_right = transl(285,0,0)
    Left_robot_frame = RDK.Item('Left_robot_frame', ITEM_TYPE_FRAME)
    Left_robot_frame.setPose(initial_pos_left)
    Right_robot_frame = RDK.Item('Right_robot_frame', ITEM_TYPE_FRAME)
    Right_robot_frame.setPose(initial_pos_right)
    column = [f"x_pos_{i}" for i in range(285, 451, 10)]            #creates row and column names according to the position and orientation of the robot base
    rows = [f"orientation_{j}" for j in range(0, 91, 10)]
    array1 = pd.DataFrame(index=rows, columns=column)               #creates an array with the row and column names as headers and index
    array2 = pd.DataFrame(index=rows, columns=column)
    for col in array1.columns:
        Left_default_orientation = Left_robot_frame.Pose()
        Right_default_orientation = Right_robot_frame.Pose()
        for row in array1.index:
            Left_sphere_pose = RDK.Item('Left_robot_center', ITEM_TYPE_FRAME).PoseWrt(RDK.Item('Origin', ITEM_TYPE_FRAME))
            x, y, z = Left_sphere_pose[0, 3], Left_sphere_pose[1, 3], Left_sphere_pose[2, 3]
            Left_sphere = create_sphere([x, y, z])
            Left_sphere_volume = Left_sphere.volume
            Right_sphere_pose = RDK.Item('Right_robot_center', ITEM_TYPE_FRAME).PoseWrt(RDK.Item('Origin', ITEM_TYPE_FRAME))
            x, y, z = Right_sphere_pose[0, 3], Right_sphere_pose[1, 3], Right_sphere_pose[2, 3]
            Right_sphere = create_sphere([x, y, z])
            Right_sphere_volume = Right_sphere.volume
            intersection = Left_sphere.intersection(Right_sphere)           #finds the intersection volume
            shared_volume = intersection.volume
            if not intersection.is_empty:
                print(f"Intersection Volume: {intersection.volume:.3f}")
                combined_volume = Left_sphere_volume + Right_sphere_volume - shared_volume          #finds combined volume
                array1.loc[row, col] = intersection.volume                                          #adds data to array
                array2.loc[row, col] = combined_volume
            else:
                print("No intersection.")
            new_left_pose = Left_robot_frame.Pose() * roty(math.radians(-10))               #rotates the robot bases by 10 degrees about the y axis 
            new_right_pose = Right_robot_frame.Pose() * roty(math.radians(10))
            Left_robot_frame.setPose(new_left_pose)
            Right_robot_frame.setPose(new_right_pose)
        Left_robot_frame.setPose(Left_default_orientation)                                  #resets the orientation back to 0 degrees
        Right_robot_frame.setPose(Right_default_orientation)
        new_left_pose = Left_robot_frame.Pose() * transl(-10,0,0)                           #translates the robot bases by 10mm in the x direction
        new_right_pose = Right_robot_frame.Pose() * transl(10,0,0)
        Left_robot_frame.setPose(new_left_pose)
        Right_robot_frame.setPose(new_right_pose)
    excel_file_path = "C:/Users/EdwinJustin/CynLr/Design & Dev - Dev/3.User Repos/Edwin/Pedestal configuration/Simulations/V2-09-11-24/Trial exports/workspace_analysis.xlsx"           #file location of the exported excel file
    with pd.ExcelWriter(excel_file_path, engine='openpyxl') as writer:
        array1.to_excel(writer, sheet_name = "Shared Volume", index = True)
        array2.to_excel(writer, sheet_name = "Combined Volume", index = True)




        

