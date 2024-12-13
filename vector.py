from robodk.robolink import*    
from robodk.robomath import*
import numpy as np

RDK = Robolink()

def version1():
    target_frame = RDK.AddFrame('target_frame', ITEM_TYPE_FRAME)
    target_frame.setPose(transl(0, 50, 20) )  #* rotz(math.radians(45)) * roty(math.radians(45)))
    vector = np.array(target_frame.Pose()).T[:3,3] - np.array([0,0,0])
    """ print(vector)
    print(target_frame.Pose()) """
    tcp_frame = RDK.AddFrame('tcp_frame', ITEM_TYPE_FRAME)
    x_vect = np.array([1,0,0])
    y_vect = np.array([0,1,0])
    z_vect = np.array([0,0,1])
    a = np.arccos(np.clip(np.dot(vector, x_vect)/(np.linalg.norm(vector) * np.linalg.norm(x_vect))))
    b = np.arccos(np.clip(np.dot(vector, y_vect)/(np.linalg.norm(vector) * np.linalg.norm(y_vect))))
    c = np.arccos(np.clip(np.dot(vector, z_vect)/(np.linalg.norm(vector) * np.linalg.norm(z_vect))))
    print(math.degrees(a), math.degrees(b), math.degrees(c))
    print(a,b,c)
    coordinates = np.array(target_frame.Pose()).T[:3,3]
    print(coordinates)
    tcp_frame.setPose(transl(coordinates[0], coordinates[1], coordinates[2]) * rotz(a) * roty(c))
    #tcp_frame.setPose(rotz(a) * roty(c))
    original_pose = tcp_frame.Pose()
    for angle1 in range(30,61,30):
        for angle2 in range(0,360,45):
            frame_name = f'frame_{angle1}_{angle2}'
            new_frame = RDK.AddFrame(frame_name, ITEM_TYPE_FRAME)
            new_frame.setPose(original_pose * rotz(math.radians(angle2)) * rotx(math.radians(angle1)))
    input("enter")
    for frame in RDK.ItemList(ITEM_TYPE_FRAME):
        frame.Delete()

def version2():
    target_frame = RDK.AddFrame('target_frame', ITEM_TYPE_FRAME)
    target_frame.setPose(transl(0, 50, 0))  # Translation only

    # Extract the vector from the frame position
    vector = np.array(target_frame.Pose()).T[:3, 3] - np.array([0, 0, 0])

    # Normalize the vector to get the direction
    vector_norm = vector / np.linalg.norm(vector)

    # Define the rotation matrix to align the tcp_frame with the vector
    # We align the Z-axis of the new frame with the vector direction
    z_axis = vector_norm  # Use the vector as the Z-axis
    x_axis = np.array([1, 0, 0])  # Default X-axis

    # Compute a new Y-axis orthogonal to Z and X
    y_axis = np.cross(z_axis, x_axis)
    y_axis /= np.linalg.norm(y_axis)

    # Recompute an orthogonal X-axis
    x_axis = np.cross(y_axis, z_axis)

    # Create the rotation matrix
    rot_matrix = np.eye(4)
    rot_matrix[:3, :3] = np.array([x_axis, y_axis, z_axis]).T
    print(rot_matrix)
    print(rotz(math.radians(45)))
    
    # Create the new pose
    coordinates = np.array(target_frame.Pose()).T[:3, 3]
    print(transl(coordinates[0], coordinates[1], coordinates[2]))
    tcp_pose = np.array(transl(coordinates[0], coordinates[1], coordinates[2])) * rot_matrix
    print(tcp_pose)

    # Add the TCP frame and set its pose
    tcp_frame = RDK.AddFrame('tcp_frame', ITEM_TYPE_FRAME)
    tcp_frame.setPose(tcp_pose)

    print("TCP Frame Pose:")
    print(tcp_pose)

    # Wait for user input and clean up
    input("Press Enter to clean up...")
    target_frame.Delete()
    tcp_frame.Delete()

if __name__ == "__main__":
    version1()
    #version2()