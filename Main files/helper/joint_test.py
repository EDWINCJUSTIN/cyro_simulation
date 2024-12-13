import position_reach as pos
import pybullet as p
import pybullet_data
import xml.etree.ElementTree as ET

robotStartPos = [-0.285,0,0]
robotStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
robotId = pos.load_robot(robotStartPos, robotStartOrientation)
numJoints = p.getNumJoints(robotId)
print("number of joints : ", numJoints)
for i in range(numJoints) :
    print(p.getJointInfo(robotId, i))
    print("Body %d's name is %s." % (i, p.getJointInfo(robotId, i)[1]))
tree = ET.parse("C:/Users/EdwinJustin/CynLr/Design & Dev - Dev/3.User Repos/Edwin/Pedestal configuration/Simulations/V2-09-11-24/urdf_code/m0609.urdf")
root = tree.getroot()
joint1 = root.find(".//joint[@name='joint1']")
if joint1 is not None:
    origin = joint1.find("origin")
    if origin is not None:
        xyz = origin.attrib.get('xyz', 'Not found')
        print(xyz)
    else:
        print("not found")
else:
    print("joint not found")
