import numpy as np
import pybullet as p
import pybullet_data

physicsClient = p.connect(p.GUI)
#p.setAdditionalSearchPath("C:/Users/EdwinJustin/anaconda3/envs/pin_env/Lib/site-packages/pybullet_data/kuka_iiwa")
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")
p.setGravity(0,0,0)

def create_grid(x_range, y_range, z_range, grid_space):
    x = np.arange(x_range[0], x_range[1], grid_space)
    y = np.arange(y_range[0], y_range[1], grid_space)
    z = np.arange(z_range[0], z_range[1], grid_space)
    grid = np.array(np.meshgrid(x, y, z)).T.reshape(-1, 3)
    return grid

def visual_grid(grid):
    for point in grid:
        print(point)
        target_position = point
        target_visual = p.createVisualShape(p.GEOM_SPHERE, radius = 0.2, rgbaColor = [1,0,0,1])
        target_body = p.createMultiBody(baseMass=0, baseVisualShapeIndex=target_visual, basePosition=target_position)

if __name__ == "__main__":
    grid = create_grid([-10,10], [0,10], [0,3], 2)
    print(grid)
    visual_grid(grid)
    orientations = [([0, 0, theta]) for theta in np.linspace(0, 2 * np.pi, 10)]
    print(orientations)
    while True:
        p.stepSimulation()