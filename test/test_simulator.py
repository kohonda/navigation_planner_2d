import matplotlib.animation as animation
from matplotlib import pyplot as plt
import numpy as np
from navigation_simulator_2d.utils import visualizer, ParameterHandler
from navigation_simulator_2d.common import RobotCommand
from navigation_simulator_2d.simulator import Simulator

# Parameters
project_dir = '../'
config_path = '../config/default.yaml'
params = ParameterHandler()
params.init(project_root_dir=project_dir, config=config_path)

# initialize simulator
simulator = Simulator()
simulator.reset(params)

# demo: constant velocity motion
linear_vel = np.array([2.0, 0.0])
angular_vel = 0.5
robot_command = RobotCommand(linear_vel=linear_vel, angular_vel=angular_vel)

save_animation = False
save_folder = '/home/honda/Videos/'

frames = []

# main loop
for _ in range(130):
    # Get robot observation from simulator
    robot_obs = simulator.get_observation()
    
    # Get static map from simulator
    static_map = simulator.get_static_map()
    
    # Update robot state with robot command
    # obstacle_map is 2d occupancy map (0: free, 1: occupied)
    obstacle_map, robot_traj = simulator.step(robot_command)

    # Visualize
    image_arr = visualizer.render(
        static_map=static_map, 
        obstacle_map=obstacle_map, 
        robot_observation=robot_obs,
        robot_traj=robot_traj,
        goal_state=params.goal_state, 
        robot_radius=params.robot_radius,
        global_path=None,
        local_path_list=None)
    
    if save_animation:
        frames.append([plt.imshow(image_arr)])      
    else: 
        # For faster rendering
        plt.imshow(image_arr)
        plt.pause(0.01)
        plt.clf()
        
# save animation from image list
if save_animation:
    anim = animation.ArtistAnimation(plt.gcf(), frames, interval=100)
    anim.save(save_folder+'animation.mp4', fps=10, writer='ffmpeg')
    
plt.close()