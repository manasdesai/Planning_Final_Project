import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from astar import AStar
from obstacles import Obstacles
from prm import PRM
from robot import RobotArm

# How many random samples to try to generate in our PRM
PRM_NODES = 1000

def plot_prm_roadmap(robot, prm, start_idx, goal_idx):
    """
    Draw the PRM graph in workspace (end‑effector) coordinates.
    Samples in blue, edges in gray, start in green, goal in red.
    """
    # compute all end‑effector positions for each sampled configuration
    ee_positions = np.array([robot.generate_link_end_points(q)[-1] for q in prm.nodes])
    
    # pick out start and goal positions by index
    ee_start, ee_goal = ee_positions[start_idx], ee_positions[goal_idx]

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(*ee_positions.T, c='blue', s=20, alpha=0.6, label='Samples')

    # draw each PRM edge by connecting the ee positions
    for i, neighbors in prm.connections.items():
        p0 = ee_positions[i]
        for j, _ in neighbors:
            p1 = ee_positions[j]
            ax.plot([p0[0], p1[0]], [p0[1], p1[1]], [p0[2], p1[2]],
                    c='gray', linewidth=0.5, alpha=0.5)

    # standout markers for start and goal
    ax.scatter(*ee_start, c='green', s=100, marker='^', label='Start')
    ax.scatter(*ee_goal,  c='red',   s=100, marker='X', label='Goal')

    ax.set(title='PRM Roadmap: End Effector Positions',
           xlabel='X [m]', ylabel='Y [m]', zlabel='Z [m]')
    ax.set_box_aspect((1, 1, 1))  # equal axis scales
    ax.legend()
    plt.show()


def plot_configurations(robot, prm, start_idx, goal_idx, obstacles):
    """
    Visualize actual robot link poses for each sample,
    with obstacles overlaid and start/goal highlighted.
    """
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')

    # draw all sampled robot poses
    for q in prm.nodes:
        links = robot.generate_link_end_points(q)
        ax.plot(links[:, 0], links[:, 1], links[:, 2], alpha=0.4)

    # show start and goal configurations more prominently
    for idx, color, marker, label in [
        (start_idx, 'green', '^', 'Start'),
        (goal_idx,  'red',   'X', 'Goal')
    ]:
        links = robot.generate_link_end_points(prm.nodes[idx])
        ax.plot(links[:, 0], links[:, 1], links[:, 2],
                c=color, marker=marker, linewidth=2, label=label)

    # overlay each obstacle as a translucent red box
    for xmin, ymin, zmin, xmax, ymax, zmax in obstacles:
        # define the cube's 8 corners
        corners = np.array([
            [xmin, ymin, zmin], [xmax, ymin, zmin],
            [xmax, ymax, zmin], [xmin, ymax, zmin],
            [xmin, ymin, zmax], [xmax, ymin, zmax],
            [xmax, ymax, zmax], [xmin, ymax, zmax],
        ])
        # build the 6 faces from those corners
        faces = [
            [corners[i] for i in face]
            for face in [[0,1,2,3], [4,5,6,7],
                         [0,1,5,4], [2,3,7,6],
                         [1,2,6,5], [3,0,4,7]]
        ]
        ax.add_collection3d(
            Poly3DCollection(faces, facecolors='red', alpha=0.3, edgecolors='k')
        )

    ax.set(title='Sample Robot Configurations with Obstacles',
           xlabel='X [m]', ylabel='Y [m]', zlabel='Z [m]')
    ax.set_box_aspect((1, 1, 1))
    ax.legend()
    plt.show()


def main():
    # initialize our 6‑DOF robot arm model
    robot = RobotArm()
    # load obstacle definitions (list of axis‑aligned boxes)
    ob = Obstacles()
    # build the probabilistic roadmap in configuration space
    prm = PRM(robot, ob.obstacles, N=PRM_NODES, K=5)

    # define start and goal joint angles (6‑vector each)
    c_start = np.array([0.0, -0.5, 0.5, -0.3, 0.6, 0.2])
    c_goal  = np.array([0.0, -np.pi/2, np.pi/2, 0.0, 0.0, 0.0])

    # run A* over the PRM graph to find feasible path indices
    astar_planner = AStar(prm, c_start, c_goal)
    start_idx, goal_idx = astar_planner.start, astar_planner.goal

    # two visual demos: the workspace roadmap and the actual link poses
    plot_prm_roadmap(robot, prm, start_idx, goal_idx)
    plot_configurations(robot, prm, start_idx, goal_idx, ob.obstacles)


if __name__ == '__main__':
    main()
