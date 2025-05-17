import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
import pdb


class PRM:
    def __init__(self, robot, obstacles, N=1000, K=6):
        self.N = N
        self.K = K
        self.robot = robot
        self._max_iterations = N * 5
        self.obstacles = obstacles
        self.nodes, self.connections, self.valid_kd = self.construct()

    def construct(self):
        # --- Generate N valid samples ---
        valid = []
        count = 1
        while len(valid) < self.N and count <= self._max_iterations:
            node = self.sample()
            if self.collision_vectorized(node):
                count += 1
                print("Obstacled detected.")
                continue
            valid.append(node)
        if count >= self._max_iterations:
            print("Point does not exist")
            exit()

        # --- Build KD Tree for valid samples ---
        valid = np.array(valid)
        valid_kd = KDTree(valid)

        # --- Connect each node with up to K neighbors ---
        connections = dict()
        for i, node in enumerate(valid):
            print(f"Connecting Node {i}")
            ds, js = valid_kd.query([node], k=self.K)
            for d, j in zip(ds[0], js[0]):
                if j == i or j < i:
                    continue
                c1 = valid[i, :]
                c2 = valid[j, :]
                if not self.collision_between(c1, c2):
                    connections.setdefault(i, []).append((j, d))
                    connections.setdefault(j, []).append((i, d))
        return valid, connections, valid_kd

    def sample(self):
        return np.random.uniform(low=0.0, high=2 * np.pi, size=(6,))

    def add(self, c):
        """
        Fill in some documentation later.
        """
        if self.collision_vectorized(c):
            print("This configuration is not in free-space.")
            return None
        ds, js = self.valid_kd.query([c], k=self.K)
        i = len(self.nodes)  # Assign c and index `i` in valid
        self.nodes = np.vstack([self.nodes, c])

        successful_connection = False
        for d, j in zip(ds[0], js[0]):
            neighbor = self.nodes[j]
            if not self.collision_between(c, neighbor):
                self.connections.setdefault(i, []).append((j, d))
                self.connections.setdefault(j, []).append((i, d))
                successful_connection = True
        if successful_connection:
            self.valid_kd = KDTree(self.nodes)
            return i
        else:
            self.nodes = self.nodes[:-1]
            print("No valid neighbors found for this configuration")
            return None

    def collision_vectorized(self, c):
        """
        Checks whether a given joint configuration is in free space.
        """
        pts = self.robot.generate_interpolated_robot_points(c)
        xs, ys, zs = pts[:, 0], pts[:, 1], pts[:, 2]
        for xmin, ymin, zmin, xmax, ymax, zmax in self.obstacles:
            collision = (
                (xs >= xmin)
                & (xs <= xmax)
                & (ys >= ymin)
                & (ys <= ymax)
                & (zs >= zmin)
                & (zs <= zmax)
            )
            if collision.any():
                return True
        return False

    def collision_between(self, c1, c2, step=0.05):
        """
        Checks whether the interpolated path between two points in configuration space is entirely in free space.
        """
        disp = c2 - c1
        dist = np.linalg.norm(disp)
        if dist == 0:
            return False
        unit = disp / dist
        # Assumes c1 and c2 exist in free space
        for i in range(1, int(dist / step) + 1):
            c = c1 + unit * step * i
            # if self.collision(c):
            #    return True
            if self.collision_vectorized(c):
                return True
        return False

    def dist(self, c1, c2):
        # Return the 2 norm of the c-space config
        return np.linalg.norm(c2 - c1, ord=2)
