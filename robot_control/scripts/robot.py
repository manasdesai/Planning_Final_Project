import numpy as np 


class RobotArm:
    """
    This class encapsulates the concept of the Barista robot. 
    The Barista robot operates within an environment with known
    obstacles. The Barista robot also has DH parameters. 
    """ 
    def __init__(self):
        """
        Sets the `obstacles` data structure. This holds the arm's obstacle bounding box coordinates.  
        Sets the `dh_table` data structure. This holds the intrinsic DH parameters for the arm, which are used for forward kinematics calculations when building a PRM. 
        """
        # DH parameters for each joint
        self.dh_table = {
            0: {"a": 0, "d": 0.1807, "alpha": np.pi/2},
            1: {"a": -.6127, "d": 0, "alpha": 0},
            2: {"a": -.57155, "d": 0, "alpha": 0},
            3: {"a": 0, "d": 0.17415, "alpha": np.pi/2},
            4: {"a": 0, "d": 0.11985, "alpha": -np.pi/2},
            5: {"a": 0, "d": 0.11655, "alpha": 0},
        }

    def generate_DH_matrix(self, a, d, alpha, theta):
        """
        This method generates a DH transform matrix for a given joint given a set of DH parameters.  
        """
        return np.array(
            [
                [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
                [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
                [0, np.sin(alpha), np.cos(alpha), d],
                [0, 0, 0, 1]
            ], dtype=float
        )
    
    def generate_link_end_points(self, q):
        """
        Returns a NumPy array of key points, computed with forward kinematics, along all links to use for collision checking.
        """
        link_ends = [[0,0,0]]
        T = np.eye(4)
        for link_index in range(len(q)):
            theta = q[link_index]
            a = self.dh_table[link_index]["a"]
            d = self.dh_table[link_index]["d"]
            alpha = self.dh_table[link_index]["alpha"]
            T = T @ self.generate_DH_matrix(a, d, alpha, theta)
            link_ends.append(T[:3,3])
        return np.array(link_ends)
    
    def generate_interpolated_robot_points(self, q, res=5):
        points = self.generate_link_end_points(q)
        interpolated = []
        num_segments = len(points) - 1
        for i in range(num_segments): 
            start_point = points[i]
            end_point = points[i+1]
            segment_points = np.linspace(start_point, end_point, res)
            if i == 0:
                interpolated.append(segment_points)
            else:
                interpolated.append(segment_points[1:])
        interpolated_array = np.concatenate(interpolated, axis=0)
        return interpolated_array