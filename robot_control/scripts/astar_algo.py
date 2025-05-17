
import numpy as np
from queue import PriorityQueue
from prm import PRM


class AStar:
    def __init__(self, prm: PRM, c_start:np.array, c_goal:np.array):

        self.start = prm.add(c_start)
        self.goal = prm.add(c_goal)
        
        self.prm = prm
        self.nodes = prm.nodes
        self.nodes_kd = prm.valid_kd
        self.connections = prm.connections

        self.q = PriorityQueue()
        self.open_set = set()
        self.closed_set = set()
        self.parents = {}
        self.g_scores = {}

        self.path = self._search()

    def _dist(self, idx1, idx2):
        """Euclidean distance between nodes by index."""
        return np.linalg.norm(self.nodes[idx1] - self.nodes[idx2])

    def _backtrack(self):
        """Reconstruct path from goal to start using parent links."""
        path = []
        current = self.goal

        while current is not None:
            path.append(self.nodes[current])
            current = self.parents.get(current)

        path.reverse()
        print("A* path:")
        for p in path:
            print(p)
        return np.array(path)

    def _search(self):
        """A* search algorithm."""
        self.q.put((0.0, self.start))
        self.open_set.add(self.start)
        self.g_scores[self.start] = 0.0
        self.parents[self.start] = None

        while not self.q.empty():
            _, current = self.q.get()
            if current in self.closed_set:
                continue

            self.closed_set.add(current)
            self.open_set.discard(current)

            if current == self.goal:
                print("A* reached the goal.")
                return self._backtrack()

            for neighbor, cost in self.connections[current]:
                tentative_g = self.g_scores[current] + cost

                if neighbor in self.closed_set:
                    continue

                if neighbor not in self.open_set or tentative_g < self.g_scores.get(
                    neighbor, float("inf")
                ):
                    self.parents[neighbor] = current
                    self.g_scores[neighbor] = tentative_g
                    f_score = tentative_g + self._dist(neighbor, self.goal)
                    self.q.put((f_score, neighbor))
                    self.open_set.add(neighbor)

        print("A* failed to find a path.")
        return None



# import numpy as np
# from scipy.spatial import KDTree
# import pdb
# from queue import PriorityQueue
# from prm import PRM


# class AStar:
#     def __init__(self, prm: PRM, c_start, c_goal):
#         self.connections = prm.connections
#         self.nodes_kd = prm.valid_kd
#         self.start = prm.add(c_start)
#         self.goal = prm.add(c_goal)

#         self.q = PriorityQueue()
#         self.open = set()
#         self.visited = set()
#         self.parents = dict()
#         self.g_scores = dict()

#         self.nodes = prm.nodes
#         self.path = self.search()

#     def dist(self, c1, c2):
#         return np.linalg.norm(c2 - c1, ord=2)

#     def backtrack(self, goal):
#         path = []
#         current = goal

#         print("BACKTRACKING the path")
#         print(self.parents)
#         # print(self.nodes)

#         # while current is not None:
#         #     path.append(self.nodes[current])
#         #     current = self.parents.get(current)
#         # path.reverse()  # From start to goal
#         print(f"backtrack path: {path}")

#         return np.array(path)

#     def search(self):
#         start, goal = self.start, self.goal  # TODO: Proper index required
#         goal_found = True

#         print(f"START: {start}")
#         print(f"GOAL: {goal}")

#         # Handle the start node
#         self.q.put((0.0, start))  # Fix this later for correctness
#         self.open.add(start)
#         self.g_scores[start] = 0.0

#         self.parents[start] = None
#         self.parents['a'] = -1 # dummy var

#         # pdb.set_trace()

#         while not self.q.empty():
#             # Pop the most promising node
#             _, current = self.q.get()
#             if current in self.visited:
#                 continue

#             # Visit the most promising node
#             self.visited.add(current)

#             # Remove the most promising node from the open set
#             self.open.remove(current)

#             # Is this the goal?
#             if current == goal:
#                 goal_found = True
#                 print("A* found the goal")
#                 print(f"current: {current}")
#                 break

#             for neighbor, distance in self.connections[current]:
#                 if neighbor in self.open:
#                     tentative_g_score = self.g_scores[current] + distance
#                     if self.g_scores[neighbor] <= tentative_g_score:
#                         continue
#                     elif self.g_scores[neighbor] > tentative_g_score:
#                         self.parents[neighbor] = current
#                         self.g_scores[neighbor] = tentative_g_score
#                         f_score = self.g_scores[neighbor] + self.dist(
#                             self.nodes[neighbor], self.nodes[goal]
#                         )
#                         self.q.put((f_score, neighbor))
#                 else:
#                     self.g_scores[neighbor] = self.g_scores[current] + distance
#                     self.parents[neighbor] = current
#                     f_score = self.g_scores[neighbor] + self.dist(
#                         self.nodes[neighbor], self.nodes[goal]
#                     )
#                     self.q.put((f_score, neighbor))
#                     self.open.add(neighbor)

#         if goal_found:
#             print("Goal Found")
#             return self.backtrack(goal)
#         else:
#             print("Goal NOT Found")

#         return None

