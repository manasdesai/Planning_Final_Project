#!/usr/bin/env python3
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

        # self.path = self._search()

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
        # return np.array(path)
        return path

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
