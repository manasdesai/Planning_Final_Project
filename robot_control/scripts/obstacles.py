#!/usr/bin/env python3
import numpy as np


class Obstacles:
    def __init__(self):
        self.obstacles = np.array(
            [
                #[1, 1, 1, 2, 2, 2],
                [0.1, -0.4, 0.1, 1, 1, 1],
                # [-1, -1, -1, 1, 1, 1],
                # [-1, -1, -1, 1, 1, -0.1],
            ],
            dtype=float,
        )  # Modify with actual bb coords


