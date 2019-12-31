
import numpy as np

class node:

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def set_g(self, G):
        self.G = G

    def set_rhs(self, rhs):
        self.rhs = rhs



class mt_dstar_lite:

    def __init__(self, height, width):
        # Graph properties
        self.H = height
        self.W = width

        # Create a graph for searching
        self.grid = np.ndarray(shape = (self.H, self.W), dtype = node)

        # Load the numpy array with graph nodes
        for i in range(self.H):
            for j in range(self.W):
                self.grid[i, j] = node(i, j)

    def plan(self, nodes, robot_pos, goal_pos):
        pass



