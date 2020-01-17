
from math import sqrt
import numpy as np
import heapq

class Node:

    INFINITY = 1000000000

    def __init__(self, row, col, x, y, cost = 0):
        self.row = row
        self.col = col
        self.x = x
        self.y = y

        # Default node value
        self.G = self.INFINITY
        self.rhs = self.INFINITY
        self.par = None
        self.key = (None, None)

        # Cost (for obstacles and such)
        self.cost = cost

    def set_g(self, G):
        self.G = G

    def set_rhs(self, rhs):
        self.rhs = rhs

    def set_par(self, par):
        self.par = par

    def set_key(self, key):
        self.key = key

    def set_cost(self, cost):
        self.cost = cost

    def __cmp__(self, other):
        """ Sort keys with lowest priority to the top of the list"""
        # Sort by the first key
        comp_val = cmp(self.key[0], other.key[0])
        if comp_val != 0:
            return comp_val

        # If there was a tie, use the second key as a tiebreaker
        return cmp(self.key[1], other.key[1])

    def __eq__(self, other):
        return (self.x == other.x) && (self.y == other.y)



class OpenList:

    def __init__(self):
        """ Initialize the open list """
        # Set up an empty min heap using list and heapq
        self.min_heap = []
        heapq.heapify(self.min_heap)

    def update(self, node):
        """ Update a node entry in the open list """
        if node in self.min_heap:
            self.delete(node)
            self.insert(node)

    def insert(self, node):
        """ Add a node to the queue """
        # Push the node to the heap
        heapq.heappush(self.min_heap, node)

    def delete(self, node):
        """ Delete a node from the open list """
        if node in self.min_heap:
            self.min_heap.remove(node)
            heapq.heapify(self.min_heap)

    def top_key(self):
        """ Get the top value in the heap """
        if len(self.min_heap) == 0:
            return (INFINITY, INFINITY)
        # The top key will be the lowest priority
        return min(self.min_heap).key

    def top(self):
        """ Get the top value from the list """
        if len(self.min_heap) == 0:
            return None
        return heapq.heappop(self.min_heap)

    def contains(self, val):
        return val in self.min_heap



class SearchSpace:

    def __init__(self, width, height):
        # Graph properties
        self.W = width
        self.H = height

        # Create a graph for searching
        self.grid = np.ndarray(shape = (self.H, self.W), dtype = Node)

        # Load the numpy array with graph nodes
        for i in range(self.H):
            for j in range(self.W):
                self.grid[i, j] = Node(i, j)

    def get_node(self, pos):
        """ Gets a node at a given (row, col) position from the grid """
        return self.grid[pos[0], pos[1]]

    def get_successors(self, node):
        succ = []
        for i in range(node.x-1, node.x+1):
            for j in range(node.y-1, node.y+1):
                succ.append(self.grid[i,j])

        return succ

    def load_search_space_from_map(self, map):
        pass


# Algorithm: http://idm-lab.org/bib/abstracts/papers/aamas10a.pdf
class mt_dstar_lite:

    def __init__(self, width, height, robot_pos, goal_pos):
        """ Initialize the Moving Target D* Lite algorithm """
        # Search variables
        self.km = 0
        self.open_list = OpenList()

        # Create a search space
        self.search_space = SearchSpace(width, height)

        # Get the node the robot is on (row, col is the pos argument)
        self.start_node = self.search_space.get_node(robot_pos)

        # Get the node the goal is on (row, col is the pos argument)
        self.goal_node = self.search_space.get_node(goal_pos)

        # Set the robot's node's rhs = 0
        self.start_node.set_rhs(0)

        # Add the robot's node to the open list
        self.open_list.insert(self.start_node, self.calculate_key(self.start_node))


    def calculate_key(self, node):
        """ Calculates a node key based on its G and RHS values, as well as the distance to the goal """
        return (min(node.G, node.rhs) + self.heuristic(node, self.goal_node), min(node.G, node.rhs))


    def heuristic(self, node1, node2):
        """ Compute the distance from one node to another """
        return sqrt((node2.x - node1.x)**2 + (node2.y - node1.y)**2)

    def cost(self, node1, node2):
        """ Computer actual cost incurred by moving from one node to another """
        if node1.cost < INFINITY and node2.cost < INFINITY:
            return heuristic(node1, node2) + node1.cost + node2.cost
        else:
            return INFINITY


    def update_state(self, node):
        """ Updates a node's status based on the progression of the search """
        # If the node is on the open list and is inconsistent, update it
        if node.G != node.rhs and self.open_list.contains(node):
            node.set_key(self.calculate_key(node))
            self.open_list.update(node)

        # If the node is not on the open list, but is inconsistent, then add it to the open list
        elif node.G != node.rhs and not self.open_list.contains(node):
            node.set_key(self.calculate_key(node))
            self.open_list.insert(node)

        # If the node is on the open list and is consistent, close it by removing it from the open list
        elif node.G == node.rhs and self.open_list.contains(node):
            self.open_list.delete(node)


    def compute_cost_minimal_path(self):
        """ Finds the best path from the start state to the goal state """
        while self.open_list.top_key() < self.calculate_key(self.goal_node) or self.goal_node.rhs > self.goal_node.G:
            # Get the highest priority node from the open list
            u_node = self.open_list.top()

            # Compute keys for this node
            old_key = u_node.key
            new_key = self.calculate_key(u_node)

            # If the new key is greater than the old key, update the open list
            if old_key < new_key:
                u_node.set_key(new_key)
                self.open_list.update(u_node)

            # Otherwise, if the node's G value is higher than its RHS value, close the node and update its successors
            elif u_node.G > u_node.rhs:
                # Make the node consistent and remove it from the open list
                u_node.set_g(u_node.rhs)
                self.open_list.delete(u_node)

                # Update successors
                for s_node in self.search_space.get_successors(u_node):
                    if s_node != self.start_node and s_node.rhs > s_node.G + self.cost(u_node, s_node):
                        s_node.set_rhs(s_node.G + self.cost(u_node, s_node))
                        s_node.set_par(u_node)
                        self.update_state(s_node)


            # Otherwise make the node inconsistent and calculate the parent nodes of its successors
            else:
                # Set G value to infinity
                u_node.set_g(Node.INFINITY)

                # Update successors
                for s_node in self.search_space.get_successors(u_node):
                    if s_node != self.start_node and s_node.par == u_node:
                        # Set the RHS to be the minimum one-step lookahead
                        s_rhs = min([node.G + self.cost(node, s_node) for node in self.search_space.get_successors(s_node)])
                        s_node.set_rhs(s_rhs)

                        if s_node.rhs >= INFINITY:
                            s_node.set_rhs(INFINITY)
                            s_node.set_par(None)
                        else:
                            # Set the parent node pointer based on the parent node that has the minimum path cost to goal from this node
                            par_nodes = [node for node in self.search_space.get_successors(s_node)]
                            s_par = [node.G + self.cost(node, s_node) for node in par_nodes]
                            s_par_node_idx = s_par.index(min(s_par))
                            s_par_node = par_nodes[s_par_node_idx]
                            s_node.set_par(s_par_node)
                    self.update_state(s_node)

    def basic_deletion(self):
        """ """
        self.start_node.set_par(None)

        # Calculate the
        #self.old_start.set_rhs()

    def optimized_deletion(self):
        """ """
        # Initialize deletion
        self.deleted_list = set()
        self.start_node.set_par(None)

    def plan(self, nodes, robot_pos, goal_pos):
        pass


