
import numpy as np
import heapq

class Node:

    INFINITY = 1000000000

    def __init__(self, x, y):
        self.x = x
        self.y = y

        # Default node value
        self.G = self.INFINITY
        self.rhs = self.INFINITY
        self.par = None
        self.key = (None, None)

    def set_g(self, G):
        self.G = G

    def set_rhs(self, rhs):
        self.rhs = rhs

    def set_par(self, par):
        self.par = par

    def set_key(self, key):
        self.key = key

    def __cmp__(self, other):
        """ Sort keys with largest being highest priority """
        # Sort by the first key
        comp_val = -cmp(self.key[0], other.key[0])
        if comp_val != 0:
            return comp_val

        # If there was a tie, use the second key as a tiebreaker
        return -cmp(self.key[1], other.key[1])

    def __eq__(self, other):
        return (self.x == other.x) && (self.y == other.y)



class OpenList:

    def __init__(self):
        """ Initialize the open list """
        # Set up an empty max heap using list and heapq
        self.max_heap = []
        heapq.heapify(self.max_heap)

    def update(self, node):
        """ Update a node entry in the open list """
        if node in self.max_heap:
            self.delete(node)
            self.insert(node)

    def insert(self, node):
        """ Add a node to the queue """
        # Push the node to the heap
        heapq.heappush(self.max_heap, node)

    def delete(self, node):
        """ Delete a node from the open list """
        if node in self.max_heap:
            self.max_heap.remove(node)
            heapq.heapify(self.max_heap)

    def top_key(self):
        """ Get the top value in the heap """
        # The top key will be the "lowest value." This is because our max heap is actually just a min-heap with a comparator that sorts in reverse
        return min(self.max_heap).key

    def top(self):
        """ Get the top value from the list """
        return heapq.heappop(self.max_heap)

    def contains(self, val):
        return val in self.max_heap



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

    def get_node(pos):
        """ Gets a node at a given (row, col) position from the grid """
        return self.grid[pos[0], pos[1]]


# Algorithm: http://idm-lab.org/bib/abstracts/papers/aamas10a.pdf
class mt_dstar_lite:

    def __init__(self, width, height, robot_pos, goal_pos):
        """ Initialize the Moving Target D* Lite algorithm """
        # Search variables
        self.km = 0
        self.open_list = OpenList()

        # Create a search space
        self.search_space = SearchSpace(width, height)

        # Get the node the robot is on
        self.start_node = self.search_space.get_node(robot_pos)

        # Get the node the goal is on
        self.goal_node = self.search_space.get_node(goal_pos)

        # Set the robot's node's rhs = 0
        self.start_node.set_rhs(0)

        # Add the robot's node to the open list
        self.open_list.insert(self.start_node, self.calculate_key(self.start_node))


    def calculate_key(self, node):
        """ Calculates a node key based on its G and RHS values, as well as the distance to the goal """
        return (min(node.G, node.rhs) + self.h(node, self.goal_node), min(node.G, node.rhs))


    def h(self, node1, node2):
        """ Compute the distance from one node to another """
        return ((node2.x - node1.x)**2 + (node2.y - node1.y)**2)


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
                # TODO

            # Otherwise TODO
            else:
                # Set G value to infinity
                u_node.set_g(Node.INFINITY)

                # Update successors
                # TODO

    def basic_deletion():
        """ """
        self.start_node.set_par(None)

        # Calculate the
        #self.old_start.set_rhs()

    def optimized_deletion():
        """ """
        # Initialize deletion
        self.deleted_list = set()
        self.start_node.set_par(None)

    def plan(self, nodes, robot_pos, goal_pos):
        pass

    # IDEA: in order to reap the benefits of MT-D*Lite, we need to preserve the old part of the map that we have not traversed
    # To do this, we can find how much the robot has moved (row and col offset) and then shift the current grid so the robot is in the middle / bottom.
    # Then we can populate the new empty spaces in the grid with uninitialized nodes, and then assign a cost to them. We can also update the best position
    # as this is the moving target.
    # This way, we keep previous local frames in the path planner independent of the SLAM algorithm. We don't need to update the entire grid of nodes each time.
    # This is a sketchy version of SLAM because it doesn't use keyframing. So we just assume the first reading was accurate and move on.
    def update_grid(self, robot_offset, new_goal):
        pass

