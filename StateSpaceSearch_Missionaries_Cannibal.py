from time import time
from queue import Queue
import pydot  # to create the visual representation of state space tree
import numpy as np


# GLOBAL CONSTANTS
BG_COLOR = "#c5e0bd"
GRAPH_TYPE = "digraph"
FIGURE_LABEL = "Fig: Missionaries and Cannibal State Space Search Tree Solution"

class Node:
    goal_state = [0, 0, 0]
    num_of_instances = 0

    def __init__(self, state, parent, action, depth):
        self.parent = parent
        self.state = state
        self.action = action
        self.depth = depth
        if self.goal_test():
            color = "#ffd700"  # color for goal state
        else:
            color = "#fffb96"  # color for valid state
        self.graph_node = pydot.Node(str(self), style="filled", fillcolor=color)
        Node.num_of_instances += 1  # increment the number of nodes explored

    def __str__(self):
        return str(self.state)

# Check if the node's state matches the goal state.
    def goal_test(self):
        if self.state == self.goal_state:
            return True
        return False

# Check if the node's state is valid according to the problem constraints.
    def is_valid(self):
        missionaries = self.state[0]
        cannibals = self.state[1]
        boat = self.state[2]
        if missionaries < 0 or missionaries > 3:
            return False
        if cannibals < 0 or cannibals > 3:
            return False
        if boat > 1 or boat < 0:
            return False
        return True

# Check if the node's state is "killed" (violating constraints)
    def is_killed(self):
        missionaries = self.state[0]
        cannibals = self.state[1]
        if missionaries < cannibals and missionaries > 0:
            return True
        # Check for the other side of the river
        if missionaries > cannibals and missionaries < 3:
            return True

# Generates child nodes by applying valid actions (moving missionaries and cannibals).
    def generate_child(self):
        children = []
        depth = self.depth + 1
        op = -1  # Subtract (move boat from left shore to right)
        

        if self.state[2] == 0:
            op = 1  # Add (move boat from right shore to left")
            

# Create child nodes for possible actions and adds them to the children list.
        for x in range(3):
            for y in range(3):
                new_state = self.state.copy()
                new_state[0], new_state[1], new_state[2] = (
                    new_state[0] + op * x,
                    new_state[1] + op * y,
                    new_state[2] + op * 1,
                )
                action = [x, y, op]
                new_node = Node(new_state, self, action, depth)
                if x + y >= 1 and x + y <= 2:
                    children.append(new_node)
        return children

# function to Backtrack from the current node to the root node to find the sequence of actions leading to the current state.
    def find_solution(self): 
        solution = []
        solution.append(self.action)
        path = self
        while path.parent != None:
            path = path.parent
            solution.append(path.action)
        solution = solution[:-1]
        solution.reverse()
        return solution

# A placeholder function for drawing a legend for the graph.
def draw_legend(graph):
    graphlegend = pydot.Cluster(
        fontsize="20",
        color="red",
        fontcolor="blue",
        style="filled",
        fillcolor="white",
    )

# Implements the BFS algorithm to explore the state space tree.
def bfs(initial_state):
    # Initialize a graph object for visualization.
    graph = pydot.Dot(
        graph_type=GRAPH_TYPE, bgcolor=BG_COLOR, label=FIGURE_LABEL
    )
    start_node = Node(initial_state, None, None, 0)

    # return the solution immediately if the start node satisfies the goal test
    if start_node.goal_test():
        return start_node.find_solution()

    q = Queue()  # empty queue
    q.put(start_node)
    explored = []
    killed = []
    print("The starting node is \ndepth=%d" % start_node.depth)
    print(str(start_node.state))

    # While the queue is not empty, explore nodes.
    while not q.empty():
        node = q.get()
        print(
            "\nThe node selected to expand is\ndepth="
            + str(node.depth)
            + "\n"
            + str(node.state)
            + "\n"
        )

        # Add nodes to the graph, add edges, and assign colors based on node characteristics.
        
        explored.append(node.state)
        graph.add_node(node.graph_node)
        if node.parent:
            diff = np.subtract(node.parent.state, node.state)
            if node.parent.state[2] == 0:
                diff[0], diff[1] = -diff[0], -diff[1]
            graph.add_edge(
                pydot.Edge(node.parent.graph_node, node.graph_node, label=str(diff))
            )

        # Generate child nodes, check if they are valid, and add them to the queue.

        children = node.generate_child()
        if not node.is_killed():
            print("The children nodes of this node are", end="")
            for child in children:
                if child.state not in explored:
                    print("\ndepth=%d" % child.depth)
                    print(str(child.state))
                    if child.goal_test():
                        print("which is the goal state\n")

                        # Add nodes to the graph, add edges, and assign colors based on node characteristics.
                        graph.add_node(child.graph_node)
                        diff = np.subtract(node.parent.state, node.state)
                        if node.parent.state[2] == 0:
                            diff[0], diff[1] = -diff[0], -diff[1]
                        graph.add_edge(
                            pydot.Edge(
                                child.parent.graph_node,
                                child.graph_node,
                                label=str(diff),
                            )
                        )

                        # colour all leaves Salmon (which is killed basically)
                        leafs = {n.get_name(): True for n in graph.get_nodes()}

                        for e in graph.get_edge_list():
                            leafs[e.get_source()] = False
                        for leaf in leafs:
                            if (
                                leafs[leaf]
                                and str(leaf) not in killed
                                and str(leaf) != '"[0, 0, 0]"'
                            ):
                                node = pydot.Node(
                                    leaf, style="filled", fillcolor="#fa8072"
                                )
                                graph.add_node(node)

                        draw_legend(graph)
                        graph.write_png("StateSpaceSearchTree_Missionaries_Cannibal.png")

                        return child.find_solution()

                    if child.is_valid():
                        q.put(child)
                    explored.append(child.state)
                else:
                    print("This node is killed")
                    killed.append('"' + str(node.state) + '"')

# Entry point for the program

if __name__ == "__main__":
    initial_state = [3, 3, 1]  # initial state of the game with 3 missionaries and 3 cannibals

    start_time = time()
    solution = bfs(initial_state)
    final_time = time() - start_time
    # print(f"Solution is: {solution}")
    print(f"Number of Nodes Explored: {Node.num_of_instances}")
    print(f"Total Time taken: {final_time} seconds.")

