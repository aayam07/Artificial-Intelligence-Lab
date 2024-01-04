import networkx as nx
import heapq
import matplotlib.pyplot as plt
import pydotplus

# Initial and goal states (from class slide example)
initial_state = (2, 8, 3, 1, 6, 4, 7, 0, 5) 
goal_state = (1, 2, 3, 8, 0, 4, 7, 6, 5)

# Function to calculate number of misplaced tiles heuristic
def misplaced_tiles(state, goal_state):
    count = 0
    for i in range(len(state)):
        if state[i] != goal_state[i]:  # Count how many tiles are not in their goal position
            count += 1
    return count

# Function to get possible moves
def get_moves(state):
    moves = []
    zero_index = state.index(0)  # Finding the index of the empty space (represented by 0)
    # Checking which moves are possible (up, down, left, right) from the current state
    if zero_index - 3 >= 0:
        moves.append('U')  # U represents moving the empty space Up
    if zero_index + 3 < len(state):
        moves.append('D')  # D represents moving the empty space Down
    if zero_index % 3 != 0:
        moves.append('L')  # L represents moving the empty space Left
    if zero_index % 3 != 2:
        moves.append('R')  # R represents moving the empty space Right
    return moves

# Function to perform moves
def make_move(state, move):
    zero_index = state.index(0)
    new_state = list(state)
    # Swapping the empty space (0) with the adjacent tile according to the move chosen
    if move == 'U':
        new_state[zero_index - 3], new_state[zero_index] = new_state[zero_index], new_state[zero_index - 3]
    elif move == 'D':
        new_state[zero_index + 3], new_state[zero_index] = new_state[zero_index], new_state[zero_index + 3]
    elif move == 'L':
        new_state[zero_index - 1], new_state[zero_index] = new_state[zero_index], new_state[zero_index - 1]
    elif move == 'R':
        new_state[zero_index + 1], new_state[zero_index] = new_state[zero_index], new_state[zero_index + 1]
    return tuple(new_state)

def astar_with_tree(start, goal):
    pq = [] # Priority queue to store states with priorities
    heapq.heappush(pq, (misplaced_tiles(start, goal), 0, start, []))  # Adding start state to the queue with its priority
    visited = set() # Set to keep track of visited states

    G = nx.Graph()
    G.add_node(start)

    while pq:  # While there are elements in the priority queue
        _, cost, current_state, path = heapq.heappop(pq)  # Pop the state with the lowest priority

        if current_state == goal:  # If the current state is the goal state
            G.add_node(goal, style='filled', fillcolor='red', shape='doublecircle')  # Highlighting the goal node
            break

        visited.add(current_state)  # Mark the current state as visited

        moves = get_moves(current_state)  # Get possible moves from the current state
        for move in moves:
            new_state = make_move(current_state, move)  # Create a new state by making a move
            if new_state not in visited:  # If the new state is not visited
                new_cost = cost + 1  # Increment the cost
                priority = new_cost + misplaced_tiles(new_state, goal)  # Calculate priority using heuristic
                heapq.heappush(pq, (priority, new_cost, new_state, path + [current_state]))   # Add to the queue
                G.add_node(new_state)
                G.add_edge(current_state, new_state)

    # Write the Graph to a DOT file using NetworkX
    nx.drawing.nx_pydot.write_dot(G, "8_puzzle_states_aayam.dot")

    return path + [current_state] if current_state == goal else None  # Return the path taken to reach the goal


# Solve the puzzle
solution = astar_with_tree(initial_state, goal_state)

if solution:
    for i, state in enumerate(solution):
        print(f"Step {i}:")
        print(state[0:3])
        print(state[3:6])
        print(state[6:9])
        print("\n")
else:
    print("No solution")
