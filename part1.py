import sys
from collections import deque
import heapq

is_astar = False
is_greedy = False

class Node:
    def __lt__(self, other):
        if (is_astar):
            return self.cost() < other.cost()
        elif (is_greedy):
            return self.heuristic() > other.heuristic()
        else:
            return self.path_cost < other.path_cost
    def __eq__(self, other):
        if other == None:
            return False
        return self.state == other.state
    def __repr__(self):
        return str(self.state)
    def cost (self):
        return self.path_cost + self.heuristic()
    def heuristic(self):
        distance = 0
        for i in range(3):
            for j in range(3):
                if self.state[i][j] == 0:
                    continue
                elif self.state[i][j] == 1:
                    distance += abs(i - 0) + abs(j - 0)
                elif self.state[i][j] == 2:
                    distance += abs(i - 0) + abs(j - 1)
                elif self.state[i][j] == 3:
                    distance += abs(i - 0) + abs(j - 2)
                elif self.state[i][j] == 4:
                    distance += abs(i - 1) + abs(j - 0)
                elif self.state[i][j] == 5:
                    distance += abs(i - 1) + abs(j - 1)
                elif self.state[i][j] == 6:
                    distance += abs(i - 1) + abs(j - 2)
                elif self.state[i][j] == 7:
                    distance += abs(i - 2) + abs(j - 0)
                elif self.state[i][j] == 8:
                    distance += abs(i - 2) + abs(j - 1)
        return distance
    
    def __hash__(self):
        return self.state[0][0] + self.state[0][1] * 10 + self.state[0][2] * 100 + self.state[1][0] * 1000 + self.state[1][1] * 10000 + self.state[1][2] * 100000 + self.state[2][0] * 1000000 + self.state[2][1] * 10000000 + self.state[2][2] * 100000000
    def __init__(self, state, parent, action, path_cost):
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost    


def up(state):
    empty_row = 0
    empty_col = 0
    for i in range(3):
        for j in range(3):
            if state[i][j] == 0:
                empty_row = i
                empty_col = j 
    if empty_row == 0:
        return None
    new_state = [row[:] for row in state]
    new_state[empty_row][empty_col] = new_state[empty_row - 1][empty_col]
    new_state[empty_row - 1][empty_col] = 0
    empty_row -= 1
    return new_state

def right(state):
    empty_row = 0
    empty_col = 0
    for i in range(3):
        for j in range(3):
            if state[i][j] == 0:
                empty_row = i
                empty_col = j 
    if empty_col == 2:
        return None
    new_state = [row[:] for row in state]
    new_state[empty_row][empty_col] = new_state[empty_row][empty_col + 1]
    new_state[empty_row][empty_col + 1] = 0
    empty_col += 1
    return new_state

def down(state):
    empty_row = 0
    empty_col = 0
    for i in range(3):
        for j in range(3):
            if state[i][j] == 0:
                empty_row = i
                empty_col = j 
    if empty_row == 2:
        return None
    new_state = [row[:] for row in state]
    new_state[empty_row][empty_col] = new_state[empty_row + 1][empty_col]
    new_state[empty_row + 1][empty_col] = 0
    empty_row += 1
    return new_state

def left(state):
    empty_row = 0
    empty_col = 0
    for i in range(3):
        for j in range(3):
            if state[i][j] == 0:
                empty_row = i
                empty_col = j 
    if empty_col == 0:
        return None
    new_state = [row[:] for row in state]
    new_state[empty_row][empty_col] = new_state[empty_row][empty_col - 1]
    new_state[empty_row][empty_col - 1] = 0
    empty_col -= 1
    return new_state
    
def expand(node):
    expanded_nodes = []
    up_state = up(node.state)
    if up_state != None:
        expanded_nodes.append(Node(up_state, node, 'U', node.path_cost+1))
    right_state = right(node.state)
    if right_state != None:
        expanded_nodes.append(Node(right_state, node, 'R', node.path_cost+1))
    down_state = down(node.state)
    if down_state != None:
        expanded_nodes.append(Node(down_state, node, 'D', node.path_cost+1))
    left_state = left(node.state)
    if left_state != None:
        expanded_nodes.append(Node(left_state, node, 'L', node.path_cost+1))
    return expanded_nodes
    
def bfs(initial_state, goal_state):
    fringe = deque()
    visited_states = set()
    fringe.append(Node(initial_state, None, None, 0))
    expanded_nodes = 0
    while fringe:
        node = fringe.popleft()
        if node in visited_states:
            continue
        visited_states.add(node)
        expanded_nodes += 1
        for child in expand(node):
            if child not in visited_states:
                fringe.append(child)
        if node.state == goal_state:
            expanded_nodes += fringe.__len__()
            path = []
            path_cost = node.path_cost
            while node.parent != None:
                path.append(node.action)
                node = node.parent
            path.reverse()
            return expanded_nodes, path, path_cost
    print("No solution found")
    
def dfs(initial_state, goal_state):
    fringe = []
    visited_states = set()
    fringe.append(Node(initial_state, None, None, 0))
    expanded_nodes = 0
    while fringe:
        node = fringe.pop()
        if node in visited_states:
            continue
        visited_states.add(node)
        expanded_nodes += 1
        for child in reversed(expand(node)):
            if child not in visited_states:
                fringe.append(child)
        if node.state == goal_state:
            expanded_nodes += fringe.__len__()
            path = []
            path_cost = node.path_cost
            while node.parent != None:
                path.append(node.action)
                node = node.parent
            path.reverse()
            return expanded_nodes, path, path_cost
    print("No solution found")

def ucs(initial_state, goal_state):
    fringe = []
    visited_states = set()
    heapq.heappush(fringe,Node(initial_state, None, None, 0))
    expanded_nodes = 0
    while fringe:
        node = heapq.heappop(fringe)
        if node in visited_states:
            continue
        visited_states.add(node)
        expanded_nodes += 1
        for child in expand(node):
            if child not in visited_states:
                heapq.heappush(fringe,child)
        if node.state == goal_state:
            expanded_nodes += fringe.__len__()
            path = []
            path_cost = node.path_cost
            while node.parent != None:
                path.append(node.action)
                node = node.parent
            path.reverse()
            return expanded_nodes, path, path_cost
    print("No solution found")

def greedy(initial_state, goal_state):
    fringe = []
    visited_states = set()
    fringe.append(Node(initial_state, None, None, 0))
    expanded_nodes = 0
    while fringe:
        node = fringe.pop() 
        if node in visited_states:
            continue
        visited_states.add(node)
        expanded_nodes += 1
        for loop_node in sorted(reversed(expand(node))):
            if loop_node not in visited_states:
                fringe.append(loop_node)
        if node.state == goal_state:
            expanded_nodes += fringe.__len__()
            path = []
            path_cost = node.path_cost
            while node.parent != None:
                path.append(node.action)
                node = node.parent
            path.reverse()
            return expanded_nodes, path, path_cost
    print("No solution found")

def astar(initial_state, goal_state):
    global is_astar
    is_astar = True
    res = ucs(initial_state, goal_state)
    is_astar = False
    return res

def main():
    global is_greedy

    input_f = sys.argv[1]
    output_f = sys.argv[2]

    # read the input file
    input_f = open(input_f, 'r')
    input_lines = input_f.readlines()
    init_state = []


    # get the initial state 
    for line in input_lines:
        line = line.strip().split()
        line = [int(i) for i in line]
        init_state.append(line)
    
    input_f.close()

    final_state =  [[1, 2, 3], [4, 5, 6], [7, 8, 0]]

    output_f = open(output_f, 'w')

    expanded_nodes, path, path_cost =  bfs(init_state, final_state)
    output_f.write('Number of expanded nodes: ' + str(expanded_nodes) + '\n')
    output_f.write('Path-cost: ' + str(path_cost) + '\n')
    output_f.write('Path: ' + ' '.join(path) + '\n')

    expanded_nodes, path, path_cost =  dfs(init_state, final_state)
    output_f.write('Number of expanded nodes: ' + str(expanded_nodes) + '\n')
    output_f.write('Path-cost: ' + str(path_cost) + '\n')
    output_f.write('Path: ' + ' '.join(path) + '\n')

    expanded_nodes, path, path_cost =  ucs(init_state, final_state)
    output_f.write('Number of expanded nodes: ' + str(expanded_nodes) + '\n')
    output_f.write('Path-cost: ' + str(path_cost) + '\n')
    output_f.write('Path: ' + ' '.join(path) + '\n')
    
    is_greedy = True
    expanded_nodes, path, path_cost =  greedy(init_state, final_state)
    output_f.write('Number of expanded nodes: ' + str(expanded_nodes) + '\n')
    output_f.write('Path-cost: ' + str(path_cost) + '\n')
    output_f.write('Path: ' + ' '.join(path) + '\n')
    is_greedy = False

    expanded_nodes, path, path_cost =  astar(init_state, final_state)
    output_f.write('Number of expanded nodes: ' + str(expanded_nodes) + '\n')
    output_f.write('Path-cost: ' + str(path_cost) + '\n')
    output_f.write('Path: ' + ' '.join(path))

    output_f.close()



if __name__ == "__main__":
    main()