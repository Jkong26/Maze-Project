
import sys
import heapq
from PIL import Image, ImageDraw

class Node():
    def __init__(self, state, parent, action, cost_from_start=0):
        """
        state: tuple (row, col)
        parent: previous node
        action: move direction from parent to this state
        cost_from_start: g(n) - cumulative cost from start node to this node
        """
        self.state = state
        self.parent = parent
        self.action = action
        self.cost_from_start = cost_from_start  # g(n): total steps taken from the start to reach this node

# StackFrontier class is used for Iterative Deepening Search (IDS)
# IDS repeatedly runs Depth-Limited DFS, and DFS uses a stack (LIFO).
class StackFrontier():
    # Constructor: initializes an empty list to store nodes
    def __init__(self):
        self.frontier = []  # List to store nodes (acts like a stack)

     # Add a node to the frontier (top of the stack)
    def add(self, node): #Adds a node to the frontier.
        self.frontier.append(node) # Append node to the end of the list

    # Check if a state (position) is already in the frontier
    def contains_state(self, state): 
        return any(node.state == state for node in self.frontier) 
     # Returns True if any node in the stack has this state

    # Check if the frontier is empty
    def empty(self):
        return len(self.frontier) == 0 # True if no nodes are left

    # Remove and return the last node added (LIFO order)
    def remove(self):
        
        # If the stack is empty, raise an error
        if self.empty():
            raise Exception("empty frontier")
    

        node = self.frontier[-1] # Get the last node
        self.frontier = self.frontier[:-1]  # Remove last node from the list
        return node # Return the removed node

# Implements a priority queue frontier for A* search
class PriorityFrontier:
    def __init__(self):
        self.elements = [] #a list that will store tuples of the form (priority, counter, node).
        self.counter = 0 # used to break ties when two nodes have the same priority. 
                        #This prevents errors in Python’s heapq

    def add(self, node, priority):
        heapq.heappush(self.elements, (priority, self.counter, node)) # push node to heap by priority
        self.counter += 1  # increment counter for next node

    # Check if there are no nodes left in the frontier
    def empty(self):
        return len(self.elements) == 0 # True if no nodes are left

    # Remove and return the node with the lowest priority
    def remove(self): 
        if self.empty(): #if it's empty raise empty frontier
            raise Exception("empty frontier")
        # Pop the node with the smallest priority from the heap
        return heapq.heappop(self.elements)[2] # Return the elements

    # Check if a specific position is already in the frontier
    def contains_state(self, state):
        # Return True if any node in the frontier has this state already
        return any(node.state == state for _, _, node in self.elements) 

# Maze object that handles loading, solving, and visualizing the maze
class Maze:
    def __init__(self, filename):
        # Load maze from file
        with open(filename) as f:
            contents = f.read()

        # Make sure the maze has exactly one start "A" and one goal "B"
        if contents.count("A") != 1 or contents.count("B") != 1:
            raise Exception("Maze must have exactly one start point (A) and one goal point (B)")

        # Split file into lines for rows
        contents = contents.splitlines()
        self.height = len(contents) # Number of rows
        self.width = max(len(line) for line in contents) # Maximum number of columns

        # Parse the maze into a grid of walls (True = wall, False = open space)
        self.walls = []
        for i in range(self.height):
            row = []
            for j in range(self.width):
                try:
                    if contents[i][j] == "A": 
                        self.start = (i, j) # Save start position
                        row.append(False)   # Not a wall
                    elif contents[i][j] == "B":
                        self.goal = (i, j)  # Save goal position
                        row.append(False)   # Not a wall
                    elif contents[i][j] == " ":
                        row.append(False)   # Empty space
                    else:
                        row.append(True) # wall
                except IndexError:
                    row.append(False)       # Fill missing cells as empty
            self.walls.append(row)          # Add row to walls grid

        self.solution = None # Will store final solution path later

    # Heuristic: Manhattan distance from current state to goal (used for A*)
    def manhattan_distance(self, state):
        # state is a tuple (row, col) representing current position
        r1, c1 = state # r1 = row of current position, c1 = column of current position
        
        # Get the goal position
        r2, c2 = self.goal # r2 = row of goal, c2 = column of goal

        # Manhattan distance = |row difference| + |column difference|
        # This is how many steps it would take to reach the goal if we can only move up/down/left/right
        return abs(r1 - r2) + abs(c1 - c2)

    # Reconstruct the path from the goal back to the start by following parent nodes
    def get_path(self, node):
        actions = [] # List to store the moves taken (e.g., "UP", "DOWN")
        cells = [] # List to store positions (row, col) along the path

        # Keep going back from the current node to the start
        while node.parent is not None:
            actions.append(node.action) # Record the move that led to this node
            cells.append(node.state) # Record the current position
            node = node.parent # Step back to the parent node
        
        # Currently, the path is from goal → start, so reverse it to get start → goal
        actions.reverse() # Reverse the list of moves
        cells.reverse() # Reverse the list of positions

        # Return two lists: actions (moves) and cells (positions)
        return actions, cells

    # Solve maze using Iterative Deepening Search (IDS)
    def solve_ids(self):
        self.num_explored = 0 # Count how many nodes we have explored
        max_depth = 0 # Start searching with depth limit 0
        self.explored = set() # Keep track of explored positions

        # Keep increasing depth until we find a solution
        while True:
            # Perform depth-limited search with current max_depth
            result = self.depth_limited_search(max_depth)

            # If a solution is found, reconstruct the path and stop
            if result is not None:
                self.solution = self.get_path(result)
                return
            # Increase depth for the next iteration        
            max_depth += 1
            # Safety check: stop if max_depth is bigger than total maze cells
            if max_depth > self.height * self.width: # upper limit to avoid infinite loop
                break
        # If no solution is found after all iterations
        print("no solution")

    # Depth-limited search used in IDS
    def depth_limited_search(self, limit):
        # Create the start node with no parent and no action
        start = Node(state=self.start, parent=None, action=None)

        # Frontier is a stack of (node, current depth)
        frontier = [(start, 0)] # stack of (node, current depth)

        # Keep track of positions we have already explored in this search
        local_explored = set()
        # Continue while there are nodes in the frontier
        while frontier:
            node, depth = frontier.pop() # Take the last node added (LIFO) and its depth
            self.num_explored += 1 # Count this node as explored
            local_explored.add(node.state) # Add this node's position to the explored set

            if node.state == self.goal:  # If we reached the goal, return this node
                self.explored = local_explored # Save all explored positions
                return node
            # Only expand nodes if we are below the depth limit
            if depth < limit:
                # Check all neighboring positions (up, down, left, right)
                for action, state in self.neighbors(node.state):
                    # Only add neighbors to state that were not already explored
                    if state not in local_explored:
                        # Create a child node for the neighbor
                        child = Node(state=state, parent=node, action=action)
                        # Add the child to the frontier with increased depth
                        frontier.append((child, depth + 1))
    # If goal not found within depth limit; save explored positions and return None
        self.explored = local_explored
        return None

   # Solve maze using A* Search (optimized)
    def solve_a_star(self):
        self.num_explored = 0  # Count how many nodes we have explored
        start = Node(state=self.start, parent=None, action=None, cost_from_start=0)

        # Priority queue (min-heap) frontier
        frontier = PriorityFrontier()
        frontier.add(start, self.manhattan_distance(self.start))  # f = g + h = 0 + h(start)

        self.explored = set()  # Keep track of explored positions

        while True:
            if frontier.empty():  # If no nodes left or frontier empty, then no solution exists
                print("no solution")
                return

            node = frontier.remove()  # Node with lowest f(n)
            self.num_explored += 1 # Increment explored node counter

            if node.state == self.goal: # If goal is reached, reconstruct path
                self.solution = self.get_path(node)
                return

            self.explored.add(node.state) # Mark current node as explored

            for action, state in self.neighbors(node.state): # Expand neighbors of current node
                if not frontier.contains_state(state) and state not in self.explored:
                    # g(n) = parent's cost + 1 (uniform move cost)
                    child = Node(state=state, parent=node, action=action, cost_from_start=node.cost_from_start + 1)
                    # f(n) = g(n) + h(n) where h(n) = Manhattan distance
                    priority = child.cost_from_start + self.manhattan_distance(state) 
                    frontier.add(child, priority) # Add child node to frontier with computed priority


    # Get neighboring cells that are not walls
    def neighbors(self, state):
         # Unpack the current position
        row, col = state


    # List all possible moves from current position
    # "up" = move one row up, "down" = move one row down
    # "left" = move one column left, "right" = move one column right
        candidates = [("up", (row - 1, col)), ("down", (row + 1, col)), ("left", (row, col - 1)), ("right", (row, col + 1))]
        
        # List to store valid neighbors (inside maze and not a wall)
        result = []

        # Check each candidate move
        for action, (r, c) in candidates:

            # If the move is inside the maze and not a wall
            if 0 <= r < self.height and 0 <= c < self.width and not self.walls[r][c]:
                result.append((action, (r, c))) # Add it to the result

        # Return the list of valid moves        
        return result

    # Print the maze with optional solution path
    def print(self):
        # Get the solution path (list of positions), if a solution exists
        solution = self.solution[1] if self.solution is not None else None
        print() # Print an empty line for spacing

        # Go through each row in the maze
        for i, row in enumerate(self.walls):

            # Go through each column in the row
            for j, col in enumerate(row):
                if col:
                    print("█", end="") # wall
                elif (i, j) == self.start:
                    print("A", end="") #Start position
                elif (i, j) == self.goal:
                    print("B", end="") # Goal position
                elif solution is not None and (i, j) in solution:
                    print("*", end="") # Part of the solution path
                else:
                    print(" ", end="") # Empty space / open path
            print() # Go to next line after finishing a row
        print() # Print an extra empty line at the end

    # Create a visual PNG image of the maze with optional path and explored cells
    def output_image(self, filename, show_solution=True, show_explored=False):
        cell_size = 50 # Each cell in the image is 50x50 pixels
        cell_border = 2  # Small border between cells

        # Create a blank image with black background
        img = Image.new("RGBA", (self.width * cell_size, self.height * cell_size), "black")
        draw = ImageDraw.Draw(img) # Object used to draw on the image

        # Get solution path (list of positions), if it exists
        solution = self.solution[1] if self.solution is not None else None

        # Loop through each row and column of the maze
        for i, row in enumerate(self.walls):
            for j, col in enumerate(row):
                if col:
                    fill = (40, 40, 40) # wall
                elif (i, j) == self.start:
                    fill = (255, 0, 0) # red for start
                elif (i, j) == self.goal:
                    fill = (0, 171, 28)  # green for goal
                elif solution is not None and show_solution and (i, j) in solution:
                    fill = (220, 235, 113) # yellow for path
                elif solution is not None and show_explored and (i, j) in self.explored:
                    fill = (212, 97, 85) # red for explored cells
                else:
                    fill = (237, 240, 252) # default empty space

                # Draw a rectangle for this cell on the image
                draw.rectangle([(j * cell_size + cell_border, i * cell_size + cell_border),
                                ((j + 1) * cell_size - cell_border, (i + 1) * cell_size - cell_border)], fill=fill)
        
        # Save the image as a PNG file
        img.save(filename)

# Command-line interface
if len(sys.argv) != 3:
    sys.exit("Usage: python maze.py maze.txt [ids|astar]")

m = Maze(sys.argv[1])
print("Maze:")
m.print()
print("Solving...")

# Choose algorithm based on argument
if sys.argv[2] == "ids":
    m.solve_ids()
elif sys.argv[2] == "astar":
    m.solve_a_star()
else:
    sys.exit("Unknown algorithm type. Use 'ids' or 'astar'.")

# Output results
if m.solution:
    path, cells = m.solution
    move_map = {"up": "U", "down": "D", "left": "L", "right": "R"}
    move_seq = "-".join([move_map[m] for m in path])
    print("Solution Path:", move_seq)
    print("Total Path Cost:", len(path))
    print("Path Length:", len(path))
else:
    print("no solution")

print("States Explored:", m.num_explored)
print("Maze with path:")
m.print()
m.output_image("maze.png", show_explored=True)
