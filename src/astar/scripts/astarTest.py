import tkinter as tk
import math
import time


class GUI:
    def __init__(self):
        self.root = tk.Tk()
        self.canvas = tk.Canvas(self.root, width=600, height=600, bg='black')
        self.canvas.pack()

        self.grid_size = 50  # Assuming 25m / 0.5m resolution

        self.start = Node(10, 10)
        self.target = Node(40, 40)

        self.grid = [[Node(x, y) for y in range(self.grid_size)] for x in range(self.grid_size)]
        self.obstacles = [(15, 15), (15, 16), (16, 15), (16, 16)]  # Example obstacles
        self.update_grid()

        self.robot = self.canvas.create_oval(0, 0, 0, 0, fill='blue', outline='black')
        self.target_visual = self.canvas.create_oval(0, 0, 0, 0, fill='red', outline='black')

        self.open_list = []   # Initialize open list
        self.closed_list = set()  # Initialize closed list
        self.path = []  # Initialize path

        self.update_gui()

    def update_gui(self):
        self.canvas.delete('all')

        for x in range(self.grid_size):
            for y in range(self.grid_size):
                node = self.grid[x][y]
                color = 'white'
                if node.obstacle:
                    color = 'black'
                elif node in self.open_list:
                    color = 'green'
                elif node in self.closed_list:
                    color = 'yellow'
                elif node in self.path:
                    color = 'blue'

                self.canvas.create_rectangle(
                    x * 12, y * 12, (x + 1) * 12, (y + 1) * 12,
                    fill=color, outline='black'
                )

        self.canvas.create_oval(
            self.start.x * 12, self.start.y * 12, self.start.x * 12 + 12, self.start.y * 12 + 12,
            fill='blue', outline='black'
        )

        self.canvas.create_oval(
            self.target.x * 12, self.target.y * 12, self.target.x * 12 + 12, self.target.y * 12 + 12,
            fill='red', outline='black'
        )

        self.root.update()
        time.sleep(0.1)  # Add a delay for visualization

    def update_grid(self):
        for y in range(self.grid_size):
            for x in range(self.grid_size):
                self.grid[x][y] = Node(x, y)
                if (x, y) in self.obstacles:
                    self.grid[x][y].obstacle = True

    def run(self):
        self.root.mainloop()

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.g_cost = float('inf')
        self.h_cost = 0
        self.parent = None
        self.obstacle = False

    def f_cost(self):
        return self.g_cost + self.h_cost


# Calculate Manhattan distance heuristic
def manhattan_distance(node, target):
    return abs(node.x - target.x) + abs(node.y - target.y)

def astar(start, target, grid_size, grid):
    open_list = [start]
    closed_list = set()

    while open_list:
        current = min(open_list, key=lambda node: node.f_cost())
        open_list.remove(current)
        closed_list.add(current)

        if current == target:
            return reconstruct_path(current)

        for neighbor in get_neighbors(current, grid_size, grid):
            if neighbor in closed_list or neighbor.obstacle:
                continue

            tentative_g_cost = current.g_cost + distance(current, neighbor)

            if tentative_g_cost < neighbor.g_cost:
                neighbor.parent = current
                neighbor.g_cost = tentative_g_cost
                neighbor.h_cost = manhattan_distance(neighbor, target)

                if neighbor not in open_list:
                    open_list.append(neighbor)

    return None  # No path found


def reconstruct_path(node):
    path = []
    while node:
        path.append((node.x, node.y))
        node = node.parent
    return path[::-1]  # Reverse the path

def get_neighbors(node, grid_size, grid):
    neighbors = []

    # Define valid directions (up, down, left, right)
    directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]

    for dx, dy in directions:
        x, y = node.x + dx, node.y + dy

        if 0 <= x < grid_size and 0 <= y < grid_size:
            neighbors.append(grid[x][y])

    return neighbors


# Calculate Euclidean distance between nodes
def distance(node1, node2):
    return math.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)

def main():
    gui = GUI()

    path = astar(gui.start, gui.target, gui.grid_size, gui.grid)
    print(path)
    if path:
        print("Path found:", path)
    else:
        print("No path found.")

    gui.run()

if __name__ == "__main__":
    main()