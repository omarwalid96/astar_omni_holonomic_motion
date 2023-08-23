import tkinter as tk
import math

class GUI:
    def __init__(self):
        self.root = tk.Tk()
        self.canvas = tk.Canvas(self.root, width=505, height=505,bg='black')
        self.canvas.pack()
        self.canvas.create_rectangle(5, 5, 500, 500, fill='white')
        self.robot = self.canvas.create_oval(0, 0, 0, 0, fill='blue', outline='black')  # Robot representation
        self.target = self.canvas.create_oval(0, 0, 0, 0, fill='red', outline='black')  # Target representation

        self.robot_x = 100  # Initial robot x-coordinate
        self.robot_y = 100  # Initial robot y-coordinate
        self.target_x = 300  # Initial target x-coordinate
        self.target_y = 300  # Initial target y-coordinate

        self.update_gui()

    def update_gui(self):
        # Update robot and target positions on the canvas
        self.canvas.coords(self.robot, self.robot_x - 10, self.robot_y - 10, self.robot_x + 10, self.robot_y + 10)
        self.canvas.coords(self.target, self.target_x - 5, self.target_y - 5, self.target_x + 5, self.target_y + 5)

        # Calculate the distance between robot and target
        distance = math.sqrt((self.target_x - self.robot_x) ** 2 + (self.target_y - self.robot_y) ** 2)

        # Move robot towards the target if they are not too close
        if distance > 1:
            angle = math.atan2(self.target_y - self.robot_y, self.target_x - self.robot_x)
            self.robot_x += math.cos(angle) * 2
            self.robot_y += math.sin(angle) * 2

        self.root.after(100, self.update_gui)  # Update the GUI every 100 milliseconds

    def run(self):
        self.root.mainloop()

if __name__ == "__main__":
    gui = GUI()
    gui.run()
