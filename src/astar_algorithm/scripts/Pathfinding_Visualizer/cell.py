# Abhinav Balasubramanian
# April 20, 2021
# ICS3UO-C
# This file has the cell class which will be called in the main program
# It also contains a function which will draw the cell on the pygame screen

import pygame

# this class will allow each cell to have its own properties that can be modified
class Cell:
    
    def __init__(self, x_pos, y_pos): # takes the x and y position as parameters
        
        # initializes all the variables used
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.neighbors = []
        self.previous_node = None
        self.wall = False
        self.f_score = float(0)
        self.g_score = float(0)
        self.h_score = float(0)

    # this function adds all the neighbors for each cell
    def addNeighbors(self, grid, column, row, count_diagonal):

        # checks if the x position is greater than 0 and if so adds the cell to the left as a neighbor
        if self.x_pos > 0:
            self.neighbors.append(grid[self.x_pos - 1][self.y_pos])

        # checks if the x position is 1 less than the number of columns and if so adds the cell to the right as a neighbor
        if self.x_pos < column - 1:
            self.neighbors.append(grid[self.x_pos + 1][self.y_pos])

        # checks if the y position is greater than 0 and if so adds the cell below as a neighbor
        if self.y_pos > 0:
            self.neighbors.append(grid[self.x_pos][self.y_pos - 1])
        
        # checks if the y position is 1 less than the number of rows and if so adds the cell above as a neighbor
        if self.y_pos < row - 1:
            self.neighbors.append(grid[self.x_pos][self.y_pos + 1])       
        
        # this extra feature will add all the diagonals as neighbors; follows the same logic in determining whether to add as a neighbor or not
        if count_diagonal == True:
            
            if self.x_pos > 0 and self.y_pos < row - 1:
                self.neighbors.append(grid[self.x_pos - 1][self.y_pos + 1])

            if self.x_pos > 0 and self.y_pos > 0:
                self.neighbors.append(grid[self.x_pos - 1][self.y_pos - 1])

            if self.x_pos < column - 1 and self.y_pos < row - 1:
                self.neighbors.append(grid[self.x_pos + 1][self.y_pos + 1])

            if self.x_pos < column - 1 and self.y_pos > 0:
                self.neighbors.append(grid[self.x_pos + 1][self.y_pos - 1])

    # this function will set the color of the cell and its shape
    def colorCell(self, win, color, shape):
        side = 20

        # checks if it is supposed to draw a node
        if shape == "node":
            pygame.draw.circle(win, color, (self.x_pos*side+9, self.y_pos*side+9), side//3) # draws the node in the correct location

        # checks if it is supposed to draw a square
        elif shape == "small square":
            pygame.draw.rect(win, color, (self.x_pos * side, self.y_pos * side, side - 2, side - 2)) # draws the square in the correct location

        # checks if it is supposed to draw a circle
        elif shape == "circle":
            pygame.draw.circle(win, color, (self.x_pos*side+9, self.y_pos*side+9), side//6) # draws the circle in the correct location

