# Abhinav Balasubramanian
# April 20, 2021
# ICS3UO-C
# This file has the cell class which will be called in the main program
# It also contains a function which will draw the cell on the pygame screen

# this class will allow each cell to have its own properties that can be modified
class Cell:
    
    def __init__(self, x_pos, y_pos): # takes the x and y position as parameters
        """!__init__ Initalize Cell

        @param x_pos: cell position in x @type x_pos: int
        @param y_pos: cell position in y @type y_pos: int
        """
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
        """!addNeighbors 
        Discovers surrounding neighbours to current cell

        @param grid: full grid @type grid: list[int][int]
        @param column: position x @type column: int
        @param row: position y @type row: int
        @param count_diagonal: turn on or off diagonal search @type count_diagonal: bool
        """
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
