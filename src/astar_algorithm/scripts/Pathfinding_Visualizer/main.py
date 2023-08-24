# Abhinav Balasubramanian
# April 20, 2021
# ICS3UO-C
# This file will intitialize the pygame application, take user inputs, and perform the main logic of the pathfinding algorithm.
# It also will update the pygame screen to show the visualization.

import pygame # imports the pygame library
import math # imports the math library
import random # imports the random library
import sys # imports sys
from cell import Cell # imports the Cell class from cell.py

# this function displays the screen when there is no solution
def displayResultScreen():
    try: # this is to handle an exception caused by retrieving the image
        displayImage(pygame.image.load('./images/no_solution.png')) # passes the image to displayImage function
    except Exception:  # catches any error
        print("ERROR RETREIVING IMAGES. Make sure you are in the program's root directory (/pathfinding_visualizer) and all images are in the images folder") # prints error statement
        exitProgram()
    pygame.display.flip() # updates the pygame application
    pygame.time.wait(3000) # waits for 3 seconds before continuing the program

# this function will check if the user wants to quit the program
def checkForExit():
    for event in pygame.event.get(): # gets the user input
        if event.type == pygame.QUIT: # checks if the event type is quit
            exitProgram() # calls the exit program function

# this function exits the program
def exitProgram():
    try: # this is to handle an exception caused by retrieving the image
        displayImage(pygame.image.load('./images/exit_screen.jpg')) # displays the goodbye screen
        pygame.display.flip() # updates the pygame display
        pygame.time.wait(1500) # shows the screen for 1.5 seconds
    except Exception: # catches any error
        print("ERROR RETREIVING IMAGES. Make sure you are in the program's root directory (/pathfinding_visualizer) and all images are in the images folder") # prints error statement
    pygame.quit() # quits the application
    sys.exit() # exits the program

# this function will determine what cell the mouse is hovering over and then change the state of the cell
def selectWall(mouse_pos, new_state, end_node, start_node): # takes 4 variables as parameters
    x = mouse_pos[0] // 20 # determines the x index
    y = mouse_pos[1] // 20 # determines the y index
    if grid[x][y] != end_node and grid[x][y] != start_node: # ensures the current cell is not the start or end node
        grid[x][y].wall = new_state # changes the state of the cell

# this function will determine what cell the mouse is hovering over and then return the value
def selectStart(mouse_pos, start_node, end_node):
    x = mouse_pos[0] // 20 # determines the x index
    y = mouse_pos[1] // 20 # determines the y index
    # print(str(x) + " " + str(y)) # this print statement can be uncommented to see the x and y of the start cell
    if grid[x][y] != end_node: # ensures the cell is not the end node
        return grid[x][y] # returns the corresponding cell
    else:
        return start_node # returns the original start node

# this function will determine what cell the mouse is hovering over and then return the value
def selectEnd(mouse_pos, start_node , end_node): 
    x = mouse_pos[0] // 20 # determines the x index
    y = mouse_pos[1] // 20 # determines the y index
    if grid[x][y] != start_node: # ensures the cell is not the start node
        return grid[x][y] # returns the corresponding cell
    else:
        return end_node # returns the original end node

# this is the heurisitic function that the A* algorithm uses
def heuristic(current_node, end_node):
    # returns the absolute euclidean distance between the current and end node
    return math.sqrt((current_node.x_pos - end_node.x_pos)**2 + abs(current_node.y_pos - end_node.y_pos)**2)  

# this function will draw the grid and color each cell with the correct color
def drawGrid():
    screen.fill((127, 195, 180)) # fills the entire screen with a light green which will later be the color of the grid lines
    
    # this loop will cycle through all the cells in the grid
    for y in range(grid_rows): 
        for x in range(grid_col):
            
            cell = grid[x][y] 
            # print(cell.x_pos,cell.y_pos)
            cell.colorCell(screen, (244,250,250), "small square") # this is the color of a regular cell
            
            if cell == start: # checks if the cell is the start node
                cell.colorCell(screen, (0, 153, 0), "node")
            
            elif cell == end: # checks if the cell is the end node
                cell.colorCell(screen, (204, 0, 0), "node")
            
            elif cell in path: # checks if the cell is in the path list
                cell.colorCell(screen, (24, 90, 90), "small square")
                font = pygame.font.SysFont('arial', 12)
                text = font.render(str(round(cell.h_score)), True, (0, 0, 0))
                text_pos = (cell.x_pos * 20, cell.y_pos * 20)
                screen.blit(text, text.get_rect(center=text_pos))

            elif cell in open_list: # checks if the cell is in the open_list list
                cell.colorCell(screen, (255, 170, 70), "circle")
                font = pygame.font.SysFont('arial', 12)
                text = font.render(str(round(cell.f_score)), True, (0, 0, 0))
                text_pos = (cell.x_pos * 20, cell.y_pos * 20)
                screen.blit(text, text.get_rect(center=text_pos))
            
            elif cell in closed_list: # checks if the cell is in the closed_list list
                cell.colorCell(screen, (255, 170, 70), "small square")
            
            elif cell.wall: # checks if the cell is a wall
                cell.colorCell(screen, (127, 195, 180), "small square")

            
            # screen.blit(arialfont, text_rect)
                
    pygame.display.update() # updates the pygame display to show the new changes

# this function will backtrack to determine all the cells that are in the shortest path
def aStarBackTrack(current): # takes the current node as the only parameter
    print("Backtracking called.") # this print statement is used for debugging
    in_path = current # assigns the current to the path_node
    while True: # runs the loop until it reaches the start node
        checkForExit()
        if in_path.previous_node == None or in_path.previous_node == start: # checks whether there is no previous node or the current node is the start node
            break # exits the loop
        else:
            path.append(in_path.previous_node) # adds the previous node to the path list
            in_path = in_path.previous_node # assigns the previouse node to the in_path variable
            drawGrid()
    print("Done Backtracking") # this print statement should be uncommented when debugging the program

# this function implements the A* algorithm
def aStarSearch():
    
    start.h_score = heuristic(start, end) # determines the euclidean distance from the start node to end node
    open_list.append(start) # adds the start node to the open_list list   
    while len(open_list) > 0: # runs the loop until the open_list list is empty
        
        checkForExit()
        
        open_list.sort(key=lambda x: x.f_score) # this returns the open_list sorted by f_scores using a lambda function passed in the optional key parameter

        current_node = open_list[0] # the current node is assigned to the cell with the lowest f score in the open_list      
        
        if current_node == end: # checks if the current node is the end
            open_list.remove(current_node) # removes the current node from the open set
            aStarBackTrack(current_node) # passes the current node to the backtracking function
            return
        
        else:
            open_list.remove(current_node) # removes from open_list and adds to closed_list
            closed_list.append(current_node)

            for cells in current_node.neighbors: # loops through all the neighbors of a cell
                
                if cells in closed_list or cells.wall == True: # checks if the cell has already been looked at or is a wall
                    continue # goes back to the top of the loop
                
                else:

                    new_g_score = current_node.g_score + 1 # adds one to the g score
                    use_new_path = False # initializes the use_new_path as false
                    
                    if cells in open_list: # checks if the cell is in the open list
                        if new_g_score < cells.g_score: #  checks if the new g score is less than the current g score
                            cells.g_score = new_g_score # assigns the new g score to the cell
                            use_new_path = True # since the g score is lower than the original one, the algorithm will now use this new path to the cell
                    
                    else:
                        cells.g_score = new_g_score # assigns the new g score to the cell              
                        use_new_path = True # since this cell has not been visited yet it will use it as a new path
                        open_list.append(cells) # adds this cell to the open_list
                    
                    if use_new_path == True: # checks if the algorithm has to use the new path
                        cells.h_score = heuristic(cells, end) # determines the h score of the cell
                        cells.f_score = cells.g_score + cells.h_score # determines the f score of the cell
                        cells.previous_node = current_node # assigns the current node as the previous node to this cell
        
        drawGrid() # updates the screen
    pygame.time.wait(700) # wait 700 milliseconds before continuing 
    displayResultScreen() # this screen will only run when the open_list is empty and there are no other neighbors; this means there is no possible path between the start and end node

# this function will display the image on the screen
def displayImage(image): # takes an image as the parameter
    screen.blit(image, (0,0))
    pygame.display.flip()

# this function will display the correct tutorial page
def displayPages():
    try: 
        # this is a list of all the images in the order they are supposed to be shown
        tutorial_images = [pygame.image.load('./images/screen_1.jpg'), pygame.image.load('./images/screen_2.jpg'), pygame.image.load('./images/screen_3.png'), pygame.image.load('./images/screen_4.jpg'), 
        pygame.image.load('./images/screen_5.jpg'), pygame.image.load('./images/screen_6.jpg'), pygame.image.load('./images/screen_7.png')]
    except Exception:
        print("ERROR RETREIVING IMAGES. Make sure you are in the program's root directory (/pathfinding_visualizer) and all images are in the images folder")
        exitProgram()

    tutorial_index = 0 # initializes tutorial_index

    while tutorial_index < len(tutorial_images): # loops through all the items in the list
        displayImage(tutorial_images[tutorial_index]) # displays the current image to the screen
        
        # checks for user input and handles it
        for event in pygame.event.get():
            if event.type == pygame.QUIT: # checks if the user wants to quit the application
                exitProgram()
            if event.type == pygame.KEYDOWN: # checks if the user pressed a key
                if event.key == pygame.K_RIGHT: # checks if the key pressed is the right arrow
                    tutorial_index += 1 # increments the tutorial_index variable
                if event.key == pygame.K_LEFT: # checks if the key pressed is the left arrow
                    if tutorial_index != 0: # ensures tutorial_index is not already at 0
                        tutorial_index-= 1 # decrements the tutorial_index variable

# this program will clear all the walls from the grid
def clearWall():
    for x in range(grid_col): # loops through all the columns
        for y in range(grid_rows): # loops through all the rows
            grid[x][y].wall = False # sets the wall state to false for every cell

# this function will generate random walls in the grid
def generateRandomWalls():

    clearWall()

    for x in range(grid_col): # loops through all the columns
        for y in range(grid_rows): # loops through all the rows
            if grid[x][y] == start or grid[x][y] == end: # if the current cell is the start or end it will skip it
                continue # goes back to the top of the loop
            else:
                if random.randint(1, 1000) < 400: # generates a random integer and then checks if it is less than 400; 400 is a random number which can be changed to effect the probability that the cell will be a wall
                    grid[x][y].wall = True # makes the current cell a wall        
                else:
                    continue
        drawGrid() # updates the grid

# this function will initialize the grid that will be used for the logic and gui of the program
def initGrid():
    # adds all the cells to the grid
    for x in range(grid_col): # loops through every column
        row_list = [] # instantiates a new list
        for y in range(grid_rows): # loops through every row
            row_list.append(Cell(x, y)) # adds a cell for each row
        grid.append(row_list) # adds the row_list to the grid list
    
    # adds the neighbors of all the cells in the grid
    for x in range(grid_col):
        for y in range(grid_rows):
            grid[x][y].addNeighbors(grid, grid_col, grid_rows, use_diagonal)
            grid[x][y].wall = False

# initializes constant variables
grid = []
grid_rows = 40
grid_col = 60
side_length = 20
screen_length = grid_col * side_length
screen_width = grid_rows * side_length
use_diagonal = True
screen_size = (screen_length, screen_width)

# initializes the pygame application
pygame.init() 
f=pygame.font.get_fonts()
arialfont=pygame.font.SysFont("comicsansms", 12)

screen = pygame.display.set_mode(screen_size)
pygame.display.set_caption("Pathfinding Visualizer")

# displays the tutorial pages
displayPages()

# main program loop
while True: # this loop will run until the user wishes to quit the program

    initGrid()

    # initializes the positions of the start and end nodes
    start = grid[5][18]
    end = grid[54][18]
    
    # instantiates/resets all the lists
    open_list = []
    closed_list = []
    score_list = []
    path = []

    # initializes all the flag variables
    is_selecting_start = True
    is_selecting_end = False
    is_selecting_walls = False 
    start_search = False
    reset_game = False


    drawGrid() # draws the current grid
    pygame.display.flip() 

    # this loop will run until the user wishes to reset the grid
    while True:
        
        # checks if the user is selecting the start node
        if is_selecting_start == True:
            for event in pygame.event.get(): # gets the user input
                if event.type == pygame.QUIT: # checks if the user wants to quit the program
                    exitProgram()
                if event.type == pygame.MOUSEBUTTONDOWN: # checks if the user pressed a mouse button
                    if event.button in (1, 3): # checks if they pressed mouse button 1, 2 or 3
                        start = selectStart(pygame.mouse.get_pos(), start, end)
                if event.type == pygame.KEYDOWN: # checks if the user pressed a key
                    if event.key == pygame.K_RETURN: # checks if the key was return/enter
                        is_selecting_start = False 
                        is_selecting_end = True
                    if event.key == pygame.K_c: # checks if the key pressed was c
                        reset_game = True
        
        # checks if the user is selecting the end node
        if is_selecting_end == True: 
            for event in pygame.event.get(): # gets the user input
                if event.type == pygame.QUIT: # checks if the user wants to quit the program
                    exitProgram()
                if event.type == pygame.MOUSEBUTTONDOWN:  # checks if the user pressed a mouse button
                    if event.button in (1, 3): # checks if they pressed mouse button 1, 2 or 3
                        end = selectEnd(pygame.mouse.get_pos(), start, end)
                if event.type == pygame.KEYDOWN: # checks if the user pressed a key
                    if event.key == pygame.K_RETURN: # checks if the key was return/enter
                        is_selecting_end = False
                        is_selecting_walls = True
                    if event.key == pygame.K_c: # checks if the key pressed was c
                        reset_game = True

        # checks if the user is selecting walls
        if is_selecting_walls == True:
            for event in pygame.event.get(): # gets the user input
                if event.type == pygame.QUIT: # checks if the user wants to quit the program
                    exitProgram()
                if event.type == pygame.MOUSEBUTTONDOWN: # checks if the user pressed a mouse button  
                    if event.button in (1, 3): # checks if they pressed mouse button 1, 2 or 3
                        selectWall(pygame.mouse.get_pos(), event.button==1, end, start)
                if event.type == pygame.MOUSEMOTION: # checks if the user moved their mouse
                    if event.buttons[0] or event.buttons[2]:
                        selectWall(pygame.mouse.get_pos(), event.buttons[0], end, start) 
                if event.type == pygame.KEYDOWN: # checks if the user pressed a key
                    if event.key == pygame.K_RETURN: # checks if the key was return/enter
                        start_search = True
                        is_selecting_walls = False
                    if event.key == pygame.K_c: # checks if the key pressed was c
                        reset_game = True
                    if event.key == pygame.K_w: # checks if the key pressed was w
                        generateRandomWalls()
                    if event.key == pygame.K_BACKSPACE: # checks if the key pressed was backspace
                        clearWall()

        # checks if the program should start the search
        if start_search == True: 
            aStarSearch()
            start_search = False

        # checks if the user wants to quit the program or clear the grid
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                exitProgram()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_c:
                    reset_game = True
        
        if reset_game == True:
            break

        drawGrid() # draws the current grid
        pygame.display.flip() # updates the screen