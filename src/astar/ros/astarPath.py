import math # imports the math library
import random # imports the random library
import sys # imports sys
from cell import Cell # imports the Cell class from cell.py
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import rospy
import time
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Header

# initializes constant variables
grid = []
width=25
height=25
gridResolution=0.05
grid_rows = int(width/gridResolution)
grid_col = int(height/gridResolution)
use_diagonal = True

robotPositionX=0
robotPositionY=0

goalPositionX=15
goalPositionY=15


# initializes the positions of the start and end nodes
start = 0
end = 0
# instantiates/resets all the lists
open_list = []
closed_list = []
score_list = []
path = []

rospy.init_node('astar')
path_publisher = rospy.Publisher('/path', Path, queue_size=10)
marker_pub = rospy.Publisher('/goal_marker', Marker, queue_size=1)

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

# this is the heurisitic function that the A* algorithm uses
def heuristic(current_node, end_node):
    # returns the absolute euclidean distance between the current and end node
    return math.sqrt((current_node.x_pos - end_node.x_pos)**2 + abs(current_node.y_pos - end_node.y_pos)**2)  


# this function will backtrack to determine all the cells that are in the shortest path
def aStarBackTrack(current): # takes the current node as the only parameter
    global start,end,open_list,closed_list,score_list,path
    print("Backtracking called.") # this print statement is used for debugging
    in_path = current # assigns the current to the path_node
    while True: # runs the loop until it reaches the start node
        if in_path.previous_node == None or in_path.previous_node == start: # checks whether there is no previous node or the current node is the start node
            break # exits the loop
        else:
            path.append(in_path.previous_node) # adds the previous node to the path list
            in_path = in_path.previous_node # assigns the previouse node to the in_path variable
    print("Done Backtracking") # this print statement should be uncommented when debugging the program
    path.reverse()

    p=Path()
    p.header.seq=1
    p.header.frame_id="map"


    for i in path:
        currentP=PoseStamped()
        currentP.header.frame_id="map"
        currentP.pose.position.x=i.x_pos*gridResolution
        currentP.pose.position.y=i.y_pos*gridResolution
        currentP.pose.orientation.w=1.0
        print(i.x_pos*gridResolution,i.y_pos*gridResolution)
        p.poses.append(currentP)
    print(p)
    path_publisher.publish(p)
    # initializes the positions of the start and end nodes
    start = 0
    end = 0
    # instantiates/resets all the lists
    open_list = []
    closed_list = []
    score_list = []
    path = []

# this function implements the A* algorithm
def aStarSearch():
    
    start.h_score = heuristic(start, end) # determines the euclidean distance from the start node to end node
    open_list.append(start) # adds the start node to the open_list list   
    while len(open_list) > 0: # runs the loop until the open_list list is empty
        
        
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
        
def goal_callback(msg):
    global end,grid
    goal_x = msg.pose.position.x
    goal_y = msg.pose.position.y
    print(goal_x,goal_y)
    end = grid[int(goal_x/gridResolution)][int(goal_y/gridResolution)]
    publish_goal_marker(Point(goal_x,goal_y,0.0))
    aStarSearch()

def odom_callback(msg):
    global start,grid
    pos_x = msg.pose.pose.position.x
    pos_y = msg.pose.pose.position.y
    # print(pos_x,pos_y)
    # print(len(grid),int(pos_x/gridResolution),int(pos_y/gridResolution))
    start = grid[int(pos_x/gridResolution)][int(pos_y/gridResolution)]
def publish_goal_marker(point):

    # Create a goal marker (2D circle)
    goal_marker = Marker()
    goal_marker.header = Header()
    goal_marker.header.frame_id = 'map'
    goal_marker.id = 0
    goal_marker.type = Marker.CYLINDER
    goal_marker.action = Marker.ADD
    goal_marker.pose.position = point  # Change this to your desired goal position
    goal_marker.pose.orientation.w = 1.0
    goal_marker.scale.x = 0.5  # Diameter of the circle
    goal_marker.scale.y = 0.5  # Diameter of the circle
    goal_marker.scale.z = 0.001  # Very small value for a 2D circle

    goal_marker.color.a = 1.0
    goal_marker.color.r = 0.0
    goal_marker.color.g = 1.0
    goal_marker.color.b = 0.0

    goal_marker.header.stamp = rospy.Time.now()
    marker_pub.publish(goal_marker)

if __name__ == '__main__':
    
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    time.sleep(1)
    initGrid()
    rate = rospy.Rate(10)  # 10 Hz
        
    while not rospy.is_shutdown():
        rate.sleep()
    
    