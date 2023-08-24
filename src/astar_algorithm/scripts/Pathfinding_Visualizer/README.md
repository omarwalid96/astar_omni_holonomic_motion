Author: Abhinav Balsubramanian

Date: April 20, 2021

Version: 1.23

Unit: 6 - Culminating

Programming Language: Python 3.9.1 64-bit

[This video will walkthrough the program and highlight all of its features](https://www.youtube.com/watch?v=wxNOco10RBg)

## Programming Description: 
This purpose of this program is visualize the process that the A* Pathfinding algorithm uses to determine the shortest path between two nodes. It will start by displaying the title screen and 
instructions on how to use the program. A general description of the pathfinding algorithm will
also be given with an option to learn more by reading another article. Then, the user will be
presented with a 40x60 grid where they can first change the position of the starting node. Then,
the user will be able to customize the ending node and then finally place walls along the grid 
area. A wall is a node that the algorithm can not consider when determining the shortest path.
Finally, the user will confirm their selection and then watch the program try to find the shortest
path. If there is a valid solution, the program will display the path by highlighting it in a 
different color. If no valid path is found, the program will stop and wait for the user to reset
the grid or exit the program. 

## Features of the Program: 
The program features a very in-depth tutorial that explains what a pathfinding algorithm is and how to use the program. Additionally, there is an option to generate random walls for the program to run through. Furthermore, the visualization is very engaging as it animates the process of finding the path and backtracking. Lastly, all the controls are very intuitive; 'W' is to generate random walls, 'C' to clear the grid. 

## Program Assumptions: 
This program assumes that the user has already installed the latest version of pygame and is running the program from the root directory (/pathfinding_visualizer). Additionally, the program assumes the user understands how the A* Pathfinding algorithm works. Lastly, fluency of English is another assumption because all the instructions are given in English.

## Restrictions: 
Once both nodes and walls have been placed, the user must wait until the program finds the 
path if possible before reseting the grid. The user also can not return to the tutorial after they go to the main screen. 

## Known Errors: 
If you try running the program through the terminal on an M1 Mac, you might face an
error where is says pygame is unable to be imported. To avoid this from happening, open the folder
in an IDE (preferably Visual Studio Code) and run the main.py file. 

## Implementation Details:
In order to run the visualizer, you must first clone or download this repository from Github. Then make sure to
install the latest version of pygame by entering 
``
pip install pygame
`` in the terminal. Finally, run the main.py file from the program's root directory (/pathfinding_visualizer).
