import matplotlib.pyplot as plt
from collections import deque
import random
import time
from queue import PriorityQueue
import numpy as np
from numpy.linalg import norm
import copy
import sys
import math

start_time = time.time()

dim = 10 #Dimensions of the maze
p = 0.3 #Probability of cell being occupied

fire_coordinates = [] # place where fire coordinates will be stored
path_coordinates = [] # place where path coordinates will be stored
parent_fire = [] # place where all the parent fire coordinates will be stored

PATH_VALUE = 0.9
OBSTACLE_VALUE = 1
FIRE_VALUE = 0.3
START_AND_END_POINT_VALUE = 0.2
BFS_VISITED_VALUE = 0.5

Q_PROBABILITY_VALUE = 0.2 # probability of fire spreading


class Node:
    # (x, y) represents coordinates of a cell in matrix
    # maintain a parent node for printing path
    def __init__(self, x, y, parent, dist):
        self.x = x # x coordinate
        self.y = y # y coordinate
        self.parent = parent # the parent coordinate of the current coordinate
        self.dist = dist # the nearest distance from the current coorindate to fire

    def __repr__(self):
        # used to print 
        return str((self.x, self.y, self.dist))

#Generating Maze by inputting values randomly
def maze_generator(dim, p):
    maze = []
    for i in range(dim):
        maze.append([])
        for j in range(dim):
            value = random.random() <= p # can either be 0 or 1
            maze[i].append(value)
    maze[0][0] = START_AND_END_POINT_VALUE #Start Point
    maze[-1][-1] = START_AND_END_POINT_VALUE # End Point
    return maze

maze = maze_generator(dim,p)

def generate_fire(q, maze):
    fire_coordinates.clear()
    temp = maze[:]
    fire = 0
    # go throughout the entire maze and check if there are coordinates on fire
    for row in range(dim):
        for col in range(dim):
           # a coordinate can become on fire only if it is not an obstacle or not already on fire
            if maze[row][col] != FIRE_VALUE and maze[row][col] != OBSTACLE_VALUE:
                # find all the children of this coordinate that are on fire
                k = generate_fire_children(row, col)
                
                prob = 1 - pow((1-q), k)
                
                check = random.random() <= prob
                # set the coordinate on fire if true
                if check == True:
                    fire = fire+1
                    temp[row][col] = FIRE_VALUE
                    fire_coordinates.append((row,col))
    return temp

def generate_fire_children(i, j):
    
    k = 0
    row_direction = [-1, 1, 0, 0] #(north, south, east, west) {check the row above, check the row below, 0, 0}
    col_direction = [0, 0, 1, -1] # {0, 0, Check the column to the right, check the column to the left}
    for d in range(0,4): # loop through every possible coordinate
        row = i + row_direction[d]
        col = j + col_direction[d]

        if row in range(0, dim): # check if the row coordinate is out of bounds
            if(col in range(0,dim)): # check if the col coordinate is out of bounds
                if(maze[row][col] == FIRE_VALUE): # Check if the (row, col) coordinate is on fire
                    k = k+1
    return k

def initialize_fire(maze):
    maze[0][0] = START_AND_END_POINT_VALUE #Start Point
    maze[-1][-1] = START_AND_END_POINT_VALUE # End Point
    while True: # generate random starting point for the fire
        row = random.randint(0,dim-1) # generate random row coordinate 
        col = random.randint(0,dim-1) # generate random col coordinate
        if maze[row][col] != OBSTACLE_VALUE and maze[row][col] != START_AND_END_POINT_VALUE: # do not generate fire if it is an obstacle or a start or goal state
            maze[row][col] = FIRE_VALUE
            fire_coordinates.append((row,col))
            break
    return maze

def generate_path(node, maze, count):
    
    # end the recursion once all the nodes have been visited
    if node.parent.parent is None:
        
        maze[node.x][node.y] = PATH_VALUE
        path_coordinates.append(node)
        return node


    
    count = count+1
    # recurse through node and generate a path
    length = generate_path(node.parent, maze, count)

    if maze[node.x][node.y] == FIRE_VALUE:
        # if there are any points in the path that are already on fire, abort.
        print("We have hit the fire. Abort")
        sys.exit(0)

    maze[node.x][node.y] = PATH_VALUE
    path_coordinates.append(node)
    
    return length

def strategy_three(start, goal, maze):
    regen_path = False
    count = 0

    
    while (start.x, start.y) != goal:

        # append all the previous fire coordinates to the parent array
        if fire_coordinates:
            for f in fire_coordinates:
                parent_fire.append(f)
        
        # generate fire for each step in the path
        maze = generate_fire(Q_PROBABILITY_VALUE, maze)

        # If the heuristic is less than or equal to zero, it means that a fire is very close by. The path has to be regenerated
        if start.dist <= 0:
            regen_path = True

        if regen_path == True:
            tempMaze = copy.deepcopy(maze)
            path_coordinates.clear()
            bfs_check = bfs_search_regular((start.x, start.y), goal, tempMaze, 1)
            if bfs_check == False:
                print("No path available.")
                sys.exit(0)

            # generate all the distances from each point in the path to the closest fire
            fire_to_path_dist_generator()
            # set start to the first coordinate in the path
            start = path_coordinates[0]
            
            # mark the path as visited
            maze[start.x][start.y] = PATH_VALUE
            regen_path = False
            count = 0
        else:
            # if the path does not have to be regerated, simply subtract all the distances between the coordinate and fire by one and go to the next spot in the path.
            maze[start.x][start.y] = PATH_VALUE           
            for coor in path_coordinates:
                coor.dist = coor.dist-1

            count = count+1
            start = path_coordinates[count]

    return True

def fire_to_path_dist_generator():

    isEmpty = False
    if fire_coordinates == []:
        isEmpty = True
    
    answer = sys.maxsize
    # if the fire has spread at the current iteration, use the fire_coordinates array
    if isEmpty == False:
        # find the euclidean distance of each coordinate in the path to the closest fire
        for coor in path_coordinates:
            for f in fire_coordinates:
                temp = euclidean_distance(coor.x, coor.y, f)
                # round the distance down
                temp = math.trunc(temp)
                if temp < answer:
                        answer = temp
            # if there is no distance currently stored, simply store the distance found
            if coor.dist is None:
                coor.dist = answer
            # do not store the distance if the current distance is less than or equal to zero
            elif coor.dist <= 0:
                continue
            else:
                # store the distance found
                coor.dist = answer
    # since no fire has been generated at the current iteration, use the parent_fire array
    else:
        for coor in path_coordinates:
            for f in parent_fire:
                temp = euclidean_distance(coor.x, coor.y, f)
                temp = math.trunc(temp)
                if temp < answer:
                        answer = temp
            if coor.dist is None:
                coor.dist = answer
            elif coor.dist <= 0:
                continue
            else:
                coor.dist = answer

def euclidean_distance(row, col, coor):
    # calculates the euclidean distance between two vectors
    a = np.array([row, col])
    b = np.array([coor[0], coor[1]])

    #sqrt(cols^2 + rows^2)
    return norm(a-b)


def bfs_search_regular(start, goal, maze, check):
    # copy the maze so that the path can be maintianed
    tempMaze = copy.deepcopy(maze)
    # initialize fringe queue and closed set array
    fringe = deque()
    closed_set = []
    # initialize the first coordinate and append it to the fringe
    src = Node(start[0], start[1], None, None)
    fringe.append(src)

    while fringe:
        current = fringe.popleft()
        
        # there is a path for this maze
        if (current.x, current.y) == goal:
            # initialize fire ONLY if it is the first iteration of bfs
            if check == 0:
                maze = initialize_fire(maze)

            # generate the path found by bfs
            node = generate_path(current, maze, 1)
            
            return node
        else:
            # check if current has already been visited
            if tempMaze[current.x][current.y] != BFS_VISITED_VALUE:
                generate_valid_children_B_regular(current, fringe, tempMaze)
                # mark the coordinate as visited
                tempMaze[current.x][current.y] = BFS_VISITED_VALUE
                closed_set.append(current)
    return False

def generate_valid_children_B_regular(current, fringe, tempMaze):
    row_direction = [-1, 1, 0, 0] #(north, south, east, west) {check the row above, check the row below, 0, 0}
    col_direction = [0, 0, 1, -1]# {0, 0, Check the column to the right, check the column to the left}

    # loop through every possible coordinate
    for i in range(0,4):
        row = current.x + row_direction[i]
        col = current.y + col_direction[i]

        # if the row coordinate is out of bounds, it cannot be added to the fringe
        if row in range(0, dim):
            # if the col coordinate is out of bounds, it cannot be added to the fringe
            if(col in range(0,dim)):
                # Check if the (row, col) coordinate is a obstacle
                if(tempMaze[row][col] != OBSTACLE_VALUE and tempMaze[row][col] != FIRE_VALUE):
                    # Add to the fringe if it is a valid coordinate and not an obstacle and fire
                    if(tempMaze[row][col] != BFS_VISITED_VALUE):
                        # Check if the coordinates are already visited
                        fringe.append(Node(row, col, current, None))
                        

start = (0,0)
goal = (dim-1, dim-1)
node = bfs_search_regular(start, goal, maze, 0)

if(node != False):
    # check to see if the initialized fire occupys the first point in the path.
    if((node.x, node.y) == fire_coordinates[0]):
        print("We have hit fire...")
        sys.exit(0)
    fire_to_path_dist_generator()
    print(strategy_three(path_coordinates[0], goal, maze))
else:
    print(node)

#creating the size of the figure as well as the size of each individual square in the grid
Grid = plt.figure(figsize=(dim,dim)).add_subplot(224) 
Grid.set_title('Welcome to the Maze Of Fire')

#Display the data as an image
Image = plt.imshow(maze)

#Colormap of the generated grid ranging from yellow to blue
#Yellow equals free space (0)
#Dark Blue equals unavailable Space(1)
Image.set_cmap('YlGnBu')

print("My program took", time.time() - start_time, "to run")
