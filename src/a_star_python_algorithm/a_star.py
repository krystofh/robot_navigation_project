#!/usr/bin/env python
#Created on Tue Dec  3 18:19:54 2019
#@author: Krystof

import cv2

class Node():
    # A node class for A* Pathfinding

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

# Closed lists search
def isInClosedList(node, nodesList):
    found = 0
    for nodeInList in nodesList:
        if node==nodeInList:
            found = 1
    return found

# Open list search
def isInOpenList(node, nodesList):
    foundIndex = -1
    for i in range(0, len(nodesList)):
        if node==nodesList[i]:
            foundIndex = i
    return foundIndex    


def astar(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    while len(open_list) > 0:
        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] == 1:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the closed list
            if (isInClosedList(child, closed_list)):
                continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            index = isInOpenList(child, open_list);
            if(index >= 0) and child.f > open_list[index].f: # only a worse path found
                continue

            # Add the child to the open list
            open_list.append(child)


def main():
# =============================================================================
    maze = [[ 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
             [ 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 1, 0, 0, 1, 1, 1],
             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 1, 0, 0, 0, 0, 0]]
# =============================================================================
    
    #maze = loadMap("map1-res.png")
    #maze = maze.tolist()
    start = (0, 0)
    end = (5, 5)
    
    path = astar(maze, start, end)
    print(path)

def loadMap(src):
    print("Loading map...")
    img = cv2.imread(src)  
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, imgBW = cv2.threshold(img,127,255,cv2.THRESH_BINARY_INV)
    imgBin = (1/255)*imgBW
    imgBin = imgBin.astype(int)
    return imgBin

if __name__ == '__main__':
    main()