#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Point
from robot_navigation_project.srv import CalculatePath
from robot_navigation_project.srv import CalculatePathRequest
from robot_navigation_project.srv import CalculatePathResponse
from robot_navigation_project.srv import SetOccupancyGrid
from robot_navigation_project.srv import SetOccupancyGridRequest
from robot_navigation_project.srv import SetOccupancyGridResponse
from robot_navigation_project.msg import StatusMsg
import a_star

global occupancy_grid

def handle_CalculatePath(req):
    global occupancy_grid       # use the saved map
    print("Planner server received request for path:\n  [%i, %i] -> [%i, %i]" % (req.start_x, req.start_y, req.end_x, req.end_y))
    startPoint = Point()
    startPoint.x = req.start_x
    startPoint.y = req.start_y
    startPoint.z = 0
    endPoint = Point()
    endPoint.x = req.end_x
    endPoint.y = req.end_y
    endPoint.z = 0

    # Call A* algorithm with : occupancy_grid, startpoint, endpoint (these as [x y], integers)
    path = a_star.astar(occupancy_grid, (int(req.start_x), int(req.start_y)), (int(req.end_x), int(req.end_y)))
    print("Obtained path:\n")
    print(path)
    # Conversion of the path into a message including the array of Points:
    response = []
    for i in range(0, len(path)):
        p = Point()
        p.x = path[i][0]
        p.y = path[i][1]
        p.z = 0.0
        response.append(p)
    return CalculatePathResponse(response)  # send back the response

def handle_SetOccupancyGrid(req):
    global occupancy_grid
    print("Planner server is setting new map.")
    print(" Dimensions: %i x %i\n" % (req.map.width, req.map.height))
    # process 1D array of cells into 2D map
    width = req.map.width
    height = req.map.height
    cells = req.map.cells
    n = width*height
    maze = []
    counter = 0
    for i in range(0, height):     # create empty rows
        maze.append([])  
        for j in range(0, width):  # create columns inside each row by copying the counter-th item from the vector
            maze[i].append(cells[counter])
            counter = counter+1
    print("Saved maze:")
    print(maze)
    occupancy_grid = maze   # save the map as global variable

    # Send back status message (Map saved)
    statusMessage = StatusMsg()
    statusMessage.complete = 1
    statusMessage.status = "Map saved"
    return SetOccupancyGridResponse(statusMessage)

def path_planner_server():
    rospy.init_node('path_planner_client')
    rospy.Service('requestPath', CalculatePath , handle_CalculatePath)  # start the service CalculatePath with the handle
    rospy.Service('setMap', SetOccupancyGrid, handle_SetOccupancyGrid)  # start the service with name,msg-type and handle params
    print("Server started...")
    rospy.spin()

if __name__ == '__main__':
    try:
        path_planner_server()
    except rospy.ROSInterruptException:
        pass