#!/usr/bin/env python
import cv2 as cv
import numpy as np
import rospy
from robot_navigation_project.srv import CalculatePath
from robot_navigation_project.srv import CalculatePathRequest
from robot_navigation_project.srv import CalculatePathResponse
from robot_navigation_project.srv import SetOccupancyGrid
from robot_navigation_project.srv import SetOccupancyGridRequest
from robot_navigation_project.srv import SetOccupancyGridResponse
from geometry_msgs.msg import Point
from robot_navigation_project.msg import Matrix2D
from robot_navigation_project.msg import StatusMsg

import robot_controller
#import a_star
#import run_trajectory

global img_height, img_width
scale_resize = 1.0             # scale by which the map image is resized when viewed
points_clicked = []
resolution = 237               # px/m ratio calculated when creating Gazebo world and used for conversions between map and physical world
global start_point_px          # start_point in px (NON-resized map)
global end_point_px            # end point in px (NON resized)
global start_point_px_res      # start point in the resized version
global end_point_px_res        # end point in the resized version
global start_point             # in m for the Gazebo sim
global end_point               # in m

""" # returns an image as an occupancy grid (0-free, 1-obstacle)
def get_occupancy_grid(img):
    cv.imshow("img", img)
    cv.waitKey(0)
    cv.destroyAllWindows()

    # img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # ret, imgBW = cv.threshold(img,127,255,cv.THRESH_BINARY_INV)
    # cv.imshow("BW", imgBW)
    # cv.waitKey(0)
    # cv.destroyAllWindows()
    # imgBin = (1/255)*imgBW
    # imgBin = imgBin.astype(int)
    #return imgBin
    return img

# callback for mouse position in the image
def mouse_callback(event,x,y,flags,param):
    global img_width, img_height, scale_resize
    if event == cv.EVENT_LBUTTONDOWN:
        #print("x: %i y: %i px" % (x, y))
        points_clicked.append((x, y))

# Gets the resized image and the path as a list of points
def display_path(img, path):
    print("\n START PX:   ")
    print("\nx: %i y %i\n" % (start_point_px_res[0], start_point_px_res[1]))
    img[start_point_px_res[1]-5: start_point_px_res[1]+5, start_point_px_res[0]-5 : start_point_px_res[0]+5 ] = (0, 0, 255)
    img[end_point_px_res[1]-5: end_point_px_res[1]+5, end_point_px_res[0]-5 : end_point_px_res[0]+5 ] = (0, 255, 0)

    for i in range(1, len(path)-1):  # skip first and last element as they were already drawn
        img[path[i][1]-2 : path[i][1]+2 , path[i][0]-2:path[i][0]+2] = (100, 100, 100)

    cv.imshow("Map", img)
    cv.waitKey(0)
    cv.destroyAllWindows() """

# Method to send the request to the path planner server
# Sends start and end point in px (non-resized)
def sendPathRequest(start_p, end_p):
    # Wait for the server to exist and be ready
    rospy.wait_for_service('requestPath')
    # Service Proxy
    requestPath = rospy.ServiceProxy('requestPath', CalculatePath)
    try:
        print("Requesting:: Start: [%i, %i] End: [%i, %i] " %(start_p[0], start_p[1], end_p[0], end_p[1]))
        response = requestPath(start_p[0], start_p[1], end_p[0], end_p[1]) # request is sent with 'start,end' as params and server resp. saved 
        return response.path         # return the path, see 'rosrv show' for info
    except rospy.ServiceException, e:
        print("Service call failed: %s" % e)

def setMapRequest(maze):
    rospy.wait_for_service('setMap')
    setMap = rospy.ServiceProxy('setMap', SetOccupancyGrid)
    try:
        print("Sending map-setting request to server")
        mapMessage = Matrix2D()
        mapMessage.width = len(maze[0])   # length of the first list entry, eg. first row
        mapMessage.height = len(maze)     # list length
        mapMessage.cells =  np.reshape(maze, mapMessage.width*mapMessage.height)  # 1D array of int8 cells
        response = setMap(mapMessage)
        return response.status
    except rospy.ServiceException, e:
        print("Service call failed: %s" % e)
        

def main():
    global scale_resize, img_height, img_width, points_clicked   
    global start_point_px, end_point_px, start_point, end_point, start_point_px_res, end_point_px_res

    # src = "map1.png"
    # print("Loading map %s ..." % src)
    # print("Please choose the start and end point by clicking in the image.")
    # img = cv.imread(src)
    # cv.imshow("Test", img)
    # cv.waitKey(0)
    # cv.destroyAllWindows()
    # #maze = get_occupancy_grid(img)

    # # save image width and height
    # img_height = int(img.shape[0])
    # img_width = int(img.shape[1])

    # #calculate the percent of original dimensions
    # width = int(img.shape[1] * scale_resize)
    # height = int(img.shape[0] * scale_resize)
    # # dsize
    # dsize = (width, height)
    # # resize image
    # img_resized = cv.resize(img, dsize)
    # cv.namedWindow("Map")

    # cv.setMouseCallback("Map", mouse_callback)

    # cv.imshow("Map", img_resized)
    # cv.waitKey(0)
    # cv.destroyAllWindows()

    # # Print the points
    # if(len(points_clicked) >= 2):
        
    #     start_point_px = (int(points_clicked[0][0]/scale_resize), int(points_clicked[0][1]/scale_resize))
    #     end_point_px = (int(points_clicked[1][0]/scale_resize), int(points_clicked[1][1]/scale_resize))

    #     start_point = px2m(start_point_px)
    #     end_point = px2m(end_point_px)

    #     start_point_px_res = points_clicked[0]
    #     end_point_px_res = points_clicked[1]

    #     print("\nStart point:\n - resized view: %i %i px\n - full scale: %i %i px\n - physical: %f %f m" % \
    #     ( start_point_px_res[0], start_point_px_res[1], start_point_px[0], start_point_px[1], start_point[0], start_point[1]))

    #     print("\nEnd point:\n - resized view: %i %i px\n - full scale: %i %i px\n - physical: %f %f m\n\n" % \
    #     ( end_point_px_res[0], end_point_px_res[1], end_point_px[0], end_point_px[1], end_point[0], end_point[1]))

    # else:
    #     print("You need to click 2 points - start and end")


    maze = [[ 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [ 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 1, 1, 1],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0]]
    status = setMapRequest(maze)
    
    if(status.complete == 1):
        print("Server saved the map...")
    else:
        print("Something went wrong with the saving!")

    # Pass the points to A* search algorithm to obtain the path
    # The non-resized points are passed, this is to be used for Gazebo
    # Therefore the coordinates are as in world and not in the image
    # HERE: send request to the path_planner_server
    path = sendPathRequest([1,2], [0,5])
    print("Path planner client received path from server:\n")
    print(path)

    #robot_controller.robot_controller(path)


    #path = a_star.astar(maze, start_point_px, end_point_px) 
    # For vizualization in the image, 
    #path_resized = a_star.astar(get_occupancy_grid(img_resized), start_point_px_res, end_point_px_res)
    #print("Path:\n")
    #print(path_resized)

    # Display the start and end point and the path between
    #cv.destroyAllWindows()  # if not closed before
    #display_path(img_resized, path_resized) # display the path

if __name__ == '__main__': 
    main()