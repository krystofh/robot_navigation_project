#!/usr/bin/env python
import cv2 as cv
import numpy as np
import a_star
import run_trajectory

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

# returns an image as an occupancy grid (0-free, 1-obstacle)
def get_occupancy_grid(img):
    img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    ret, imgBW = cv.threshold(img,127,255,cv.THRESH_BINARY_INV)
    imgBW[imgBW<255] = 0
    imgBW[imgBW==255] = 1
    #print("imgBW")
    #print(imgBW)    
    return imgBW

# callback for mouse position in the image
def mouse_callback(event,x,y,flags,param):
    global img_width, img_height, scale_resize
    if event == cv.EVENT_LBUTTONDOWN:
        #print("x: %i y: %i px" % (x, y))
        points_clicked.append((x, y))

# Receives the movement trajectory in px and converts to m
def convert_trajectory(trajectory_px):
    print("")

# Converts the px to m position in Gazebo world according to the map resolution
def px2m(point):
    global resolution
    return ((float(point[0])/float(resolution)) , (float(point[1])/float(resolution)))  # return point in m

# Gets the resized image and the path as a list of points
def display_path(img, path):
    print("\n START PX:   ")
    print("\nx: %i y %i\n" % (start_point_px_res[0], start_point_px_res[1]))
    offset = 0
    offset2 = 0
    if(offset != 0):
        img[start_point_px_res[1]-offset: start_point_px_res[1]+offset, start_point_px_res[0]-offset : start_point_px_res[0]+offset ] = (0, 0, 255)
        img[end_point_px_res[1]-offset: end_point_px_res[1]+offset, end_point_px_res[0]-offset : end_point_px_res[0]+offset ] = (0, 255, 0)
    else:
        img[start_point_px_res[1], start_point_px_res[0]] = (0, 0, 255)
        img[end_point_px_res[1], end_point_px_res[0]] = (0, 255, 0)
   
    for i in range(1, len(path)-1):  # skip first and last element as they were already drawn
        if offset2 !=0:
            img[path[i][0]-offset2 : path[i][0]+offset2 , path[i][1]-offset2:path[i][1]+offset2] = (100, 100, 100)
        else:
            img[path[i][0], path[i][1]] = (100, 100, 100)
       
    cv.imshow("Map", img)
    cv.waitKey(0)
    cv.destroyAllWindows()

def main():
    global scale_resize, img_height, img_width, points_clicked   
    global start_point_px, end_point_px, start_point, end_point, start_point_px_res, end_point_px_res

    src = "Map4-2.png"
    print("Loading map %s ..." % src)
    print("Please chose the start and end point by clicking in the image.")
    img = cv.imread(src)
    maze = get_occupancy_grid(img)

    # save image width and height
    img_height = int(img.shape[0])
    img_width = int(img.shape[1])

    #calculate the percent of original dimensions
    width = int(img.shape[1] * scale_resize)
    height = int(img.shape[0] * scale_resize)
    # dsize
    dsize = (width, height)
    # resize image
    img_resized = cv.resize(img, dsize)
    cv.namedWindow("Map")

    cv.setMouseCallback("Map", mouse_callback)

    cv.imshow("Map", img_resized)
    cv.waitKey(0)
    cv.destroyAllWindows()

    # Print the points
    if(len(points_clicked) >= 2):
        
        start_point_px = (int(points_clicked[0][0]/scale_resize), int(points_clicked[0][1]/scale_resize))
        end_point_px = (int(points_clicked[1][0]/scale_resize), int(points_clicked[1][1]/scale_resize))

        start_point = px2m(start_point_px)
        end_point = px2m(end_point_px)

        start_point_px_res = points_clicked[0]
        end_point_px_res = points_clicked[1]

        print("\nStart point:\n - resized view: %i %i px\n - full scale: %i %i px\n - physical: %f %f m" % \
        ( start_point_px_res[0], start_point_px_res[1], start_point_px[0], start_point_px[1], start_point[0], start_point[1]))

        print("\nEnd point:\n - resized view: %i %i px\n - full scale: %i %i px\n - physical: %f %f m\n\n" % \
        ( end_point_px_res[0], end_point_px_res[1], end_point_px[0], end_point_px[1], end_point[0], end_point[1]))

    else:
        print("You need to click 2 points - start and end")

    # Pass the points to A* search algorithm to obtain the path
    # The non-resized points are passed, this is to be used for Gazebo
    # Therefore the coordinates are as in world and not in the image

    #path = a_star.astar(maze, start_point_px, end_point_px) 
    # For vizualization in the image, 
    #maze = [[ 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    #          [ 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    #          [0, 0, 0, 0, 1, 0, 0, 1, 1, 1],
    #          [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #          [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    #          [0, 0, 0, 0, 1, 0, 0, 0, 0, 0]]
    #        start = (0, 0)
    # end = (5, 5) 
    print("Pixel:  ")
    #print(maze[144][190])
    print("Map:")
    print(maze)
    path_resized = a_star.astar(maze, (start_point_px_res[1], start_point_px_res[0]), (end_point_px_res[1], end_point_px_res[0]))
    print("Path:\n")
    print(path_resized)

    # Display the start and end point and the path between
    cv.destroyAllWindows()  # if not closed before
    display_path(img_resized, path_resized) # display the path

if __name__ == '__main__': 
    main()