#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Oct 20 13:23:40 2018

@author: 2020shatgiskessell
"""
import cv2
import numpy as np
import scipy.signal
import math


class Node:
    def __init__(self, xcoordinate= None, ycoordinate= None, h=None, g=None, cost=None):
        self.xcoordinate = xcoordinate
        self.ycoordinate = ycoordinate
        self.h = h
        self.g = g
        self.cost = h + g

    def __repr__(self):
        return repr((self.xcoordinate, self.ycoordinate, self.h, self.g, self.cost))



roomimg = cv2.imread("/Users/2020shatgiskessell/Desktop/Maps/medium2.jpg")

# edge detection
thresh = cv2.cvtColor(roomimg, cv2.COLOR_BGR2GRAY)
edge = cv2.Canny(thresh, 100, 200)
height,width,channels = roomimg.shape



#define the dimensions of the grid
def estimate_noise(I):

  H, W = I.shape

  M = [[1, -2, 1],
       [-2, 4, -2],
       [1, -2, 1]]

  sigma = np.sum(np.sum(np.absolute(scipy.signal.convolve2d(np.array(I), M))))

  sigma = sigma * np.sqrt(0.5 * np.pi) / (6 * (W-2) * (H-2))

  return sigma

boxsize = (math.pow(estimate_noise(edge),-0.708)* 112.32)
matrix_icrement_width = int(width/int(boxsize))
matrix_icrement_height = int(height/int(boxsize))
Matrix =  [[0 for x in range(matrix_icrement_width)] for y in range(matrix_icrement_height)]


Nodes = []
closed_list = []


#CREATE STARTING AND ENDING NODES
starting_node = Node(xcoordinate = 0, ycoordinate = 15, h = 0, g = 0 )
closed_list.append(starting_node)

ending_node = Node(xcoordinate = 19, ycoordinate = 0, h = 0, g = 0 )
#Test endpoints include:
#(17, 18)
#(17, 25)
#(19, 0)


cv2.circle(roomimg,(ending_node.xcoordinate * int(boxsize), ending_node.ycoordinate* int(boxsize)), 5, (0,255,54), -1)


#defines what are obstacles and what are not
cut_off_point = 15
print (boxsize)

box_num = 0
boxesdrawn = 0
for i in range (0,width, int(boxsize)):
    for j in range (0,height, int(boxsize)):
        #1. DRAW THE BLOCKS
        roi_gray = edge[j:j+int(boxsize),i:i+int(boxsize)]
        #2. FIND INTENSITY OF ROI
        roi_avg_intensity = np.mean(roi_gray)
        #3. BASED ON THAT, SEE IF ROI IS AN OBSTACLE OR NOT

        if roi_avg_intensity > cut_off_point:

            #DRAW RECTANGLE AROUND OBSATCLE
            cv2.rectangle(roomimg, (i,j), (i+int(boxsize), j+int(boxsize)),(0,0,255))
            #store top left corneer of ROI in Matrx
            Matrix[int(j/int(boxsize))][int(i/int(boxsize))] = "o"
            boxesdrawn += 1
        else:
            #DRAW RECTANGLE AROUND CLEAR PATH
            cv2.rectangle(roomimg, (i,j), (i+int(boxsize), j+int(boxsize)),(0,255,255))

            #Creates Node if there is a clear path
            h = math.sqrt(math.pow((i/int(boxsize))-ending_node.xcoordinate,2)+math.pow((j/int(boxsize))-ending_node.ycoordinate,2))

            newNode = Node(xcoordinate = i/int(boxsize), ycoordinate = j/int(boxsize), h = h, g = 0 )

           #Add node to Matrix
            Matrix[int(j/int(boxsize))][int(i/int(boxsize))] = newNode

        box_num += 1

def draw_line(x1,y1, x2,y2):
    #convert Matrix point to points on the image
    x1 = int (x1 * int(boxsize))
    #deal with the center instead of the top left corner
    x1 = x1 + int(int(boxsize)/2)

    y1 = int (y1 * int(boxsize))
    y1 = y1 + int(int(boxsize)/2)

    x2 = int(x2 * int(boxsize))
    x2 = x2 + int(int(boxsize)/2)

    y2 = int(y2 * int(boxsize))
    y2 = y2 + int(int(boxsize)/2)


    cv2.line(roomimg,(x1,y1), (x2,y2), (255,255,255), 2)
#---------------------------------------------------------STUFF FOR A STAR ALGORITHM----------------------------------------------------------------------------
k = 0
p = 0
possible_next_steps = []
closed_list = []
def a_star (node, gvalue, possible_next_steps, prev_node):
    global k
    k = k+1
    #check to see if program has reached the end point
    if node.xcoordinate == ending_node.xcoordinate and node.ycoordinate == ending_node.ycoordinate:
        print ("Reached end node")
        return

    x = int(node.xcoordinate)
    y = int(node.ycoordinate)
    
    #get all the possible next steps from given node
    north = Matrix[y-1][x]
    east = Matrix[y][x+1]
    south = Matrix[y+1][x]
    west = Matrix[y][x-1]
    north_east = Matrix[y-1][x+1]
    north_west = Matrix[y-1][x-1]
    south_east = Matrix[y+1][x+1]
    south_west = Matrix[y+1][x-1]

    #calculate g value of each node (BE CAREFUL WITH THE MATH HERE) and add to array
    if north != "o" and x >= 0 and (y-1) >=0 and north not in closed_list:
        north.g = 10
        possible_next_steps.append(north)
        #print ("north x is ", north.xcoordinate, "north y is ", north.ycoordinate)

    if east != "o" and (x+1) >= 0 and y >=0 and east not in closed_list:
        east.g = 10
        possible_next_steps.append(east)
        #print ("east x is ", east.xcoordinate, "east y is ", east.ycoordinate)

    if south != "o" and x >= 0 and (y+1) >=0 and south not in closed_list:
        south.g = 10
        possible_next_steps.append(south)
        #print ("south x is ", south.xcoordinate, "south y is ", south.ycoordinate)

    if west != "o" and (x-1) >= 0 and (y) >=0 and west not in closed_list:
        west.g = 10
        possible_next_steps.append(west)
        #print ("west x is ", west.xcoordinate, "west y is ", west.ycoordinate)

    if north_east != "o" and (x+1) >= 0 and (y-1) >=0 and north_east not in closed_list:
        north_east.g = 15
        possible_next_steps.append(north_east)
        #print ("north east x is ", north_east.xcoordinate, "north east y is ", north_east.ycoordinate)

    if north_west != "o" and (x-1) >= 0 and (y-1) >=0 and north_west not in closed_list:
        north_west.g = 15
        possible_next_steps.append(north_west)
        #print ("north west x is ", north_west.xcoordinate, "north west y is ", north_west.ycoordinate)

    if south_east != "o" and (x+1) >= 0 and (y+1) >=0 and south_east not in closed_list:
        south_east.g = 15
        possible_next_steps.append(south_east)
        #print ("south east x is ", south_east.xcoordinate, "south east y is ", south_east.ycoordinate)

    if south_west != "o" and (x-1) >= 0 and (y+1) >=0 and south_west not in closed_list:
        south_west.g = 15
        possible_next_steps.append(south_west)
        #print ("south west x is ", south_west.xcoordinate, "south west y is ", south_west.ycoordinate)

    #check if there are any possible next steps
    if not possible_next_steps:
        print ("There are no possible next steps")
        return

    #Sort possible next steps by cost
    possible_next_steps = sorted(possible_next_steps, key=lambda x: x.cost)
    #Find lowest cost node and make that next step
    next_step = possible_next_steps[0]
    #Add lowest cost node to closed list
    closed_list.append(next_step)
    #Remove this node from possible next steps
    possible_next_steps.remove(next_step)


    print ("(x,y) = (", next_step.xcoordinate ,",", next_step.ycoordinate,")")

    #Draw line between current node and the next step node
    draw_line(x, y, next_step.xcoordinate, next_step.ycoordinate)
    #update gvalue
    gvalue = gvalue + next_step.g
    #repeat for next node, but check if program is going backwards cus it reached a dead end
    if k < 100:
        a_star(next_step, gvalue, possible_next_steps, node)

a_star(starting_node, 0, possible_next_steps, None)
#---------------------------------------------------------STUFF FOR A STAR ALGORITHM----------------------------------------------------------------------------

def print_matrix():
    f = open('obstaclemap.txt', 'w')
    f.write('\n'.join([''.join(['{:4}'.format(item) for item in row])
      for row in Matrix]))
    f.close()


#print matrix
#print_matrix()

#save image file
#cv2.imwrite('roomimg.jpg', roomimg)

#display image
cv2.imshow("image", roomimg)


if cv2.waitKey(0) & 0xff == 27:
    cv2.destroyAllWindows()
