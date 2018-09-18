#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 15 21:23:40 2018

@author: 2020shatgiskesselldd
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
# ret, thresh = cv2.threshold(roomimg, 127, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C)
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

ending_node = Node(xcoordinate = 19, ycoordinate = 1, h = 0, g = 0 )


#defines what are obstacles and what are not
cut_off_point = 15
print (boxsize)
#U HAVE TO CHANGE CUT OFF POINT BASED ON EVERY IMAGE

box_num = 0
boxesdrawn = 0
for i in range (0,width, int(boxsize)):
    for j in range (0,height, int(boxsize)):
        #1. DRAW THE BLOCKS
        roi_gray = edge[j:j+int(boxsize),i:i+int(boxsize)]
        #2. FIND INTENSITY OF ROI
        roi_avg_intensity = np.mean(roi_gray)
        #3. BASED ON THAT, SEE IF ROI IS AN OBSTACLE OR NOT
        #print(box_num,"(",i,",",j,")")
        
        if roi_avg_intensity > cut_off_point:
            # if box_num < 200:
                # print("roi_avg_intensity:", roi_avg_intensity)
                
            #DRAW RECTANGLE AROUND OBSATCLE
            cv2.rectangle(roomimg, (i,j), (i+int(boxsize), j+int(boxsize)),(0,0,255))
            #store top left corneer of ROI in Matrx
            Matrix[int(j/int(boxsize))][int(i/int(boxsize))] = "o"
            boxesdrawn += 1
        else:
            #store top left corneer of ROI in Matrx
            cv2.rectangle(roomimg, (i,j), (i+int(boxsize), j+int(boxsize)),(0,255,255))
            
            
            #Make sure i and j are not switch up!
            #Creates Node if there is a clear path
            h = math.sqrt(math.pow(i-ending_node.xcoordinate,2)+math.pow(j-ending_node.ycoordinate,2))
            newNode = Node(xcoordinate = i, ycoordinate = j, h = h, g = 0 )

           #Add node to Matrix
            Matrix[int(j/int(boxsize))][int(i/int(boxsize))] = newNode



        box_num += 1
        
def draw_line(x1,y1, x2,y2):
    #convert Matrix point to points on the image
    x1 = x1 * int(boxsize)
    #deal with the center instead of the top left corner
    x1 = x1 + int(int(boxsize)/2)
    
    y1 = y1 * int(boxsize)
    y1 = y1 + int(int(boxsize)/2)
    
    x2 = x2 * int(boxsize)
    x2 = x2 + int(int(boxsize)/2)

    y2 = y2 * int(boxsize)
    y2 = y2 + int(int(boxsize)/2)


    cv2.line(roomimg,(x1,y1), (x2,y2), (255,255,255), 2)

#---------------------------------------------------------STUFF FOR A STAR ALGORITHM----------------------------------------------------------------------------


def astar (node, gvalue):
    
    #check to see if program has reached the end point
    if node.xcoordinate == ending_node.xcoordinate:
        if node.ycoordinate == ending_node.ycoordinate:
            print ("Reached end node")
            return
    
    x = node.xcoordinate
    y = node.ycoordinate
    
    #check to see if end point has been reaches
    if x == ending_node.xcoordinate:
        if y == ending_node.ycoordinate:
            return
        
        
    possible_next_steps = []
    
    #get all the possible next steps from given node
    #PROBLEM HERE: ALL OF THESE ARE BEING RECONGIZED AS OBSTACLES!!!!
    north = Matrix[x][y-1]
    east = Matrix[x+1][y]
    south = Matrix[x][y+1]
    west = Matrix[x-1][y]
    north_east = Matrix[x+1][y-1]
    north_west = Matrix[x-1][y-1]
    south_east = Matrix[x+1][y+1]
    south_west = Matrix[x-1][y+1]
    
    #calculate g value of each node (BE CAREFUL WITH THE MATH HERE) and add to array
    if north != "o":
        north.g = gvalue + 10
        possible_next_steps.append(north)
        print ("north x is ", north.xcoordinate, "north y is ", north.ycoordinate)
    
    if east != "o":
        east.g = gvalue + 10
        possible_next_steps.append(east)
        print ("east x is ", east.xcoordinate, "east y is ", east.ycoordinate)


    if south != "o":
        south.g = gvalue + 10
        possible_next_steps.append(south)
        print ("south x is ", south.xcoordinate, "south y is ", south.ycoordinate)


    if west != "o":
        west.g = gvalue + 10
        possible_next_steps.append(west)
        print ("west x is ", west.xcoordinate, "west y is ", west.ycoordinate)

    if north_east != "o":
        north_east.g = gvalue + 15
        possible_next_steps.append(north_east)
        print ("north east x is ", north_east.xcoordinate, "north east y is ", north_east.ycoordinate)

    if north_west != "o":
        north_west.g = gvalue + 15
        possible_next_steps.append(north_west)
        print ("north west x is ", north_west.xcoordinate, "north west y is ", north_west.ycoordinate)


    if south_east != "o":
        south_east.g = gvalue + 15
        possible_next_steps.append(south_east)
        print ("south east x is ", south_east.xcoordinate, "south east y is ", south_east.ycoordinate)

    if south_west != "o":
        south_west.g = gvalue + 15
        possible_next_steps.append(south_west)
        print ("south west x is ", south_west.xcoordinate, "south west y is ", south_west.ycoordinate)

#Find lowerst cost node and add that to closed_list
    newlist = sorted(possible_next_steps, key=lambda x: x.cost)
    closed_list.append(newlist[0])
#Draw line between current node and the next step node
    draw_line(x, y, x+1, y-1)
#update gvalue'
    gvalue = gvalue + newlist[0].g
    #gvalue = gvalue + next_step.get(next_step)

#repeat for next node
    #astar(possible_next_steps[0], gvalue)



    
#A STAR TIME
#1. Start with node N

#2. Calculate distance between node N and its nearest neighbores in the NESW directions (possible add northeast, northwest, southeast, soutthwest)
    #perhaps look at Matrix

#3. Add that to h
#4. Bubble sort lowerst cost node and add that to closed array
#5. N = lowest cost node
    
    
#FIGURE OUT WHAT TO DO AT END


#ALL OF THIS IS ONLY IF U WANT A STAR


astar(starting_node, 0)
#---------------------------------------------------------STUFF FOR A STAR ALGORITHM----------------------------------------------------------------------------


def print_matrix():
    f = open('obstaclemap.txt', 'w')
    f.write('\n'.join([''.join(['{:4}'.format(item) for item in row]) 
      for row in Matrix]))
    f.close()



#print_matrix()

#cv2.imwrite('roomimg.jpg', roomimg)
cv2.imshow("image", roomimg)


if cv2.waitKey(0) & 0xff == 27:
    cv2.destroyAllWindows()