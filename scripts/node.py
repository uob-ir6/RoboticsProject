#!/usr/bin/python3

"""
This is the main entry point for the particle filter exercise node. It
subscribes to laser, map, and odometry and creates an instance of
pf.PFLocaliser() to do the localisation.
"""

import rospy
import random
import numpy as np




class WaiterRobotsNode(object):
    
    def printMap(self):
        print("printing map...")
        map = self.map
        for i in range(len(map)-1,-1, -1):
            for j in range(0,len(map[i])):
                print(map[i][j], end = '')
                print('|', end = '')
            print()

        

    def initialiseMapAndRobots(self):
        print("initialising map and robots...")
        # initialise n*m map with the tables locations T and the kitchen location K 
        # TODO this map is flipped such that it matches our designs and grid, not sure if that was the best call
        self.map = np.array([ 
                    ['x','x','x','x','x','x','x','x','x','x','x','x'],
                    ['x',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ','x'],
                    ['x',' ','T',' ',' ',' ','T',' ',' ',' ','T','x'],
                    ['x',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ','x'],
                    ['x',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ','x'],
                    ['x',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ','x'],
                    ['x',' ','T',' ',' ',' ','T',' ',' ',' ',' ','x'],
                    ['x',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ','x'],
                    ['x',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ','x'],
                    ['x',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ','x'],
                    ['x',' ','T',' ',' ',' ',' ',' ',' ','K','K','x'],
                    ['x','x','x','x','x','x','x','x','x','x','x','x'] ])
        self.map = self.map[::-1]
                     
        # set list of starting locations on the map -> each robot will be assignned their assignment point at the start of the simulation - filling from the left
        startingLocations = [(1,4),(1,5),(1,6),(1,7) , (3,10),(4,10),(5,10),(6,10),(7,10)]

        # initialise robots with their robot id and their pose (x,y,theta) and their state (idle, serving, delivering), their positional state location <x,y> assignment point <x,y>
        


    def __init__(self):
        self.map = []
        self.robots = []

        print("Hello World")
        self.printMap()
        self.initialiseMapAndRobots()
        self.printMap()






    # order simulation model #TODO this is probably a different node
    # will need to listen to the frequency model and add them to a queue
    def orderModel (numberOfTable):
        # defined demand phases high medium and low
        HIGH = 1 # every factor of 1
        MEDIUM = 2 # every factor of 2
        LOW = 3 # every factor of 3

        # for now Ill set the order frequency to HIGH
        orderFrequency = HIGH
        while True:

            orderFrequency = random.randint(1,3)
            if orderFrequency == 1:
                orderFrequency = HIGH
            elif orderFrequency == 2:
                orderFrequency = MEDIUM
            elif orderFrequency == 3:
                orderFrequency = LOW
            

            x = 10 # suggested number of orders for each phase
            # randomly change order frequency every x orders with noise
            numberOfOrders = random.normal(x, 0.5*x)

            for i in range (0,numberOfOrders):
                # randomly generate order
                table = random.randint(1, numberOfTable)
                order = 'T' + str(order)
                # publish order

                print("Order for table " + str(table) + " is " + str(order))
                #distribution centered around centered around the frequency  
                rospy.sleep(random.normal(orderFrequency, 0.5 * orderFrequency))


        # send order message to waiter robot system







# main method
if __name__ == '__main__':
    # --- Main Program ---
    rospy.init_node('waiter_robots_node')
    node = WaiterRobotsNode()
    rospy.spin()
    # print("hello world")
  