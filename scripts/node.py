#!/usr/bin/python3

"""
This is the main entry point for the particle filter exercise node. It
subscribes to laser, map, and odometry and creates an instance of
pf.PFLocaliser() to do the localisation.
"""

import rospy
import random



class WaiterRobotsNode(object):
    def __init__(self):
        print("Hello World")




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
  