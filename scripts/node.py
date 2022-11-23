import random

import rospy

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
    rospy.init_node('waiter_robots_node')
    node = WaiterRobotsNode()
    rospy.spin()