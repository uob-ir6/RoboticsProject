import rospy

class WaiterRobotsNode(object):
    def __init__(self):
        # add stuff here 
        pass 

if __name__ == '__main__':
    rospy.init_node('waiter_robots_node')
    node = WaiterRobotsNode()
    rospy.spin()