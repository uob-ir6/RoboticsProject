
from geometry_msgs.msg import Twist
import rospy
import time
from nav_msgs.msg import Odometry

class Robot(object):
    def __init__(self, robotId, pose, state, location, assignmentPoint):
        self.id = robotId
        self.pose = pose
        self.state = state
        self.location = location
        self.assignmentPoint = assignmentPoint

        

        robot_prefix = "robot_"+str(self.id)
        # robot_prefix = ""
        odom_Sub = rospy.Subscriber(robot_prefix+"/odom", Odometry, self.odomCallback)

        
        time.sleep(3)
       
        
        # add motion model subscribeers here stuff here TODO
        # add functions to execute the actions here TODO



    def odomCallback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        self.pose = data.pose.pose
       
        
        

    def motion (self) : # pose, action):
        # (x,y,theta)
        # 

        # 0 thatn up 90

        

        robot_prefix = "robot_"+str(self.id)
        # robot_prefix = ""
        pub = rospy.Publisher(robot_prefix+'/cmd_vel', Twist, queue_size=100)
        # pub = rospy.Publisher("cmd_vel", Twist, queue_size=100)

        # rospy.init_node('motion_model', anonymous=True)
        
        # moveForwardOneSquare(pub)
        # rotate(self, pub, 3)
        rotate(self, pub, 2)
        moveForwardOneSquare(self, pub)

        # # (0,1,2,3) where 0 is up, 1 is down, 2 is left, 3 is right
        # if action == 0 : 
        #     expectedTheta = 0
        # elif action == 1:
        #     expectedTheta = 180
        # elif action == 2:
        #     expectedTheta = 90
        # elif action == 3:
        #     expectedTheta = 270
        #     # point robot up
        #     # move robot 1 grid unit up


    def execute(thetaAmount, forwardAmount):
        # execute the action
        # update the pose
        # update the state
        # update the location
        # update the assignment point
        # publish the new pose
        pass

def moveForwardOneSquare (self, pub):
    #each square is 1 meter

    # get current position 
    
    currentPose = (self.pose.position.x, self.pose.position.y)
    previousPose = currentPose

    xdif = 0
    ydif = 0
    
    i =0
    rate = rospy.Rate(10) # 10hz
    threshold = 0.1
    cellSize = 2
    while (xdif < (cellSize - threshold) and ydif  < (cellSize - threshold)):
            base_data = Twist()
            base_data.linear.x = 1
            pub.publish( base_data )
            # update current position
            currentPose = (self.pose.position.x, self.pose.position.y)
            # calculate difference between current position and previous position
            xdif = abs(currentPose[0] - previousPose[0])
            ydif = abs(currentPose[1] - previousPose[1])

            rate.sleep()
            i+= 1
    # move forward one square

def rotate (self, pub, direction ):
    # get current pose 
    # threshold = (z,w)

    print("ori: ", self.pose.orientation)

    up = (0.7, 0.7)
    down = (-0.7, 0.7)
    left = (1, 0)
    right = (0, 1)

    targetDirection = None

    if (direction == 0):
        targetDirection = up
    elif direction == 1:
        targetDirection = down
    elif direction == 2:
        targetDirection = left
    elif direction == 3:
        targetDirection = right


    currentOrientation = (self.pose.orientation.z, self.pose.orientation.w)
    
    


    # determine wwhich direction to rotate 

    # rotate until threshold for direction   
    i =0
    rate = rospy.Rate(10) # 10hz


    zdif = 1
    wdif = 1
    threshold = 0.02

    while (zdif > threshold or wdif > threshold):
            print("rotating zdif: ", zdif, " wdif: ", wdif)
            base_data = Twist()
            base_data.angular.z = 0.3 # 0.75 rad/s
            pub.publish( base_data )
            # update current orientation
            currentOrientation = (self.pose.orientation.z, self.pose.orientation.w)
            # calculate difference between current orientation and desired orientation
            zdif = abs(currentOrientation[0] - targetDirection[0])
            wdif = abs(currentOrientation[1] - targetDirection[1])
            rate.sleep()
            i+= 1
