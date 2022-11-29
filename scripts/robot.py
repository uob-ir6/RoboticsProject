
from geometry_msgs.msg import Twist

from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from tf.msg import tfMessage
from tf import transformations
import rospy
import time
from nav_msgs.msg import Odometry
import numpy as np

class Robot(object):
    def __init__(self, robotId, pose, state, location, assignmentPoint):
        self.id = robotId
        self.pose = pose
        self.states = state
        # states include = "kitchen" "table" "demand-rate" "assignment-point" "order-attribution"
        self.activePaths = [] # array of (policies, utility) mappings
        self.location = location
        self.assignmentPoint = assignmentPoint
        self.tf_message = tfMessage()
        

        robot_prefix = "robot_"+str(self.id)
        # robot_prefix = ""
        odom_Sub = rospy.Subscriber(robot_prefix+"/odom", Odometry, self.odomCallback)

        
        time.sleep(3)
       
        
        # add motion model subscribeers here stuff here TODO
        # add functions to execute the actions here TODO



    def odomCallback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        self.pose = data.pose.pose
       
        
        

    def motion (self, path,goalStates) :
        #path is (policy, utility) mapping

        policy = path[0]
        utility = path[1]


        

        

        robot_prefix = "robot_"+str(self.id)
        # robot_prefix = ""
        pub = rospy.Publisher(robot_prefix+'/cmd_vel', Twist, queue_size=100)


        # takes in a policy and executes it


        # for each action in the policy 0, 1, 2 ,3 = up, down, left, right
        print("executing policy: ", policy)
        while len(policy) > 0:
            # previouse location
            prevLocation = self.location


            #   execute the action
            print("executing action: ", policy[0])
            rotate(self,pub,policy[0])
            moveForwardOneSquare(self,pub)

            #  update the pose give action and location
            # TODO localistion

            # set the expected location
            expectedLocation = [prevLocation[0], prevLocation[1]]
            if policy[0] == 0:
                expectedLocation[1] += 1
            elif policy[0]  == 1:
                expectedLocation[1] -= 1
            elif policy[0]  == 2:
                expectedLocation[0] -= 1
            elif policy[0]  == 3:
                expectedLocation[0] += 1

            # TODO for now just set the location to the expected location
            self.location = expectedLocation

            #  update policy if the state is not the same as the policy state
            if self.location != expectedLocation:
                # update policy based on utility
                from node import WaiterRobotsNode
                super(WaiterRobotsNode, self).updatePolicy(utility, self.location, goalStates) # TODO goal states
                pass


            #  if policy state is the same as the state then update the policy removing from the end 
            else:
                policy.pop(0)
       



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
     # after movement is complete, recalculate transform 
    currentTime = rospy.Time.now()
    recalculateTransform(self, currentTime)

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
    
    # after movement is complete, recalculate transform 
    currentTime = rospy.Time.now()
    recalculateTransform(self, currentTime)


def recalculateTransform  (self,currentTime):
    """
    Creates updated transform from /odom to /map given recent odometry and
    laser data.
    
    :Args:
        | currentTime (rospy.Time()): Time stamp for this update
        """
    


    transform = Transform()
    t_est = transformations.quaternion_matrix([self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w])
    t_est[0,3] = self.pose.position.x
    t_est[1,3] = self.pose.position.y
    t_est[2,3] = self.pose.position.z

    t_odom = transformations.quaternion_matrix([self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w])
    t_odom[0,3] = self.pose.position.x
    t_odom[1,3] = self.pose.position.y
    t_odom[2,3] = self.pose.position.z
    T = np.dot(t_odom, np.linalg.inv(t_odom))
    q = transformations.quaternion_from_matrix(T) 

    transform.translation.x = T[0,3]
    transform.translation.y = T[1,3]
    transform.translation.z = T[2,3]
    transform.rotation.x = q[0]
    transform.rotation.y = q[1]
    transform.rotation.z = q[2]
    transform.rotation.w = q[3]

    new_tfstamped = TransformStamped()
    
    prefix = "robot_"+str(self.id)
    new_tfstamped.child_frame_id = prefix + "odom"
    new_tfstamped.header.frame_id = "base_link"
    new_tfstamped.header.stamp = currentTime
    new_tfstamped.transform = transform

    self.tf_message = tfMessage(transforms = [new_tfstamped])