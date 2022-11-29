#!/usr/bin/python3

"""
This is the main entry point for the particle filter exercise node. It
subscribes to laser, map, and odometry and creates an instance of
pf.PFLocaliser() to do the localisation.
"""

import rospy
import random
import numpy as np
import time
from geometry_msgs.msg import Pose
from robot import Robot
import sys
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String



class WaiterRobotsNode(object):

    def __init__(self):
        # declare attributes
        self.map = []
        self.robots = []
        self.tableLocations = []
        self.activePaths = []
        
        # path planning mdp model parameters
        self.pathStates = []
        self.actions = []
        self.transitionModel = []

        rospy.loginfo("Waiting for a map...")
        try:
            ocuccupancy_map = rospy.wait_for_message("/map", OccupancyGrid, 20)
        except:
            rospy.logerr("Problem getting a map. Check that you have a map_server"
                     " running: rosrun map_server map_server <mapname> " )
            sys.exit(1)
        rospy.loginfo("Map received. %d X %d, %f px/m." %
                      (ocuccupancy_map.info.width, ocuccupancy_map.info.height,
                       ocuccupancy_map.info.resolution))


        
        
        
        # initialise map and robots

       
        self.initialiseMapAndRobots(2)


        # # start order attributiob subscriber 
        # rospy.Subscriber("/order", String, self.orderAttribution)
        
        # self.printMap()

        # print(self.robots)

        # self.robots[0].motion()

        #initialise path planning mdp model

        # self.initialsiePathPlanningMDP()
        # self.pathPlanning(self.robots[0], (4,1),[(1,6),(1,1)])

        
        # test motion_model with path and blank utility
        # to test put the robot in quare 4,1 and run this code
        # you should notice some bad things such as the robot 360 every time it moves and the cell size maybe slightly off 
        # but it kinda works 

        path = ([0,2,2,2,1],[])
        goalStates = [(1,1)]
        self.robots[0].motion(path,goalStates)
        
        # self.orderModel(6)
    

   
        # 


    def printMap(self):
        print("printing map...")
        map = self.map
        for i in range(len(map)-1,-1, -1):
            for j in range(0,len(map[i])):
                print(map[i][j], end = '')
                print('|', end = '')
            print()

        

    def initialiseMapAndRobots(self, numberOfRobot):
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
                     
        # set list of starting locations on the map <x,y> -> each robot will be assignned their assignment point at the start of the simulation - filling from the left
        startingLocations = [(4,1),(5,1),(6,1),(7,1) , (10,3),(10,4),(10,5),(10,6),(10,7)]
        
        # list table locations for convinience -> done manually can do automatically using the map
        tableLocations = [(2,1),(2,5),(2,9),(6,5), (6,9), (10,9)]

        # initialise robots with their robot id and their pose (x,y,theta) and their state (), their positional state location <x,y> assignment point <x,y>
        for i in range (0, numberOfRobot):
            robotsPose = Pose()
            robotId = i
            pose = robotsPose
            state = 'idle'
            location = startingLocations[i]
            assignmentPoint = tableLocations[i]
            robot =  Robot(robotId, pose, state, location, assignmentPoint)
            self.robots.append(robot)



    def orderAttribution(self,data):
        # recieve an order/ get the next order from the queue
        print("order recieved: ", data.data)
        
        order = data.data # order of the form # tn where n is the table index

        # define the goal states for the kitchen and each table 

        # kitchen goal state is (9,2) of the form (x,y)

        kitchenState1 = (9,1) 
        kitchenState2 = (9,2)

        # table goal states are the squares around the table of the form [(x,y), (x,y), (x,y), (x,y)]

        # get table index from the order

        tableIndex = int(order[1:])

        kitchenGoalStates = np.concatenate((self.getGoalStates(kitchenState1), self.getGoalStates(kitchenState2)), axis=0)

         # table location
        tableLocation = self.tableLocations[tableIndex]
        tableGoalStates =  self.getGoalStates(tableLocation)

        # loop through robots that are in the oder attribution sate and calculate their respective policies
        idlingRobots = []
        for robot in self.robots:
            if robot.state == 'order-attribution':
                idlingRobots.append(robot)
        
        # for each robot calculate the policy to the kitchen and the table
        policies = []
        for robot in idlingRobots:
            kitchenPolicy = self.pathPlanning(robot, kitchenGoalStates)
            tablePolicy = self.pathPlanning(kitchenGoalStates, tableGoalStates)
            policies.append((kitchenPolicy, tablePolicy,robot.id))



        # assign the order to the robot with the shortest path
        shortestPath = 40 # we know the max path generated is 20 each way so this is a safe upper bound
        shortestPolicyIndex = 0

        for i in range(0, len(policies)):
            if len(policies[i][0]) + len(policies[i][1]) < shortestPath:
                shortestPath = len(policies[i][0][0]) + len(policies[i][1][0])
                shortestPolicyIndex = i
        
        # assign the order to the robot with the shortest path
        print("assigning order to robot: ", policies[shortestPolicyIndex][2], " with path to kitchen: ", policies[shortestPolicyIndex][0][0], " and path to table: ", policies[shortestPolicyIndex][1][0])
        # TODO send off 

        # add the policies to the robots active paths
        self.robots[policies[shortestPolicyIndex][2]].activePaths.append(policies[shortestPolicyIndex][0][0])
        self.robots[policies[shortestPolicyIndex][2]].activePaths.append(policies[shortestPolicyIndex][1][0])


        # tell the robot to move to the follow the path and update its state - will need utility for this aswell
        self.robots[policies[shortestPolicyIndex][2]].state = 'moving-to-kitchen'
        self.robots[policies[shortestPolicyIndex][2]].motion(policies[shortestPolicyIndex][0],kitchenGoalStates)
        self.robots[policies[shortestPolicyIndex][2]].state = 'kitchen'
        # TODO its collect order - logic for this
        self.robots[policies[shortestPolicyIndex][2]].state = 'moving-to-table'
        self.robots[policies[shortestPolicyIndex][2]].motion(policies[shortestPolicyIndex][1],tableGoalStates)
        self.robots[policies[shortestPolicyIndex][2]].state = 'table'
        # TODO its delivered order - logic for this
        # TODO probably should of logged for all the above

        # add the path to the robots active path list - again utility is needed for this

        # execute policy 
        # robot state update 

        # remove from active paths 


        # 
        pass




    # order simulation model #TODO this is probably a different node
    # will need to listen to the frequency model and add them to a queue
    def orderModel (self, numberOfTable):
        # defined demand phases high medium and low
        HIGH = 1 # every factor of 1
        MEDIUM = 2 # every factor of 2
        LOW = 3 # every factor of 3

        #TODO set timeout between tables

        while True:

            orderFrequency = random.randint(1,3)
            if orderFrequency == 1:
                orderFrequency = HIGH
            elif orderFrequency == 2:
                orderFrequency = MEDIUM
            elif orderFrequency == 3:
                orderFrequency = LOW
            
            print("switching to demand phade: ", orderFrequency)
            x = 10 # suggested number of orders for each phase
            # randomly change order frequency every x orders with noise
            numberOfOrders = np.random.normal(x, 0.5*x)

            for i in range (0, int(numberOfOrders)):
                # randomly generate order
                table = random.randint(1, numberOfTable)
                order = 'T' + str(table)
                # publish order

                print("Order for table " + str(table) + " is " + str(order))
                #distribution centered around centered around the frequency  
                rospy.sleep(np.random.normal(orderFrequency, 0.5 * orderFrequency))

            
        # send order message to waiter robot system

    def initialsiePathPlanningMDP(self):
        # initialise path planning mdp model parameters
        
        # define states - we can 
        self.pathStates = self.map

           # define actions

        # for each state is their an obstacle above, below, left, right, we define the actions for a state as a 4-tuple of the possible actions 
        # (a,b,c,d) where a = up, b = down, c = left, d = right

        self.actions = []
        for i in range (0, len(self.pathStates)):
            actionsRow = []
            for j in range (0, len(self.pathStates[i])):
                # check if state is an obstacle if so there is no actions 
                if self.pathStates[i][j] != ' ':
                    actionsRow.append([False,False,False,False])
                    continue

                left = True
                right = True
                up = True
                down = True

                # check if there is an obstacle to the left
                if j-1 >= 0:
                    if self.pathStates[i][j-1] != ' ':
                        left = False
                # check if there is an obstacle to the right
                if j+1 < len(self.pathStates[j]):
                    if self.pathStates[i][j+1] != ' ':
                        right = False
                # check if there is an obstacle above
                if i+1 < len(self.pathStates[i]):
                    if self.pathStates[i+1][j] != ' ':
                        up = False
                # check if there is an obstacle below
                if i-1 >= 0:
                    if self.pathStates[i-1][j] != ' ':
                        down = False
                
                actionsRow.append([up,down,left,right])

                
            self.actions.append(actionsRow)
        print(self.pathStates[8])
        print(self.actions[4][6])


        # define transition model

        # this ones a bit more complicated, we need to initialise a shared transition model which itslef is a 4d array
        # for each state and action, there is a probabilty to move to each of the other states
        # the transition model is a 4d array of the form [state][action][state][probability]

        # therefore we need an action array [up, down, left, right] = ()

        # start timer
        start = time.time()

        emptyTransitionModel = []
        for i in range (0, len(self.pathStates)):
            emptyTransitionModelRow = [0] * (len(self.pathStates[i]))
            emptyTransitionModel.append(emptyTransitionModelRow)

        self.transitionModel = []

        for i in range (0, len(self.pathStates)):
            transitionModelRow = []
            for j in range (0, len(self.pathStates[i])):
                #for each action
                transitionModelActionModels = [] 
                for k in range (0, len(self.actions[i][j])):
                    #set the initial probabilities to 0 except for the expected state
                    
                    possibleTransitions = np.copy(emptyTransitionModel)
                    if (self.actions[i][j][k]):
                        if(k == 0 ):
                            # up
                            if (i+1 < len(self.pathStates[i])):
                                possibleTransitions[i+1][j] = 1
                        elif(k == 1):
                            # down
                            if (i-1 >= 0):
                                possibleTransitions[i-1][j] = 1
                        elif(k == 2):
                            # left
                            if (j-1 >= 0):
                                possibleTransitions[i][j-1] = 1
                        elif(k == 3):
                            # right
                            if (j+1 < len(self.pathStates[j])):
                                possibleTransitions[i][j+1] = 1
                    transitionModelActionModels.append(possibleTransitions)
                    # possibleTransitions[i][j] = '9' I use this to show current state whilst debugging
                    # if (i == 6 and j == 6):
                    #     print("for state ", i, j, " action ", actions[i][j], "at : ",k, " possible transitions are \n", possibleTransitions)
                transitionModelRow.append(transitionModelActionModels)
            self.transitionModel.append(transitionModelRow)

        #end timer
        end = time.time()
        print("time taken to generate transition model: ", end - start)

# define some helper functions for the path planning mdp 
# get the reward for a state
    def getReward(self, rewardModel, state):
        #state = (x,y)
        return rewardModel[state[1]][state[0]]
    
    def getActions(self, state):
        #state = (x,y)
        return self.actions[state[1]][state[0]]
    def getTransitionProbabity(self, state, action, nextState):
        #state = (x,y)
        #nextState = (x,y)
        # action = (up = 0, down = 1, left = 2, right = 3)
        #sum up the values in the transition 
        sum = 0
        for i in range (0, len(self.transitionModel[state[1]][state[0]][action])):
            for j in range (0, len(self.transitionModel[state[1]][state[0]][action][i])):
                sum += self.transitionModel[state[1]][state[0]][action][i][j]
        print("sum: ", sum)

        #calculate the probability
        return self.transitionModel[state[1]][state[0]][action][nextState[1]][nextState[0]] / sum



    #given table location and map return goalstates
    def getGoalStates(self, tableLocation):
        goalStates = []
        # goal states are the 4 adjacent states to the table
        # check if there is an obstacle to the left
        if tableLocation[0]-1 >= 0:
            if self.pathStates[tableLocation[1]][tableLocation[0]-1] == ' ':
                goalStates.append((tableLocation[1],tableLocation[0]-1))
        # check if there is an obstacle to the right
        if tableLocation[0]+1 < len(self.pathStates[tableLocation[1]]):
            if self.pathStates[tableLocation[1]][tableLocation[0]+1] == ' ':
                goalStates.append((tableLocation[1],tableLocation[0]+1))
        # check if there is an obstacle above
        if tableLocation[1]+1 < len(self.pathStates):
            if self.pathStates[tableLocation[1]+1][tableLocation[0]] == ' ':
                goalStates.append((tableLocation[1]+1,tableLocation[0]))
        # check if there is an obstacle below
        if tableLocation[1]-1 >= 0:
            if self.pathStates[tableLocation[1]-1][tableLocation[0]] == ' ':
                goalStates.append((tableLocation[1]-1, tableLocation[0]))
        
        # returns goal state array of the form [(x,y), (x,y), (x,y), (x,y)]
        return goalStates


    # given utility, actions, transitionModel and current state, return the policy []
    def getPolicy(self, utilities, currentState, goalStates):
        policy = []
     
        found = False
        while (not found and len(policy) < 20):
            # for each action find the action that maximises the utility


            maxAction = -1
            maxUtility = -1
            for k in range (0, len(self.actions[currentState[1]][currentState[0]])):
                # for each action find the utility
                # sum of transition_model(state,action) * utility_policy(state)
                sum = 0
                for l in range (0, len(self.transitionModel[currentState[1]][currentState[0]][k])):
                    for m in range (0, len(self.transitionModel[currentState[1]][currentState[0]][k][l])):
                        sum += self.transitionModel[currentState[1]][currentState[0]][k][l][m] * utilities[l][m]
                if (sum > maxUtility):
                    maxUtility = sum
                    maxAction = k
            # update the policy
            policy.append(maxAction)
            # update the current state
            if (maxAction == 0):
                currentState = (currentState[0], currentState[1] + 1)
            elif (maxAction == 1):
                currentState = (currentState[0], currentState[1] - 1)
            elif (maxAction == 2):
                currentState = (currentState[0] - 1, currentState[1])
            elif (maxAction == 3):
                currentState = (currentState[0] + 1, currentState[1])

            #check if we have reached the goal
            for i in range (0, len(goalStates)):
                if (currentState == goalStates[i]):
                    found = True
                    break

        print("policy for location: ", currentState, " is: ", policy)
        return policy

    def pathPlanning(self,robot, initialState, goalStates ): # TODO remove default values - just for testing
        # a = (x,y) b = (x,y)
        # calculate the best path between the two points 

      
     


        # define rewards TODO kinda want to store this with the robot or atleast the policy/goal state

        # TODO 
        rewards = []
        # new array n*n give a reward for each state -0.1 for each step
        for i in range (0, len(self.pathStates)):
            rewardsRow = []
            for j in range (0, len(self.pathStates[i])):
                rewardsRow.append(-0.1)
            rewards.append(rewardsRow)
        print(rewards)


        # give a reward of 1 for reaching the goal state
        for i in range (0, len(goalStates)):
            rewards[goalStates[i][1]][goalStates[i][0]] = 1

        # for each path in the active paths that is not your own, follow the path applying negative reward for each step -1

        # TODO figure out form of the policy so that we can store the active path



        # for each robot apply a negative reward for their current location -1, except for the current robot

        for i in range (0, len(self.robots)):  
            if i != robot.id:
                print("robot: ", i,"is at location: ", self.robots[i].location)
                rewards[self.robots[i].location[1]][self.robots[i].location[0]] = -1
         
        #print(rewards nicely)
        for i in range (0, len(rewards)):
            print(rewards[i])



        
        print("reward: ", self.getReward(rewards, (5,1)))
        print("transition probability: ", self.getTransitionProbabity((5,1), 0, (5,2)))          

        # calculate optimal policy

        # algorthm for value iteration

        # policy* = argmax_policy expected reward of policy
        # utility_policy(state) = expected reward of policy starting from state with discount factor gamma
        # policy*(state) = argmax_action sum of transition_model(state,action) * utility_policy(state)
        # utility_policy(state) = reward(state) + gamma * max_action(sum of transition_model(state,action) * utility_policy(state))


        gamma = 0.9

        start = time.time()
        # start with abritrary utitlies
        utilities = []
        for i in range (0, len(self.pathStates)):
            utilitiesRow = []
            for j in range (0, len(self.pathStates[i])):
                utilitiesRow.append(0)# abs(np.random.normal(0,0.1)))
            utilities.append(utilitiesRow)

        end = time.time()
        print("time taken to generate utilities: ", end - start)

        # print(utilities nicely)
        for i in range (0, len(utilities)):
            print(utilities[i])

        
        start = time.time()

        converged = False
        #for now we will just do 100 iterations
        conv = 0
        maxChange =1
        while ((not converged) and (maxChange > 0.01)) :

            maxChange = 0

            #for each state calculate the updated utility
            for i in range (0, len(self.pathStates)):
                for j in range (0, len(self.pathStates[i])):
                    # for each action find the action that maximises the utility

                    maxAction = -1
                    maxUtility = -1
                    for k in range (0, len(self.actions[i][j])):
                        # for each action find the utility
                        # sum of transition_model(state,action) * utility_policy(state)
                        sum = 0
                        for l in range (0, len(self.transitionModel[i][j][k])):
                            for m in range (0, len(self.transitionModel[i][j][k][l])):
                                sum += self.transitionModel[i][j][k][l][m] * utilities[l][m]
                        if (sum > maxUtility):
                            maxUtility = sum
                            maxAction = k
                    # update the utility
                    previousUtility = utilities[i][j]
                    utilities[i][j] = self.getReward(rewards, (j,i)) + gamma * maxUtility
                    if (abs(previousUtility - utilities[i][j]) > maxChange):
                        maxChange = abs(previousUtility - utilities[i][j])

            conv = conv + 1
            print("i: ", conv ,": max change: ", maxChange)
            if (conv > 50):
                converged = True
        


        end = time.time()
        print("time taken to calculate utilities: ", end - start)
        #set utilites array to 0 with noise
        # update utilities based on neighbours
        # repeat until convergence

        # #give utilities caluclate the policy
     
        currentState = initialState
     

        policy = self.getPolicy(utilities, currentState, goalStates)
        return (policy,utilities)

        # to store the path chosen we need to store the policy and the robot following it 
        # ((robot,(x,y)), y[]), where x is the initial state and y is the policy
        # then when the robot moves we update the policy to remove the first action and if necessary recalculate the policy from the utility
        # therefore the robot will need to store the current utility its following  
        



# main method
if __name__ == '__main__':
    # --- Main Program ---
    rospy.init_node('waiter_robots_node')
    node = WaiterRobotsNode()
    rospy.spin()
    # print("hello world")
  