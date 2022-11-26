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

from robot import Robot




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
        
        
        
        # initialise map and robots
        self.initialiseMapAndRobots(2)
        self.printMap()

        print(self.robots)

        #initialise path planning mdp model
        self.initialsiePathPlanningMDP()
        self.pathPlanning(self.robots[0])

        # self.orderModel(6)
    
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
            robotId = i
            pose = startingLocations[i]
            state = 'idle'
            location = startingLocations[i]
            assignmentPoint = tableLocations[i]
            robot =  Robot(robotId, pose, state, location, assignmentPoint)
            self.robots.append(robot)









    # order simulation model #TODO this is probably a different node
    # will need to listen to the frequency model and add them to a queue
    def orderModel (self, numberOfTable):
        # defined demand phases high medium and low
        HIGH = 1 # every factor of 1
        MEDIUM = 2 # every factor of 2
        LOW = 3 # every factor of 3

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

    def pathPlanning(self,robot, b = (10,10)): # TODO remove default values - just for testing
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
        rewards[b[1]][b[0]] = 1
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
        while (not converged) :

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
                    utilities[i][j] = self.getReward(rewards, (j,i)) + gamma * maxUtility

            conv = conv + 1
            if (conv > 50):
                converged = True
        
        # print(utilities nicely)
        for i in range (0, len(utilities)):
            print(utilities[i])

        end = time.time()
        print("time taken to calculate utilities: ", end - start)
        #set utilites array to 0 with noise
        # update utilities based on neighbours
        # repeat until convergence


        


        # return optimal policy
        pass



# main method
if __name__ == '__main__':
    # --- Main Program ---
    rospy.init_node('waiter_robots_node')
    node = WaiterRobotsNode()
    rospy.spin()
    # print("hello world")
  