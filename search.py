# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]



###############################################################################################
###############################################################################################
###############################################################################################
def node(problem, parent = None, action = None, path = 0):
    STATE = problem.problem.getStartState()
    PARENT = parent
    ACTION = action
    PATH_COST = parent.path + problem.stepCost(parent.STATE, action)
    return (STATE, PARENT, ACTION, PATH_COST)
###############################################################################################
###############################################################################################
###############################################################################################

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    from util import Stack

    fringe = Stack()

    exploredNodes = []  #All visited states
    path = []   #the path followed from the start node

    nodeState = problem.getStartState()

    if problem.isGoalState(nodeState):
        return path

    #in the succesorStack we store tuples which are like (nodeState, [Path])
    #the path at this point is 0 since we are at the starting node
    #iinitialize the frontier using the initial state of problem.
    fringe.push((nodeState,path))

    while (True):

        if fringe.isEmpty():
            return []

        currentNode, path = fringe.pop()
        exploredNodes.append(currentNode)       #put the currentNode in the list of the explored nodes

        #uncomment lines 133 and 134 so that the program works properly with the autograder
        #and comment lines 144-145

        if problem.isGoalState(currentNode):    #if the node is indeed a goal
            return path

        #get currentNode's successors
        successors = problem.getSuccessors(currentNode)

        if successors:
            for successor in successors:
                if successor[0] not in exploredNodes:

                    #comment here for the program to work right with the autograder
                    #if problem.isGoalState(successor[0]):
                    #   return path + [successor[1]]

                    nPath = path + [successor[1]]
                    fringe.push((successor[0], nPath))



def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    from util import Queue

    fringe = Queue()

    exploredNodes = []  #All visited states
    path = []   #the path followed from the start node

    nodeState = problem.getStartState()

    if problem.isGoalState(nodeState):
        return path

    #in the succesorStack we store tuples which are like (nodeState, [Path])
    #the path at this point is 0 since we are at the starting node
    #iinitialize the frontier using the initial state of problem.
    fringe.push((nodeState,path))

    while (True):

        if fringe.isEmpty():
            return []

        currentNode, path = fringe.pop()
        exploredNodes.append(currentNode)       #put the currentNode in the list of the explored nodes

        #uncomment lines 182 and 183 so that the program works properly with the autograder

        if problem.isGoalState(currentNode):    #if the node is indeed a goal
            return path

        #get currentNode's successors
        successors = problem.getSuccessors(currentNode)

        if successors:
            for successor in successors:
                if successor[0] not in exploredNodes and successor[0] not in (state[0] for state in fringe.list):
                    #comment here for the program to work right with the autograder
                    #if problem.isGoalState(successor[0]):
                     #  return path + [successor[1]]

                    nPath = path + [successor[1]]
                    fringe.push((successor[0], nPath))



def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    from util import PriorityQueue

    fringe = PriorityQueue()


    exploredNodes = []  #All visited states
    path = []   #the path followed from the start node

    nodeState = problem.getStartState()

    #if problem.isGoalState(nodeState):
        #return path

    #the path cost at this point is 0 since we are exploring the first node of the graph
    priority = 0
    fringe.push((nodeState,path),priority)

    while (True):

        if fringe.isEmpty():
            return []

        currentNode, path = fringe.pop()
        exploredNodes.append(currentNode)       #put the currentNode in the list of the explored nodes

        #uncomment lines 182 and 183 so that the program works properly with the autograder

        if problem.isGoalState(currentNode):    #if the node is indeed a goal
            return path

        #get currentNode's successors
        successors = problem.getSuccessors(currentNode)

        if successors:
            for successor in successors:
                if (successor[0] not in exploredNodes and successor[0] not in (state[2][0] for state in fringe.heap)):

                    #comment here for the program to work right with the autograder
                    #if problem.isGoalState(successor[0]):
                    #   return path + [successor[1]]

                    nPath = path + [successor[1]]
                    priority = problem.getCostOfActions(nPath)
                    fringe.push((successor[0], nPath), priority)

                #in case the node already exists in the fringe we check which one has the highest priority/path cost and
                #peak the lowest one
                elif (successor[0] not in exploredNodes and successor[0] in (state[2][0] for state in fringe.heap)):
                    for state in fringe.heap:
                        if (state[2][0] == successor[0]):
                            fNodePriority = problem.getCostOfActions(state[2][1])
                    succPriority = problem.getCostOfActions(path + [successor[1]])

                    if(fNodePriority > succPriority):
                        nPath = path + [successor[1]]
                        fringe.update((successor[0], nPath), succPriority)



def f(state, problem, heuristic):
    return 0

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    from util import PriorityQueue

    fringe = PriorityQueue()


    exploredNodes = []  #All visited states
    path = []   #the path followed from the start node

    nodeState = problem.getStartState()

    #if problem.isGoalState(nodeState):
        #return path

    #the path cost at this point is 0 since we are exploring the first node of the graph
    priority = problem.getCostOfActions(path) + heuristic(nodeState, problem)
    fringe.push((nodeState,path),priority)

    while (True):

        if fringe.isEmpty():
            return []

        currentNode, path = fringe.pop()

        if currentNode in exploredNodes:
            continue

        exploredNodes.append(currentNode)       #put the currentNode in the list of the explored nodes

        #uncomment lines 182 and 183 so that the program works properly with the autograder

        if problem.isGoalState(currentNode):    #if the node is indeed a goal
            return path

        #get currentNode's successors
        successors = problem.getSuccessors(currentNode)

        if successors:
            for successor in successors:
                if (successor[0] not in exploredNodes):

                    #comment here for the program to work right with the autograder
                    #if problem.isGoalState(successor[0]):
                    #   return path + [successor[1]]

                    nPath = path + [successor[1]]
                    priority = problem.getCostOfActions(nPath) + heuristic(successor[0],problem)
                    fringe.push((successor[0], nPath), priority)

                #in case the node already exists in the fringe we check which one has the highest priority/path cost and
                #peak the lowest one
                elif (successor[0] not in exploredNodes):
                    for state in fringe.heap:
                        if (state[2][0] == successor[0]):
                            fNodePriority = problem.getCostOfActions(state[2][1])
                    succPriority = problem.getCostOfActions(path + [successor[1]]) + heuristic(successor[0],problem)

                    if(fNodePriority > succPriority):
                        nPath = path + [successor[1]]
                        fringe.update((successor[0], nPath), succPriority)


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
