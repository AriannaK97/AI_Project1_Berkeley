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
    #initializing a stack where the successors of the nodes will be stored
    successorsStack = Stack()
    #Creating an empty list in order to keep track of all the nodes we have already visited (fringe)
    exploredNodes = []
    #getting the starting state of the graph we search. The state is of type x,y
    #nodeState = problem.getStartState()
    newNode = node(problem)
    #in the succesorStack we store tuples which are like (nodeState, [Path])
    #the path at this point is 0 since we are at the starting node
    #iinitialize the frontier using the initial state of problem.
    #successorsStack.push((nodeState,[]))
    successorsStack.push(newNode)
    while not successorsStack.isEmpty():

        #currentNode, path = successorsStack.pop()
        #state, parent, action, path = successorsStack.pop()
        currentNode = node(successorsStack.pop())
        #puth the node in the list of the explored nodes
        exploredNodes.append(currentNode)

        #if the node is indeed a goal
        if problem.isGoalState(currentNode[0]):
            return path
        else:

            # in case the node is not our GoalState we call for its successors
            #expand the chosen node , adding the resulting nodes to the
            #frontier only if their state is not in the frontier or the
            #explored set

            successors = problem.getSuccessors(currentNode[0])

            for successor in successors:
                if successor[0] not in exploredNodes:
                    #calculating the new path from the starting Node to the successor node
                    # which is now in hand and about to be stored in the stack which will keep
                    #"feeding" the while loop either till the stack is empty or we reach our goal
                    nPath = path + [successor[1]]
                    successorsStack.push((successor[0], nPath))


    return None




    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    from util import Queue
    #initializing a queue where the successors of the nodes will be stored the queueu is the fringe
    fringe = Queue()
    #Creating an empty list in order to keep track of all the nodes we have already visited (frontier)
    exploredNodes = []
    #getting the starting state of the graph we search. The state is of type x,y
    nodeState = problem.getStartState()
    #in the succesorStack we store tuples which are like (nodeState, [Path])
    #the path at this point is 0 since we are at the starting node
    #iinitialize the frontier using the initial state of problem.
    fringe.push((nodeState,[]))

    while not fringe.isEmpty():

        currentNode, path = fringe.pop()
        #puth the node in the list of the explored nodes
        exploredNodes.append(currentNode)

        #if the node is indeed a goal
        if problem.isGoalState(currentNode):
            return path
        else:

            # in case the node is not our GoalState we call for its successors
            #expand the chosen node , adding the resulting nodes to the
            #frontier only if their state is not in the frontier or the
            #explored set

            successors = problem.getSuccessors(currentNode)

            for successor in successors:
                if successor[0] not in (exploredNodes or  frontier):
                    #calculating the new path from the starting Node to the successor node
                    # which is now in hand and about to be stored in the stack which will keep
                    #"feeding" the while loop either till the stack is empty or we reach our goal
                    nPath = path + [successor[1]]
                    fringe.push((successor[0], nPath))
    #util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    from util import PriorityQueue
    #initializing a stack where the successors of the nodes will be stored
    fringe = PriorityQueue()
    #Creating an empty list in order to keep track of all the nodes we have already visited (frontier)
    exploredNodes = []
    #getting the starting state of the graph we search. The state is of type x,y
    nodeState = problem.getStartState()
    #in the succesorStack we store tuples which are like (nodeState, [Path])
    #the path at this point is 0 since we are at the starting node
    #iinitialize the fringe using the initial state of problem.
    path  = 0
    fringe.push((nodeState,[]), path)

    while not fringe.isEmpty():

        currentNode, path = fringe.pop()
        #puth the node in the list of the explored nodes
        exploredNodes.append(currentNode)

        #if the node is indeed a goal
        if problem.isGoalState(currentNode):
            return path
        else:

            # in case the node is not our GoalState we call for its successors
            #expand the chosen node , adding the resulting nodes to the
            #frontier only if their state is not in the frontier or the
            #explored set

            successors = problem.getSuccessors(currentNode)

            for successor in successors:
                if successor[0] not in (exploredNodes or  fringe):
                    #calculating the new path from the starting Node to the successor node
                    # which is now in hand and about to be stored in the stack which will keep
                    #"feeding" the while loop either till the stack is empty or we reach our goal
                    nPath = path + [successor[1]]
                    fringe.push((successor[0], nPath), nPath)
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    from util import PriorityQueue
    #initializing a stack where the successors of the nodes will be stored
    fringe = PriorityQueue()
    #Creating an empty list in order to keep track of all the nodes we have already visited (frontier)
    exploredNodes = []
    #getting the starting state of the graph we search. The state is of type x,y
    nodeState = problem.getStartState()
    #in the succesorStack we store tuples which are like (nodeState, [Path])
    #the path at this point is 0 since we are at the starting node
    #iinitialize the fringe using the initial state of problem.
    heuristicPath  = heuristic
    path = []
    fringe.push((nodeState, path), heuristicPath)

    while not fringe.isEmpty():

        currentNode, path = fringe.pop()
        #puth the node in the list of the explored nodes
        exploredNodes.append(currentNode)

        #if the node is indeed a goal
        if problem.isGoalState(currentNode):
            return path
        else:

            # in case the node is not our GoalState we call for its successors
            #expand the chosen node , adding the resulting nodes to the
            #frontier only if their state is not in the frontier or the
            #explored set

            successors = problem.getSuccessors(currentNode)

            for successor in successors:
                if successor[0] not in (exploredNodes or  fringe):
                    #calculating the new path from the starting Node to the successor node
                    # which is now in hand and about to be stored in the stack which will keep
                    #"feeding" the while loop either till the stack is empty or we reach our goal
                    nPath = path + [successor[1]]
                    fringe.push((successor[0], nPath), heuristic)

    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
