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
    # Create stack and visited list
    DFSStack = util.Stack()
    DFSVisited = list()

    # Find the start state and create a node for the start state
    startState = problem.getStartState()
    startNode = (startState, [])

    # Push first node onto the DFS Stack
    DFSStack.push(startNode)
    
    # While stack is not empty
    while (not DFSStack.isEmpty()):
        # Pop a node of the stack
        state, directions = DFSStack.pop()
        
        # If state hasn't been visited then expand
        if state not in DFSVisited:
            # Add state to visited
            DFSVisited.append(state)

            # If we have reached goal then return directions to goal state
            # Otherwise find expand the node
            if problem.isGoalState(state):
                return directions
            else:
                successorsFromExpansion = problem.getSuccessors(state)
                
                # For each expanded node find the state and action to get there and run
                # the DFS on that node
                for nextState, nextAction, unneccessaryCost in successorsFromExpansion:
                    nextDirection = directions + [nextAction]
                    nextNode = (nextState, nextDirection)
                    DFSStack.push(nextNode)

    #util.raiseNotDefined()
    

def breadthFirstSearch(problem):
    "*** YOUR CODE HERE ***"
    # Create queue and visited list
    # Find the start state and create a node for the start state
    queue = util.Queue()
    start_state = problem.getStartState()
    visited = []
    queue.push((start_state, []))             # Add start state and directions to queue

    # While queue is not empty
    while (not queue.isEmpty()):
        # Pop a node off the queue
        visiting = queue.pop()
        directions, node = visiting[1], visiting[0]

        # If state hasn't been visited then expand
        if node in visited:
            continue
        else:
            is_goal_state = problem.isGoalState(node)

            # If we have reached goal then return directions to goal state
            # Otherwise find expand the node
            if is_goal_state:
                return directions
            else:
                node_successors = problem.getSuccessors(node)

                # For each expanded node find the state and action total cost getting to
                # that state from previous path to get there and run the BFS on that node
                for successor in node_successors:
                    next_dir, next_state = successor[1], successor[0]
                    queue.push((next_state, directions + [(next_dir)]))
                visited.append(node)
        
    return []
                

            

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    # Create prioirty queue and visited list
    UCSPQueue = util.PriorityQueue()
    UCSVisited = dict()

    # Find the start state and create a node for the start state
    startState = problem.getStartState()
    startNode = (startState, [], 0)
    
    # Push first node onto the UCS queue
    UCSPQueue.update(startNode, 0)
    
    # While queue is not empty
    while (not UCSPQueue.isEmpty()):
        # Pop a node of the priority queue
        state, directions, cost = UCSPQueue.pop()
        
        # If state hasn't been visited then expand
        # or if the cost of the node is less than
        # the cost of the previous state
        if state not in UCSVisited or cost < UCSVisited[state]:
            # Set the cost of the state when it is visited
            UCSVisited[state] = cost

            # If we have reached goal then return directions to goal state
            # Otherwise find expand the node
            if problem.isGoalState(state):
                return directions
            else:
                successorsFromExpansion = problem.getSuccessors(state)
                
                # For each expanded node find the state and action total cost getting to
                # that state from previous path to get there and run the UCS on that node
                for nextState, nextAction, nextCost in successorsFromExpansion:
                    nextDirection = directions + [nextAction]
                    totalCost = cost + nextCost
                    nextNode = (nextState, nextDirection, totalCost)
                    UCSPQueue.update(nextNode, totalCost)

#    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    # Create prioirty queue and visited list
    # Find the start state and create a node for the start state
    queue = util.PriorityQueue()
    queue.push((problem.getStartState(),  [] ), 0)
    visited = []

     # While queue is not empty
    while not queue.isEmpty():
        # Pop a node of the priority queue
        current = queue.pop()
        directions, node  = current[1], current[0]

        # If state hasn't been visited then expand
        if node not in visited:

            # If we have reached goal then return directions to goal state
            # Otherwise find expand the node
            if problem.isGoalState(node):
                return directions
            else:
                # Set state to visited
                visited.append(node)
                successors = problem.getSuccessors(node)

                # For each expanded node find the state and action total cost getting to
                # that state from previous path to get there and run the A* search on that node
                for successor in successors:
                    next_direction, next_state = successor[1], successor[0]
                    new_directions = directions + [ (next_direction) ]

                    queue.push((next_state, new_directions), problem.getCostOfActions(new_directions) + heuristic( next_state, problem ) )
        else:
            continue


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
