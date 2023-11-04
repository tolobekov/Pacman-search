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
    return [s, s, w, s, w, w, s, w]


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


    startingNode = problem.getStartState()
    if problem.isGoalState(startingNode):
        return []

    myQueue = util.Stack()
    visitedNodes = []
    visitedCosts = []
    # (node,actions)
    myQueue.push((startingNode, []))

    while not myQueue.isEmpty():
        currentNode, actions = myQueue.pop()
        if currentNode not in visitedNodes:
            visitedNodes.append(currentNode)
            visitedCosts.append(0)
            if problem.isGoalState(currentNode):
                return actions, visitedNodes, visitedCosts

            for nextNode, action, cost in problem.getSuccessors(currentNode):
                newAction = actions + [action]
                myQueue.push((nextNode, newAction))


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""


    startingNode = problem.getStartState()
    if problem.isGoalState(startingNode):
        return []

    myQueue = util.Queue()
    visitedNodes = []
    visitedCosts = []
    # (node,actions)
    myQueue.push((startingNode, []))

    while not myQueue.isEmpty():
        currentNode, actions = myQueue.pop()
        if currentNode not in visitedNodes:
            visitedNodes.append(currentNode)
            visitedCosts.append(0)
            if problem.isGoalState(currentNode):
                return actions, visitedNodes, visitedCosts

            for nextNode, action, cost in problem.getSuccessors(currentNode):
                newAction = actions + [action]
                myQueue.push((nextNode, newAction))

    util.raiseNotDefined()



def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def manhattanHeuristic(position, problem, info={}):
    "The Manhattan distance heuristic for a PositionSearchProblem"
    # Retrieve the goal position
    goal = problem.goal

    # Compute the Manhattan distance
    return abs(position[0] - goal[0]) + abs(position[1] - goal[1])


def euclideanHeuristic(position, problem, info={}):
    "The Euclidean distance heuristic for a PositionSearchProblem"
    xy1 = position
    xy2 = problem.goal
    return ((xy1[0] - xy2[0]) ** 2 + (xy1[1] - xy2[1]) ** 2) ** 0.5

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    """
    Search the node that has the lowest combined cost and heuristic first.
    Returns:
        actions: The final route the agent must complete.
        visitedNodes: Nodes visited during the search process to find the optimal route.
        visitedCosts: The costs (heuristic + actual cost to get there) of the visited nodes.
    """

    # Initialize the priority queue
    frontier = util.PriorityQueue()

    # Add the starting point to the frontier with priority 0
    start_state = problem.getStartState()
    frontier.push((start_state, [], 0), 0 + heuristic(start_state, problem))

    # Set of explored nodes
    explored = set()
    visitedNodes = []  # To store nodes visited
    visitedCosts = []  # To store costs of nodes visited

    while not frontier.isEmpty():
        state, actions, g = frontier.pop()

        # If the state is already explored, continue to the next iteration
        if state in explored:
            continue

        # Store visited node and its cost
        visitedNodes.append(state)
        visitedCosts.append(g + heuristic(state, problem))  # g + h

        # If we've found the goal state, return the information
        if problem.isGoalState(state):
            return actions, visitedNodes, visitedCosts

        # Mark state as explored
        explored.add(state)

        for nextState, action, stepCost in problem.getSuccessors(state):
            # Calculate the new cost (g value) for this child
            new_g = g + stepCost
            # Calculate the f value of this child
            f = new_g + heuristic(nextState, problem)

            # If nextState hasn't been explored or is not in the frontier
            if nextState not in explored and not any(node[0] == nextState for node in frontier.heap):
                # Add the child to the frontier
                frontier.push((nextState, actions + [action], new_g), f)
            elif any(node[0] == nextState for node in frontier.heap):  # the state is in the frontier
                # Check if we have a better path to this state in the frontier
                # If so, update the cost and actions to get to this state
                for index, (p, c, i) in enumerate(frontier.heap):
                    if c[0] == nextState:
                        if f < i:
                            del frontier.heap[index]
                            frontier.push((nextState, actions + [action], new_g), f)
                        break

    # No solution found
    return [], visitedNodes, visitedCosts

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch

