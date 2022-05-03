import util

class SearchProblem:
    def getStartState(self):
        util.raiseNotDefined()
    def isGoalState(self, state):
        util.raiseNotDefined()
    def getSuccessors(self, state):
        util.raiseNotDefined()
    def getCostOfActions(self, actions):
        util.raiseNotDefined()

def depthFirstSearch(problem):
    frontier = util.Stack()
    exploredNodes = []
    startState = problem.getStartState()
    startNode = (startState, [])
    frontier.push(startNode)
    while not frontier.isEmpty():
        currentState, actions = frontier.pop()
        if currentState not in exploredNodes:
            exploredNodes.append(currentState)
            if problem.isGoalState(currentState):
                return actions
            else:
                successors = problem.getSuccessors(currentState)
                for succState, succAction, succCost in successors:
                    newAction = actions + [succAction]
                    newNode = (succState, newAction)
                    frontier.push(newNode)
    return actions  

def breadthFirstSearch(problem):
    frontier = util.Queue()
    exploredNodes = []
    startState = problem.getStartState()
    startNode = (startState, [], 0)
    frontier.push(startNode)
    while not frontier.isEmpty():
        currentState, actions, currentCost = frontier.pop()
        if currentState not in exploredNodes:
            exploredNodes.append(currentState)
            if problem.isGoalState(currentState):
                return actions
            else:
                successors = problem.getSuccessors(currentState)
                for succState, succAction, succCost in successors:
                    newAction = actions + [succAction]
                    newCost = currentCost + succCost
                    newNode = (succState, newAction, newCost)
                    frontier.push(newNode)
    return actions
        
def uniformCostSearch(problem):
    frontier = util.PriorityQueue()
    exploredNodes = {}
    startState = problem.getStartState()
    startNode = (startState, [], 0)
    frontier.push(startNode, 0)
    while not frontier.isEmpty():
        currentState, actions, currentCost = frontier.pop()
        if (currentState not in exploredNodes) or (currentCost < exploredNodes[currentState]):
            exploredNodes[currentState] = currentCost
            if problem.isGoalState(currentState):
                return actions
            else:
                successors = problem.getSuccessors(currentState)
                for succState, succAction, succCost in successors:
                    newAction = actions + [succAction]
                    newCost = currentCost + succCost
                    newNode = (succState, newAction, newCost)
                    frontier.update(newNode, newCost)
    return actions

def nullHeuristic(state, problem=None):
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    frontier = util.PriorityQueue()
    exploredNodes = []
    startState = problem.getStartState()
    startNode = (startState, [], 0)
    frontier.push(startNode, 0)
    while not frontier.isEmpty():
        currentState, actions, currentCost = frontier.pop()
        currentNode = (currentState, currentCost)
        exploredNodes.append((currentState, currentCost))
        if problem.isGoalState(currentState):
            return actions
        else:
            successors = problem.getSuccessors(currentState)
            for succState, succAction, succCost in successors:
                newAction = actions + [succAction]
                newCost = problem.getCostOfActions(newAction)
                newNode = (succState, newAction, newCost)
                already_explored = False
                for explored in exploredNodes:
                    exploredState, exploredCost = explored
                    if (succState == exploredState) and (newCost >= exploredCost):
                        already_explored = True
                if not already_explored:
                    frontier.push(newNode, newCost + heuristic(succState, problem))
                    exploredNodes.append((succState, newCost))
    return actions


bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
