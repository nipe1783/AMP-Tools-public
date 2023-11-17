```python
path planC(problem): # autonomous-car path planning function. 
#Inputs: problem: similar to that from the AMP-Toolbox. A problem consists of the car agent and the environment. 
#Outputs: path: a list of control inputs with time durations to perform in order to maneuver the car from start to goal.

    # initialize an empty path variable
    path = []
    # Create a tree for the sampling based method.
    tree = createTree()
    # initialize the tree root with the agents initial state.
    tree.root = problem.agent.init_state
    # initialize the desired goal state to later check if our sampled state is close enough to satisfy the exit condition.
    xGoal = problem.agent.goal_state
    # maximum number of allowed iterations. 
    n
    
    # continues iterating until a solution is found or the iteration counter is met.
    while counter < n:
        #increment iteration counter.
        counter++ 
        # generate a random state sample.
        xRand = generateRandomSample() 
        # check if the random sampled state is in an obstacle in the problem's state space. This function is described below.
        if validState(xRand, problem): 
            # find the nearest node in the tree to the randomly generated node.
            xNear = nearestNode(xRand, tree) 
            # generate random control values for the agent's acceleration and steering turn rate. This function is described below.
            u = generateRandomControl(xNear) 
            # generate a trajectory using the agent's dynamics, control inputs and a time duration. This function is described below.
            subTrajectory = generateLocalTrajectory(problem.agent.dynamics, xNear, u, delta, iterations)
            # check the validity of each state in the sub trajectory. This function is described below.
            if subTrajectoryIsValid(subTrajectory):
                # create a new node from the state at the end of the valid trajectory.
                xNew = subTrajectory[end]
                # add the new node to the tree
                tree.addNode(xNew)
                # create an edge object which connects from xNear->xNew. 
                edge = Edge(xNear, xNew)
                # assign the distance to the edge to be used by A*.
                edge.distance = distance(xNear, xNew)
                # assign the controls applied for this sub-trajectory to be used in the path variable.
                edge.controls = u
                # assign the time duration applied for this sub-trajectory to also be used in the path variable.
                edge.delta = delta
                # add the edge to the tree.
                tree.addEdge(edge)
                # check if the newest node added to the tree is within the threshold distance to the desired goal state.
                if distance(xNew, xGoal) < threshold:
                    # use A* to search the tree for the shortest path from the root node to the xNew node.
                    # A* will use the edge.distance field to determine the shortest path on the graph.
                    # A* will return a list of nodes and a list of edges that result in the shortest path.
                    nodes, edges = aStar.search(tree, xNew)
                    # loop through each edge in the A* result
                    for edge in edges:
                        # add controls and time duration delta to the path variable.
                        path.append([edge.controls, edge.delta])
                    # return path, success case.
                    return path
        
    # return path, failure case.
    return path

```

```python
u generateRandomSample(state): # generateRandomSample function. 
# Inputs: the state of the agent.
# Outputs: random controls with constraints defined by the autonomus-car problem.
    # if the velocity of the agent is <= 1/6 the gear = 1
    if state.v <= 1/6:
        # if the gear = 1 the acceleration range is from -1/6 to 1/6. generate a random control within this range.
        u1 = random(-1/6, 1/6) 
    # if the velocity of the agent is <= 2/6 and > 1/6 the gear = 2
    else if state.v <= 2/6:
        # if the gear = 2 the acceleration range is from -1/6 to 2/6. generate a random control within this range.
        u1 = random(-1/6, 2/6)
    # gear must be 3
    else:
        # gear = 3 the valid acceleration range is from -1/6 to 3/6. generate a random control within this range.
        u1 = random(-1/6, 3/6) 
    # generate a random steering turn rate control within the given range
    u2 = random(-pi/6, pi/6)
    # create a list containing the two controls.
    u = [u1, u2]
    # return the random controls
    return u

```

```python
subTrajectory generateLocalTrajectory(dynamics, state, u, delta, iterations): # generateLocalTrajectory function. 
# Inputs: the dynamics of the agent, the state of the agent, control input u, time duration delta and desired number of sub states in the output trajectory.
# Outputs: the trajectory. This is a list of states each with some time duration between each state.
    # initialize a trajectory list. 
    subTrajectory = []
    # loop for iterations number of times. 
    for i = 0 to iterations:
        # use the Runge-Kutta method to approximate the subsequent state after applying some controls u to a state for time duration.
        newState = rungeKutta(dynamics, state, u, delta/iterations)
        # append the resulting state from the Runge-Kutta method to the trajectory variable.
        subTrajectory.append(newState)
        # assign the state variable to newState so the next iteration will use the updated state for the Runge-Kutta method.
        state = newState
    # return the trajectory.
    return subTrajectory   
```

```python
newState rungeKutta(dynamics, state, u, delta): # rungeKutta function.
    # Runge-Kutta method described in lecture 14. 
    # Takes in an agent's dynamics, state, control input u and time duration delta.
    # Calculates an approximate subsequent state after applying the input parameters. 
    # For the self-driving car example it will utilize the continuous dynamics equations provided.
    # For example, (x_dot = v*vos(theta), y_dot = v*sin(theta), ...) are used to calculate a subsequent state. 
```

```python
bool isStateValid(state, problem):
    # check if the agent collides with obstacles in state space.
    # create a polygon representation of our agent based on the state and agent geometry.
    agentPolyon = createPolygon(problem.agent, state)
    # loop through each obstacle in the problem.
    for obstacle in problem obstacles:
        # loop through each edge in the obstacle polygon
        for obstacleLine in obstacle:
            # loop through each edge in the agent polygon
            for agentLine in agentPolyon:
                # if the two lines intersect the agent and obstacle are colliding. return false
                if intersects(obstacleLine, agentLine):
                    return false
    # check if the agents velocity is within the desired bounds.
    if state.v not inBounds(v):
        return false
    # check if the agents steering angle is within the desired bounds.
    if state.phi not inBounds(phi):
        return false
    # state is valid, return true
    return true

```

```python
bool subTrajectoryIsValid(subTrajectory, problem): # subTrajectoryIsValid function.
# Inputs: subTrajectory: list of states. problem: contains the environment and agent information.
# Outputs: boolean true for valid false for invalid.
    # Loop through each state in the trajectory.
    for state in subTrajectory:
        # if current state is in an obstacle in the problem's state space.
        if not isStateValid(state, problem):
            # the trajectory is not valid.
            return false
    # the trjactory is valid.
    return true
```


```python
paths planG(problem): # autonomous-car path planning function with possible tire-failure dynamics
#Inputs: problem: similar to that from the AMP-Toolbox. A problem consists of the car agent and the environment. 
#Outputs: paths: a list of possible agent paths depending on the dynamics after hitting a bump.
    # initialize empty path and paths variables.
    paths = [], path = []
    # loop through each bump in the environment
    for bump in problem.bumps
        # transform the bump object into an obstacle polygon
        bumpObstacle = createObstacle(bump)
        # add the bump obstacle to the list of obstacles in the problem
        problem.obstacles.append(bumpObstacle)
    # utilize motion planner 'planC' to solve the problem where bumps are also considered obstacles.
    path = planC(problem)
    # if the motion planner fails this means there is no solution without driving over bumps.
    if path.length() == 0:
        # remove all of the bumpObstacles from the environment. leaving only the original obstacles.
        problem.removeObstacles(bumpObstacles)
        # solve the problem using 'planC' while driving through bumps.
        # this path is valid under the condition that driving over bumps does not change the vehicle's dynamics.
        path = planC(problem)
        # append path to the possible paths list.
        paths.append(path)
        # if the agent has either no punctures or a single puncture, account for the change in dynamics.
        if problem.agent has less than two punctures:
            # initialize a state variable with the agent's initial state
            state = problem.agent.state_init
            # find the first state in the path that the agent runs over a bump. This function is described below.
            bumpHitState = getBumphitState(problem.bumps, path)
            # if the agent has no punctures account for both dynamic change possiblities.
            if agent has no punctures:
                # Copy the problem to a new variable to solve for a path with the single flat tire dynamics.
                # These problem's will account for all possibilities after running over a bump.
                # Note that I do not include the case of both tires popping at once.
                # This is because the dynamics of a car with two popped tires is the same as a car with no punctures.
                problemLP = problem.copy()
                problemRP = problem.copy()
                # set the agent's dynamics resulting from the new puncture.
                problemLP.agent.dynamics = lPDynamics
                problemRP.agent.dynamics = rPDynamics
                # set the agent's starting state to where the agent collided with the bump.
                problemLP.agent.state_init = bumpHitState
                problemRP.agent.state_init = bumpHitState
                # solve the problem given a left or right puncture. 
                # These paths will be partial path solutions from the bump hit state to the goal.
                partialPathsLP = planG(problemLP)
                partialPathsRP = planG(problemRP)
                # Additionally compose a partial path where the agent does not pop a tire on the current bump.
                # Note that this accounts for the case where the agent possibly pops a tire after hitting a future bump.
                partialPathsNP = planG(problem)
                # Compose entire paths by combining the relevant pre-change in dynamics path steps with the partialPath after puncture.
                # These paths will take the form of 'pathPrePuncture -> bumpState -> pathPostPuncture'.
                pathsLP = composeCompletePath(path, partialPathsLP, bumpHitState)
                pathsRP = composeCompletePath(path, partialPathsRP, bumpHitState)
                pathsNP = composeCompletePath(path, partialPathNP, bumpHitState)
                # Append paths with all paths produced by various dynamic possibilites.
                paths.append(each path in pathsLP), paths.append(each path in pathsRP), paths.append(each path in pathsNP)
            # if the agent only has a single puncture, the new tire pop will cause dynamics of a vehicle with two punctures.
            else:
                # copy the problem to a new variable to solve for a path with both flat tire dynamics.
                problemBP = problem.copy()
                # set the agent's dynamics according to the new puncture.
                problemBP.agent.dynamics = BPDynamics
                # solve the problem given both tires are punctured. 
                # These paths will be partial path solutions from the bump hit state to the goal.
                partialPathsBP = planG(problemBP)
                # Compose entire paths by combining the relevant pre-change in dynamics path steps with the partialPath after puncture.
                pathsBP = composeCompletePath(path, partialPathsBP, bumpHitState)
                # Append paths with all the produced double puncture paths.
                paths.append(each path in pathsBP)
    # planC algorithm was able to find a path without colliding with bump.
    else:
        paths.append(path)
    # return possible paths
    return paths
```

```python
bumpHitState = getBumphitState(problem.bumps, path, dynamics):
    # Loop through each node in the path.
    for node in path:
        # Calculate subTrajectory between each node with the generateLocalTrajectory function.
        subTrajectory = generateLocalTrajectory(dynamics, node.state, node.u, node.delta, node.iterations)
        # loop through each bump in the problem
        for bump in bumps:
            # determine if subTrajectory intersects bump in problem through polygon intersection methods.
            if subTrajectory intersects bump:
                # return the state nearest to the bump collison.
                return nearest(subTrajectory, bump)

```
