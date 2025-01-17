class GridWorldState():
    
    def __init__(self, coord, cost, obstacle_coords = [], cost_map = []):
        self.ncol = self.nrow = 9
        self.id = coord
        self.coord = coord 
        self.cost = cost 
        self.total_cost = cost
        # Ecercise 3.1
        self.obstacle_coords = obstacle_coords
        # Exercise 3.2
        self.cost_map = cost_map
        
    def step(self, action):
        row, col = self.coord
        next_row, next_col = row, col

        if action == 'L': next_col = max(col-1, 0)
        elif action == 'R': next_col = min(col+1, self.ncol-1)
        elif action == 'U': next_row = max(row-1, 0)
        elif action == 'D': next_row = min(row+1, self.nrow-1)

        next_state_coord = (next_row, next_col)

        # Exercise 3.1
        if next_state_coord in self.obstacle_coords:
            next_state_coord = self.coord

        # Exercise 3.2
        next_state_cost = 0
        if self.cost_map:
            next_state_cost = self.cost_map[next_state_coord[0]][next_state_coord[1]]
        
        next_state = GridWorldState(next_state_coord, next_state_cost, self.obstacle_coords, self.cost_map)
        return next_state

    def estimate_cost_to_go(self, goal):
        cost_to_go_estimate = abs(goal.coord[0] - self.coord[0])
        cost_to_go_estimate += abs(goal.coord[1] - self.coord[1])
        return cost_to_go_estimate

    def __eq__(self, other):
        return self.id==other.id

    def __lt__(self, other):
        return self.total_cost < other.total_cost


    
    
    
    
    fringe = queue.PriorityQueue()
    fringe.put(start)
    path = {start.id: []} # a dict of `vertex: actions`

    explored = {start.id: start.cost} # a dict of `vertex: cost_so_far`
    reachedEnd = False
    while not fringe.empty():
        current = fringe.get()
        if current==goal:
            print ("reached the end, jolly good")
            print (path[current.id])
            print (explored[current.id])
            break
        for action in actionset: # "simulate" executing actions
            neighbor = current.step(action)
            cost_so_far = explored[current.id] + current.cost
            # neighbour not explored or update the neighbour
            if (neighbor.id not in explored) or (cost_so_far < explored[neighbor.id]):
                explored[neighbor.id] = cost_so_far
                path[neighbor.id] = path[current.id] + [action]
                vfp = cost_so_far + neighbor.estimate_cost_to_go(goal)
                neighbor.total_cost = vfp
                fringe.put(neighbor)
