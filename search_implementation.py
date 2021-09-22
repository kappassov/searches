def heuristic(pos, goal):
    #Chebyshev distance from current position to the goal
    return max(abs(pos[0] - goal[0]),abs(pos[1] - goal[1]))

def get_neighbors(pos):
    neighbors = []
    for ix, iy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,1),(1,-1),(-1,-1)]:
        nx = pos[0] + ix
        ny = pos[1] + iy
        if nx < 1 or nx > 8 or ny < 1 or ny > 8:
            continue
        neighbors.append((nx,ny))
    return neighbors

def move_cost(next_node, barrier):
    for bar_pos in barrier:
        if next_node in bar_pos:
            return 200
    return 1 

def UCSearch(start, goal, barrier): 
    G = {} 
    G[start] = 0

    expandedNodes = set() 
    fringe = set([start]) 
    parent = {} 
    
    while len(fringe) > 0:
        curNode = None 
        curGscore = None 
        
        for pos in fringe:
            if curNode is None or G[pos] < curGscore:
                curGscore = G[pos]
                curNode = pos
        fringe.remove(curNode)
        
        if curNode == goal:
            path = [curNode]
            while curNode in parent:
                curNode = parent[curNode]
                path.append(curNode)
            path.reverse()
            return path, expandedNodes, G[goal] #Done!
        
        if(curNode not in expandedNodes):
            expandedNodes.add(curNode)
            neighbors = get_neighbors(curNode)
            for neighbor in neighbors:
                if (neighbor not in fringe and neighbor not in expandedNodes):
                    fringe.add(neighbor)
                    G[neighbor] = curGscore + move_cost(neighbor, barrier)
                    if(neighbor not in parent):
                        parent[neighbor] = curNode 
                elif(curGscore + move_cost(neighbor, barrier) < G[neighbor] and neighbor not in expandedNodes):
                    G[neighbor] = curGscore + move_cost(neighbor, barrier)
                    if(neighbor not in parent):
                        parent[neighbor] = curNode
                    
    raise RuntimeError("UCS failed to find a solution")
    
def aStarSearch(start, goal, barrier):
    G = {} 
    F = {} 
    
    G[start] = 0
    F[start] = heuristic(start, goal)
    
    expandedNodes = set() 
    fringe = set([start]) 
    parent = {} 
    
    while len(fringe) > 0:
        curNode = None
        curFscore = None 
        
        for pos in fringe:
            if curNode is None or F[pos] < curFscore:
                curFscore = F[pos]
                curNode = pos
        fringe.remove(curNode)
        
        if curNode == goal:
            path = [curNode]
            while curNode in parent:
                curNode = parent[curNode]
                path.append(curNode)
            path.reverse()
            return path, expandedNodes, F[goal] #Done!
        
        if(curNode not in expandedNodes):
            expandedNodes.add(curNode)
            neighbors = get_neighbors(curNode)
            for neighbor in neighbors:
                if (neighbor not in fringe and neighbor not in expandedNodes):
                    fringe.add(neighbor)
                    G[neighbor] = G[curNode] + move_cost(neighbor, barrier)
                    F[neighbor] = G[neighbor] + heuristic(neighbor, goal)
                    if(neighbor not in parent ):
                        parent[neighbor] = curNode 
                elif(G[curNode] + move_cost(neighbor, barrier) < G[neighbor] and neighbor not in expandedNodes):
                    G[neighbor] = G[curNode] + move_cost(neighbor, barrier)
                    if(neighbor not in parent ):
                        parent[neighbor] = curNode
    raise RuntimeError("A* failed to find a solution")