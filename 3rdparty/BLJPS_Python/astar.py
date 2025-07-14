import math, heapq, time

def blocked(cX,cY,dX,dY,matrix):
    if cX+dX <0 or cX+dX>=matrix.shape[0]:
        return True
    if cY+dY < 0 or cY+dY >=matrix.shape[1]:
        return True
    if dX != 0 and dY != 0:
        if matrix[cX+dX][cY]!=0 and matrix[cX][cY+dY]!=0:
            return True
        if matrix[cX+dX][cY+dY] !=0:
            return True
    else:
        if dX != 0:
            if matrix[cX+dX][cY]!=0:
                return True
        else:
            if matrix[cX][cY+dY]!=0:
                return True
    return False

def heuristic(a, b):
    xdist = math.fabs(b[0] - a[0])
    ydist = math.fabs(b[1] - a[1])
    return xdist+ ydist
 
def method(matrix,start,goal):
    close_set = set()
    came_from = {}
    start = start[1],start[0] # swap x and y coordinates because of numpy internals
    goal = goal[1],goal[0] # swap x and y coordinates because of numpy internals
    gscore = {start:0}
    fscore = {start:heuristic(start,goal)}

    pqueue = []

    heapq.heappush(pqueue, (fscore[start],start))

    while pqueue:

        current = heapq.heappop(pqueue)[1]
        if current == goal:
            path = []
            while current in came_from:
                path.append((current[1],current[0])) # revert swap x and y coordinates because of numpy internals
                current = came_from[current]
            path.append((start[1],start[0])) # revert swap x and y coordinates because of numpy internals
            path = path[::]
            return path
                

        close_set.add(current)
        for dX, dY in [(0,1),(0,-1),(1,0),(-1,0)]: #[(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]:
            
            if blocked(current[0],current[1],dX,dY,matrix):
                continue

            neighbour = current[0] + dX, current[1] + dY

            if dX!=0 and dY!=0:
                tentative_g_score = gscore[current] + 14
            else:
                tentative_g_score = gscore[current] + 10
            
            if neighbour in close_set: #and tentative_g_score >= gscore.get(neighbour,0):
                continue
                
            if  (tentative_g_score < gscore.get(neighbour,0) or
                 neighbour not in [i[1]for i in pqueue]):
                came_from[neighbour] = current
                gscore[neighbour] = tentative_g_score
                fscore[neighbour] = tentative_g_score + heuristic(neighbour,goal)
                heapq.heappush(pqueue, (fscore[neighbour], neighbour))
    return []
