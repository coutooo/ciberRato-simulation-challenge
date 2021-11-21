from heapq import *

def heuristic(a, b):
    return abs(b[0] - a[0]) + abs(b[1] - a[1]) 

def astar(start, goal,visited,walls):

    neighbors = [(0,2),(0,-2),(2,0),(-2,0)]

    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []

    heappush(oheap, (fscore[start], start))
    
    while oheap:

        current = heappop(oheap)[1]

        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            #data.append(start)
            return data

        close_set.add(current)
        for i, k in neighbors:
            neighbor = current[0] + i, current[1] + k            
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            
            if neighbor not in visited:
                continue

            # Make sure walkable terrain
            if i == 2 and ((current[0]+1,current[1]) in walls):
                continue
            if i == -2 and ((current[0]-1,current[1]) in walls):
                continue
            if k == 2 and ((current[0],current[1]+1) in walls):
                continue
            if k == -2 and ((current[0],current[1]-1) in walls):
                continue

                
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
                
            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heappush(oheap, (fscore[neighbor], neighbor))
           
    return None


# arr =  [(2, 0), (2, 2), (4, 2), (0, 2), (-2, 2), (-4, 2)]
# walls = [(0, 1), (1, 2), (3, 2), (-2, 1), (-2, 3), (2, -1), (5, 2), (2, 1), (-4, 3), (-5, 2), (2, 3), (-4, 1), (4, 3), (0, -1), (0, 3), (4, 1)]
# start = -4,2
# goal = 0,2
# print(astar(start,goal,arr,walls))
