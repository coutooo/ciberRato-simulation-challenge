from heapq import *

def heuristic(a, b):
    return (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2

def astar(array, start, goal):

    neighbors = [(0,2),(0,-2),(2,0),(-2,0)]

    close_set = set()
    came_from = {}
    g = {start:0}
    f = {start:heuristic(start, goal)}
    oheap = []
    c=0
    heappush(oheap, (f[start], start))
    
    while oheap:

        current = heappop(oheap)[1]
        # print(current)

        if current == goal:
            data = []
            c+=1
            while current in came_from:
                
                data.append(current)
                current = came_from[current]
            #data.append(start)
            # print(c)
            return [start] + data[::-1]

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j            
            tentative_g_score = g[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < len(array):
                if 0 <= neighbor[1] < len(array[0]):                
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue
                
            if neighbor in close_set and tentative_g_score >= g.get(neighbor, 0):
                continue
                
            if  tentative_g_score < g.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                g[neighbor] = tentative_g_score
                f[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heappush(oheap, (f[neighbor], neighbor))
           
    return None