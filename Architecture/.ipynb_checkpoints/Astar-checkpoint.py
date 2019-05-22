import matplotlib.pyplot as plt
import numpy as np
import math



def heuristic( start, goal):
    #Use Chebyshev distance heuristic if we can move one square either
    #adjacent or diagonal
    D = 1
    D2 = 1
    dx = abs(start[0] - goal[0])
    dy = abs(start[1] - goal[1])
    #return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)
    return dx + dy
 
def get_vertex_neighbours( pos,grid):
    n = []
    #Moves allow link a chess king
    for dx, dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,1),(1,-1),(-1,-1)]:
        x2 = pos[0] + dx
        y2 = pos[1] + dy
        if x2 < 0 or x2 > grid.shape[0] -1 or y2 < 0 or y2 > grid.shape[1] -1 :
            continue
        n.append((x2, y2))
    return n
 
def move_cost(b, grid):
    return grid[b[0],b[1]] #Normal movement cost
 
def AStarSearch(start, end,grid):
    start=(start[1],start[0])
    end=(end[1],end[0])
    start = (int(round(start[0]/10)),int(round(start[1]/10)))
    end = (int(round(end[0]/10)),int(round(end[1]/10)))
    G = {} #Actual movement cost to each position from the start position
    F = {} #Estimated movement cost of start to end going via this position
 
    #Initialize starting values
    G[start] = 0 
    F[start] = heuristic(start, end)
 
    closedVertices = set()
    openVertices = set([start])
    cameFrom = {}
 
    while len(openVertices) > 0:
        #Get the vertex in the open list with the lowest F score
        current = None
        currentFscore = None
        for pos in openVertices:
            if current is None or F[pos] < currentFscore:
                currentFscore = F[pos]
                current = pos
 
        #Check if we have reached the goal
        if current == end:
            #Retrace our route backward
            path = [tuple(reversed(current))]
            while current in cameFrom:
                current = cameFrom[current]
                path.append(tuple(reversed(current)))
            path.reverse()
            return path, F[end], G #Done!
 
        #Mark the current vertex as closed
        openVertices.remove(current)
        closedVertices.add(current)
 
        #Update scores for vertices near the current position
        for neighbour in get_vertex_neighbours(current,grid):
            if neighbour in closedVertices: 
                continue #We have already processed this node exhaustively
            candidateG = G[current] + move_cost(neighbour,grid)
 
            if neighbour not in openVertices:
                openVertices.add(neighbour) #Discovered a new vertex
            elif candidateG >= G[neighbour]:
                continue #This G score is worse than previously found
 
            #Adopt this G score
            cameFrom[neighbour] = current
            G[neighbour] = candidateG
            H = heuristic(neighbour, end)
            F[neighbour] = G[neighbour] + H
 
    raise RuntimeError("A* failed to find a solution")


def find_vertice(path):
  #####################################"      MISE EN FORME DE LA TRAJEC POUR DES LIGNES DROITES
    listedecime = []
    listeangle = []
    virage = []
    dd = []
    XY =[]
    #for i in range(0, len(PATH), 10):
    # listedecime.append(PATH[i])
     
    listedecime = path
    
    for i in range(len(listedecime)-2):
      ax = listedecime[i][0]
      ay = listedecime[i][1]
      bx = listedecime[i+1][0]
      by = listedecime[i+1][1]
      cx = listedecime[i+2][0]
      cy = listedecime[i+2][1]
      v1 = ax*bx + ay*by
      AB = np.sqrt((ax - bx)*(ax - bx)+ (ay - by)*(ay - by))
      BC = np.sqrt((bx - cx)*(bx - cx) + (by - cy)*(by - cy))
      CA = np.sqrt((cx - ax)*(cx - ax) + (cy - ay)*(cy - ay))
      angle = math.acos((AB*AB + BC*BC - CA*CA) / (2 * AB * BC))
      angle = angle/np.pi * 360
      listeangle.append(angle)
      
      if (angle < 330):
        virage.append(1)
      else:
        virage.append(0)
      
    n = 0
    
    for i in range(len(virage)):
        if(virage[i] == 1):
            n=n+1
        else:
            if(n>1):
                dd.append(int(i-round(n/2)))
                n=0
            elif(n==1):
                dd.append(i)
                n=0
            else:
                n=0
    
    for i in dd:
      
      XY.append(path[i])
    XY.append(path[-1])
    
    return np.array(XY)*10

def checkpoint_corrector(robot,XY):
    
    R_pos= robot.get_position()
    x_robot_bis = R_pos[0] 
    y_robot_bis = R_pos[1] 
    theta_robot = R_pos[2]
    next_checkpoint = XY[0]
    v1 = (math.cos(math.radians(theta_robot)),math.sin(math.radians(theta_robot)))
    v2 = (next_checkpoint[0] - x_robot_bis , next_checkpoint[1] - y_robot_bis)
    v1_norm = math.sqrt(v1[0]*v1[0] + v1[1]*v1[1])
    v2_norm = math.sqrt(v2[0]*v2[0] + v2[1]*v2[1])
    
    theta = math.degrees(math.acos((v2[0]*v1[0]+v2[1]*v1[1]) / v2_norm))

    while(theta > 60):
        x_robot_bis =  x_robot_bis - 10 * math.cos(math.radians(theta_robot)) 
        y_robot_bis = y_robot_bis - 10* math.sin(math.radians(theta_robot))
        v2 = (next_checkpoint[0] - x_robot_bis , next_checkpoint[1] - y_robot_bis)
        v2_norm = math.sqrt(v2[0]*v2[0] + v2[1]*v2[1])
        theta = math.degrees(math.acos((v2[0]*v1[0]+v2[1]*v1[1]) / v2_norm))
        print(x_robot_bis,y_robot_bis,theta)
    XY = np.insert(XY,0, [x_robot_bis, y_robot_bis]).reshape((-1,2))
    return XY