import matplotlib.pyplot as plt
import numpy as np
import math
import time


    
    

def get_segement(robot_pos,grid,theta):
    dist_max = 200


    x_r = int(round(robot_pos[0]/10))
    y_r = int(round(robot_pos[1]/10))
    angle_robot = robot_pos[2]
    up_right = []
    up_left = []
    down_right = []
    down_left = []
    up = []
    down = []
    left = []
    right = []
    for d in range(dist_max):
        
        up_right.append(grid[y_r - d ,x_r + d])
        up_left.append(grid[y_r + d ,x_r + d])
        down_right.append(grid[y_r - d ,x_r - d])
        down_left.append(grid[y_r + d ,x_r - d])
        
        down.append(grid[y_r  ,x_r - d])
        up.append(grid[y_r  ,x_r + d])
        left.append(grid[y_r + d ,x_r ])
        right.append(grid[y_r - d ,x_r ])
        

    if(0<theta<=22.5):
        return  up
    elif(22.5<theta<=45):
        return up_left
    elif(45<theta<=67.5):
        return left
    elif(67.5<theta<=90):
        return down_left
    elif(90<theta<=112.5):
        return down_right
    elif(112.5<theta<=135):
        return down
    elif(135<theta<=157.5):
        return right
    else:
        return up_right
    

    


    
    
def validate_objective(Set_Objective,grid, Get_Pos):
    x_robot = int(Get_Pos[0]/10)
    y_robot = int(Get_Pos[1]/10)
    #grid[x_robot-10:x_robot+10 , y_robot-10 : y_robot+10] = 0
    
    x, y = round(Set_Objective[0]/10) , round(Set_Objective[1]/10)
    d = 10
    
    if(grid[y,x]):
        return[x *10 , y*10]
    
    candidate = [[x,y],[x,y],[x,y],[x,y]]
    while( d < 700 ):
        candidate = [[x+d,y+d],[x+d,y-d],[x-d,y+d],[x-d,y-d]]
        for i in candidate:
            if i[0] > 0 and i[0] < 799 and i[1] > 0 and i[1] < 799:
                if grid[i[1],i[0]] == 0:
                    print("new coordinate: ",i[0] *10 , i[1]*10 )
                    return [i[0] *10 , i[1]*10]
            
        d = d + 20


        
"""    
    x = round(Get_Pos[0] / 10)
    y = round(Get_Pos[1] / 10)
    new_x = x
    new_y = y
    v2 = (Get_Pos[0]-Set_Objective[0],Get_Pos[1]-Set_Objective[1])
    norm_v2 = math.sqrt(v2[0]*v2[0]+v2[1]*v2[1])
    v2_normalized = (v2[0] / norm_v2, v2[1] / norm_v2)
    d = 0
    
    while(grid[ new_y,new_x] != 0  ):
        print("GRID3", grid[ new_y,new_x])
        print("X ,Y",new_x,new_y)
        d = d + 10
        new_x = x + round(d* v2_normalized[0])
        new_y = y + round(d* v2_normalized[1])
        if( d > 1000 or new_x >799 or  new_x <= 0 or new_y >799 or  new_y <= 0 ):
            print("Destination not available")
            return [Get_Pos[0],Get_Pos[1]]
    
    print("OBJECTIVE" ,new_x*10 ,new_y*10)
    return [new_x*10,new_y*10]
"""         
    
def heuristic( start, goal):
    #Use Chebyshev distance heuristic if we can move one square either
    #adjacent or diagonal
    D = 1
    D2 =2
    dx = abs(start[0] - goal[0])
    dy = abs(start[1] - goal[1])
    if dx > dy :
        return 14*dy + 10*(dx - dy)
    else:
        return 14*dx + 10*(dy - dx)
    #return D2 * math.sqrt(dx*dx + dy*dy) + D*(dx + dy)
    #return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)
    #return dx + dy
 
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
 
    
    return [end], None , None 


def find_vertice(path):
  #####################################"      MISE EN FORME DE LA TRAJEC POUR DES LIGNES DROITES
    if(len(path) < 2):
        return np.array(path)*10
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

def checkpoint_in_fov(robot,XY):
    #WORKING IN MILLIMETER HERE
    increment = 200 #Step of the search
    R_pos= robot.get_position()
    x_robot_bis = R_pos[0] 
    y_robot_bis = R_pos[1] 
    theta_robot = R_pos[2]
    v1 = (math.cos(theta_robot),math.sin(theta_robot))
    next_checkpoint = XY[0]
    #print(XY)
    v2 = (next_checkpoint[0] - x_robot_bis , next_checkpoint[1] - y_robot_bis)
    v1_norm = math.sqrt(v1[0]*v1[0] + v1[1]*v1[1])
    v2_norm = math.sqrt(v2[0]*v2[0] + v2[1]*v2[1])
    
    theta = math.degrees(math.acos((v2[0]*v1[0]+v2[1]*v1[1]) / v2_norm))  #Angle between robot orientation and next checkpoint
    
    i = 0
    while(theta > 80  and i < 3 and x_robot_bis <7999 and x_robot_bis>0 and y_robot_bis>0 and y_robot_bis<0): #60 is the maximum angle to make sure the robot go forward to this objective
        x_robot_bis =  x_robot_bis - increment * math.cos(math.radians(theta_robot)) 
        y_robot_bis = y_robot_bis - increment* math.sin(math.radians(theta_robot))
        v2 = (next_checkpoint[0] - x_robot_bis , next_checkpoint[1] - y_robot_bis)
        v2_norm = math.sqrt(v2[0]*v2[0] + v2[1]*v2[1])
        theta = math.degrees(math.acos((v2[0]*v1[0]+v2[1]*v1[1]) / v2_norm))
        print(x_robot_bis,y_robot_bis,theta)
     #Makes the robot go backward in straight line until he see the next checkpoint in it's field of view
        i = i+1
        if(i >= 3 ) :
            print("backward")
            R_pos= robot.get_position()
            return  R_pos[0]-200 * math.cos(math.radians(theta_robot)) ,R_pos[1]-200 *math.sin(math.radians(theta_robot)),R_pos[2] 
    
    return x_robot_bis , y_robot_bis,theta_robot



def checkpoint_corrector(XY):
    new_XY = []
    step = np.flip(XY)
    new_XY.append(step[0])

    step = np.delete(step,(0), axis=0)
    

    while(len(step) > 1):
        vec = (new_XY[-1][0]-step[0][0], new_XY[-1][1]-step[0][1])
        vec_norm = math.sqrt(vec[0]*vec[0] + vec[1]*vec[1])
        
        if(vec_norm >= 100):
            new_XY.append(step[0])
            step = np.delete(step,(0), axis=0)
        else:
            step = np.delete(step,(0), axis=0)


    return np.flip(np.array(new_XY))       
    