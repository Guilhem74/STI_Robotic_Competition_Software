import matplotlib.pyplot as plt
import numpy as np
import math
import time

def dotproduct(v1, v2):
        return sum((a*b) for a, b in zip(v1, v2))

def length(v):
        return math.sqrt(dotproduct(v, v))
    
def find_path(grid,coord_objective,robot_pos):
    #Search a new checkpoint between X_robot and X_objective 
    #If there are not : Search a new checkpoint between Y_robot and Y_objective 
    X_objective , Y_objective ,_ = coord_objective
    X_robot , Y_robot, Angle_robot = robot_pos
    dist = length([X_objective - X_robot,Y_objective - Y_robot])
    if( Y_objective > 5000 and X_objective < 5000 ) :
        #Objective in top
        if(Y_Robot<5000 or X_Robot>5000):
            return [[X_objective,Y_robot,0]]
        else:
            return None
            
    elif( Y_robot > 5000 and X_robot < 5000 ):
        if(Y_objective < 5000 or X_objective > 5000):
            return [[X_robot,Y_objective,0]]
        else:
            return None
        
            
    else:
        return None
        
    
        
        
        
        
        
        

def get_path(grid,coord,robot_pos):
    x_r, y_r, angle_r = robot_pos
    x_c , y_c = coord
    
    
    theta = math.atan2(y_c - y_r , x_c - x_r) - math.radians(angle_r)
    distance = math.sqrt(( (x_c - x_r )*(x_c - x_r )+ (y_c - y_r)*(y_c - y_r) ))
    
    value , path_coord = get_map_information(robot_pos,grid,math.degrees(theta), distance)
    if(len(path_coord)==0):
        path_coord.append([int(round(robot_pos[0]/10)) , int(round(robot_pos[1]/10))])
        
        
    return value, path_coord


def get_segment(coord,grid,orientation,dist_max):
    
    

    x = round(coord[0])
    y = round(coord[1])
    path_coord = [[0,0]]
    detection_line = []
    for d in range(dist_max):
        
        if( y +round(orientation[1]* d) >0 and y +round(orientation[1]* d) < 799 
          and 0 +round(orientation[0]* d) >0 and 0 +round(orientation[0]* d)< 799):
            detection_line.append(grid[y +round(orientation[1]* d) ,x  +round( orientation[0]*  d)])
            path_coord.append( [x  +round( orientation[0]*  d),y +round(orientation[1]* d)])
    return detection_line , path_coord

def get_map_information(robot_pos,grid,theta, distance_detection):
    theta = math.radians(theta)
    
    distance = 20
    x_r = int(round(robot_pos[0]/10))
    y_r = int(round(robot_pos[1]/10))
    path_coord = np.array([[x_r,y_r]])
    list_maximum_lines = [0]
    orientation = (1,1)
    dist_max_detection = int(distance_detection/10)
    x0 = x_r + distance*math.cos(theta)
    x1 = x_r + distance*math.cos(theta)
    y0 = y_r + distance*math.sin(theta)
    y1 = y_r + distance*math.sin(theta)
    dx0 = math.cos(theta+(math.pi)/2)    
    dy0 = math.sin(theta+(math.pi)/2)
    dx1 = math.cos(theta-(math.pi)/2)
    dy1 = math.sin(theta-(math.pi)/2)
    orientation = (math.cos(theta),math.sin(theta))
    for i in range(distance-1):
        if(x0>0 and x0<799 and y0>0 and y0<799):
            result0 , path_coord0 = get_segment([x0,y0],grid,orientation,dist_max_detection)
            path_coord=np.insert(path_coord,0,np.array(path_coord0),axis=0)
            if(len(result0) >0):
                list_maximum_lines.append(max(result0))
        else:
            list_maximum_lines.append(255)
                
        if(x1>0 and x1<799 and y1>0 and y1<799):
            result1 , path_coord1 = get_segment([x1,y1],grid,orientation,dist_max_detection)
            path_coord=np.insert(path_coord,0,np.array(path_coord1),axis=0)
            if(len(result1) >0):
                list_maximum_lines.append(max(result1))
        else:
                list_maximum_lines.append(255)
        
        
        x0 = x0 + dx0
        x1 = x1 + dx1
        y0 = y0 + dy0
        y1 = y1 + dy1
    
        
        
    return max(list_maximum_lines), path_coord
        

        
        
        
    return max(list_maximum_lines)
        
        
        
def get_obstacle_robot(robot_position,grid_updated):
    f , _ = get_map_information(robot_position,grid_updated,0,1000)
    fl , _ =get_map_information(robot_position,grid_updated,45,1000)
    l , _ =get_map_information(robot_position,grid_updated,90,1000)
    bl , _ =get_map_information(robot_position,grid_updated,135,1000)
    b , _ =get_map_information(robot_position,grid_updated,180,1000)
    br , _ =get_map_information(robot_position,grid_updated,180+45,1000)
    r , _ =get_map_information(robot_position,grid_updated,270,1000)
    fr , _ =get_map_information(robot_position,grid_updated,270+45,1000)
    obstacle_robot = {"Front" :f,
                              "FrontLeft":fl ,
                              "Left":l,
                              "BackLeft":bl ,
                              "Back": b,
                              "BackRigth": br,
                              "Right": r,
                              "FrontRight":fr ,
                             }
    return obstacle_robot
    

        
        
    
        

    
    
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
    