
import numpy as np
import math
import  time, sys , serial
import threading
from collections import deque
from Serial_ import *
from parse import *

SERIAL_PORT_ROBOT = '/dev/ttyACM0'
SERIAL_PORT_COllECTOR = '/dev/ttyACM1'
SERIAL_RATE = 115200
class Robot_Class:
    def __init__(self,x=0,y=0,a=0):
        self.X_Pos = x
        self.Y_Pos = y
        self.Angle_Deg = a
        self.Robot_Speed=750
        self.map= Map()
        self.Try_Single_Goal=0
        self.Max_Try_Single_Goal=2
        self.ir_sensors_state = {
            'a':0,
            'b':0,
            'c':0,
            'd':0,
            'e':0,
            'f':0,
            'g':0,
            'h':0,
            'i':0,
            'j':0,
            'k':0,
            'l':0,
            'm':0}
        self.buf_broadcast = deque([]) # Queue of message for broadcasting via TCP IP
        self.buf_stm = deque([])
        self.ser = serial.Serial(SERIAL_PORT_ROBOT, SERIAL_RATE) # Connection to STM32 ROBOT
        self.ser_Collector= serial.Serial(SERIAL_PORT_COllECTOR,115200)
        self.serial_T = serialTread(self.ser)
        threading.Thread(target=self.serial_T.run, args=(self.buf_broadcast,self.buf_stm)).start()#create serial thread for robot control
    def Send_Messages(self,All_Commands):
        for string_ in All_Commands:
            self.buf_stm.append(string_)#Send every messages
    def Get_Read_Buffer(self):
        return self.buf_broadcast
    def Clear_Read_Buffer(self):
        self.buf_broadcast.clear()
        return
    def Collect_Message(self):
        if(len(self.buf_broadcast)):
            return self.buf_broadcast.popleft()
        return None
    def Wait_For_A_Message(self, Timeout):#Timeout in seconds
        Start_Time=time.time()
        while(True):
            Message=self.Collect_Message()
            if((time.time()-Start_Time)>Timeout and Message==None):
                return None
            elif(Message !=None):
                return Message
    def Wait_For_M3_Answer(self, Timeout):
        Start_Time=time.time()
        while(True):
            Message=self.Collect_Message()
            if((time.time()-Start_Time)>Timeout and Message==None):
                return None, None
            elif(Message is not None):
                Valid, Output= self.Is_it_M3_Answer(Message)
                print(Message)
                if(Valid is not False):
                    return Valid, Output
    def Is_it_M3_Answer(self, Message):
        if(Message is None):
            return False, None
        str2=parse(" M0 X{} Y{} A{} T{} S{}\r\n",Message)
        if(str2 is not None):
            array_argument=np.zeros(5)
            for i in range(5):
                array_argument[i]=float(str2[i])
            self.Set_Robot_Position(array_argument[0],array_argument[1],array_argument[2])
            feedback_sensors = int(array_argument[4])# feedback sensor MLKJIHGFEDCBA 
            self.Set_Sensor_State(feedback_sensors)
            if array_argument[3]==0: #T0 argument Arrived or sensor
                return True, 'Arrived'
            elif array_argument[3]==1:
                return True, 'Timeout'
            elif array_argument[3]==2:
                return True, 'Blocked'
            
        return False, None
    def Go_To(self,X,Y,R,level=0,Max_Level=3, Avoidance=True):
        """WARNING: Recursive function"""
        if (level==0):
            self.Reset_Nb_Try_Goal()
        print('Going to', str(X), ' ', str(Y), 'with R', str(R), 'level', str(level))
        X_Robot , Y_Robot, A_Robot = self.Get_Robot_Position()
        Timeout = (math.sqrt((X-X_Robot)*(X-X_Robot)+(Y-Y_Robot)*(Y-Y_Robot))/self.Robot_Speed)*1000
        Set_Coordinate=[X,Y,self.Robot_Speed,max(Timeout*1.2,1000),R]
        while(level<=Max_Level and self.Keep_Trying_Goal()):
            self.Send_Go_To_Coordinate(Set_Coordinate)
            self.Increment_Nb_Try_Goal()
            M0_State, Output=self.Wait_For_M3_Answer((Timeout*1.2+2000)/1000)
            if(M0_State== False):
                print('STM32 Lack of answer')
            else:
                print(Output)
                if(Output=='Blocked' and Avoidance==True):
                    X_,Y_,R_, Found, X_Backward, Y_Backward=self.Obstacle_Avoidance_Calculation()
                    if Found:
                        self.Go_To(X_Backward,Y_Backward,1,level=level+1,Max_Level=level+1,Avoidance=False)
                        #Move backward only and do not go further
                        self.Go_To(X_,Y_,R_,level+1,Max_Level)
                    else:
                        break;
                elif(Output=='Timeout'):
                    break;
                elif(Output=='Arrived'):
                    break;
                else:
                    break;
        print('End of', level)
    def Set_Max_Try_Single_Goal(self,x):
        self.Max_Try_Single_Goal=x
    def Reset_Nb_Try_Goal(self):
        self.Try_Single_Goal=0
    def Increment_Nb_Try_Goal(self):
        self.Try_Single_Goal=self.Try_Single_Goal+1
    def Keep_Trying_Goal(self):
        if (self.Try_Single_Goal<self.Max_Try_Single_Goal):
            return True
        else:
            return False
    
    def Send_Go_To_Coordinate(self,Set_Coordinate):
        X_Des=Set_Coordinate[0]
        Y_Des=Set_Coordinate[1]
        Max_Speed=Set_Coordinate[2]
        TimeOut=Set_Coordinate[3]
        Backward_Parameter=Set_Coordinate[4]
        """Create messages"""
        G0_String='G0 X' + str(round(X_Des))+' Y'+str(round(Y_Des))+' T'+str(round(TimeOut))+ ' R'+str(round(Backward_Parameter))+'\r\n'
        M201_String='M201 H0 S' + str(Max_Speed)+'\r\n'
        M3_String='M3 H3\r\n'
        All_Commands = (G0_String,M201_String,M3_String)
        print(All_Commands)
        self.Send_Messages(All_Commands)
        
    def Set_Robot_Position(self,x,y,a, send=False):
        self.X_Pos = x
        self.Y_Pos = y
        self.Angle_Deg = a
        if send :
            command = 'G92 X' + str(round(self.X_Pos)) +' Y'+str(round(self.Y_Pos))+ ' A'+str(round(self.Angle_Deg))+ '\r\n'
            self.Send_Messages(command)
    
    def Get_Robot_Position(self):
        return self.X_Pos,self.Y_Pos,self.Angle_Deg

    def Get_Beacon_Position(self):
        return self.X_Pos,self.Y_Pos,self.Angle_Deg
    
    def Set_Sensor_State(self,state):
        obstacle_position = []
        sensors_state = {'a':1,'b':2,'c':4,'d':8,'e':16,'f':32,'g':64,'h':128,'i':256,'j':512,'k':1024,'l':2048,'m':4096}
        mask = 1 
        for sensor in  sensors_state:
            if state&(mask*sensors_state[sensor])>0:
                sensors_state[sensor]=1
            else:
                sensors_state[sensor]=0
        self.ir_sensors_state=sensors_state
        print(self.ir_sensors_state)
    def Get_Sensor_State(self):
        return self.ir_sensors_state
                    
    def Has_reached_final_position(self, Final_Pos):
        #Coordinate must be in mm
        X,Y,_=self.get_position()
        Distance_X=Final_Pos[0]-X
        Distance_Y=Final_Pos[1]-Y
        if(math.sqrt(Distance_X*Distance_X+Distance_Y*Distance_Y)<200):
            return True
        return False
    def Get_Free_Space_Around(self,Sensor_State):
            
        #Get information from sensor and map
        Space_Free = {'Front':1,'Left':1,'Right':1,'Back':1 ,'FrontLeft':1,'FrontRight':1, 'BackLeft':1, 'BackRight':1, }
        if(Sensor_State['k'] or Sensor_State['l'] or Sensor_State['m'] or( Sensor_State['g'] and not(Sensor_State['j']))or( Sensor_State['e'] and not(Sensor_State['i']))):
            Space_Free['Front']=0;
        if(Sensor_State['a'] or Sensor_State['b'] or Sensor_State['c'] or Sensor_State['d']):
            Space_Free['Back']=0;
        if(Sensor_State['h'] or Sensor_State['g'] or Sensor_State['j']):
            Space_Free['Left']=0;
        if(Sensor_State['e'] or Sensor_State['f'] or Sensor_State['i']):
            Space_Free['Right']=0;
        if((Sensor_State['l'] and not(Sensor_State['k']))or ( Sensor_State['e'] and not(Sensor_State['i']))):
            Space_Free['FrontRight']=0;
        if((Sensor_State['k'] and not(Sensor_State['l']))or ( Sensor_State['g'] and not(Sensor_State['j']))):
            Space_Free['FrontLeft']=0;
        return Space_Free
    
    def Can_I_Go(self, Objectif):
        X_Robot,Y_Robot,A_Robot = self.Get_Robot_Position()
        Objectif_X, Objectif_Y = Objectif
        if(Objectif_X<0 or Objectif_X>8000 or Objectif_Y<0 or Objectif_Y>8000 ):
            return False
        elif(self.map.IsRockZone(Objectif) or self.map.IsPlatformZone(Objectif)):#Rock zone
            return False
        else:
            return True
    def Which_Zone_Am_I(self):
        X_Robot,Y_Robot,A_Robot = self.Get_Robot_Position()
        Zone = {'Normal':0,'Rock':0,'Platform':0,'Grass':0 }
        if(self.map.IsRockZone((X_Robot,Y_Robot))):
            Zone['Rock']=1
        elif(self.map.IsPlatformZone((X_Robot,Y_Robot))):
            Zone['Platform']=1
        elif(self.map.IsGrassZone((X_Robot,Y_Robot))):
            Zone['Grass']=1
        elif(self.map.IsNormalZone((X_Robot,Y_Robot))):
            Zone['Normal']=1  
        return Zone   
    def Obstacle_Avoidance_Calculation(self):
        return self.map.Calculate_Side_Checkpoint(self.Get_Robot_Position(),self.Get_Free_Space_Around(self.Get_Sensor_State()))
    def Start_Collector():
        self.ser_Collector.write(b'G1 R1500 L2100\r\n')
    def Reverse_Collector():
        self.ser_Collector.write(b'G1 R-1500 L-2100\r\n')
    def Stop_Collector():
        self.ser_Collector.write(b'G1 R0 L0\r\n')


class Map:
    def IsRockZone(self,Objectif):
        Objectif_X, Objectif_Y = Objectif
        if(Objectif_X<3000 and Objectif_Y>5000):
            return True
        return False
    def IsPlatformZone(self,Objectif):
        Objectif_X, Objectif_Y = Objectif
        if(Objectif_X>5000 and Objectif_Y>6000):
            return True
        return False
    def IsGrassZone(self,Objectif):
        Objectif_X, Objectif_Y = Objectif
        if(Objectif_X>4000 and Objectif_Y<2000):
            return True
        return False
    def IsNormalZone(self,Objectif):
        Objectif_X, Objectif_Y = Objectif
        if(Objectif_X>0 and Objectif_X<8000 and Objectif_Y>0 and Objectif_Y<8000):
            return not(self.IsRockZone(Objectif) or self.IsPlatformZone(Objectif) or self.IsGrassZone(Objectif))
        else:
            return False
    def IsAccessible(self,Objectif):
        return (self.IsNormalZone(Objectif) or self.IsGrassZone(Objectif))
    def Calculate_Side_Checkpoint(self,Robot_Pos, Space_Around):
        X, Y, A = Robot_Pos
        X_Robot, Y_Robot, A_Robot= Robot_Pos
        X_Backward=X+math.cos(math.radians(A-180))*300
        Y_Backward=Y+math.sin(math.radians(A-180))*300
        print(Space_Around)
        if(not(Space_Around['Front']) and Space_Around['FrontRight']):
            X_New=X+math.cos(math.radians(A-90))*300
            Y_New=Y+math.sin(math.radians(A-90))*300
            if(self.IsAccessible((X_New,Y_New))):
                print('Front go to Right',X_New, Y_New)
                return X_New,Y_New,0 , True, X_Backward,Y_Backward
        if(not(Space_Around['Front']) and Space_Around['FrontLeft']): 
            X_New=X+math.cos(math.radians(A+90))*300
            Y_New=Y+math.sin(math.radians(A+90))*300
            if(self.IsAccessible((X_New,Y_New))):
                print('Front go to left',X_New, Y_New)
                return X_New,Y_New,0 , True, X_Backward,Y_Backward
        if(Space_Around['Front'] and( not(Space_Around['Right'])or not(Space_Around['Left']))):
            X_New=X+math.cos(math.radians(A))*300
            Y_New=Y+math.sin(math.radians(A))*300
            if(self.IsAccessible((X_New,Y_New))):
                print('Avoidance Side go to the Front',X_New, Y_New)
                return X_New,Y_New,0 , True, X_Backward,Y_Backward
        if(not(Space_Around['Front']) and not(Space_Around['Left']) and not(Space_Around['Right']) ):
            #Back LEFT
            
            X_Backward=X+math.cos(math.radians(A-180))*600
            Y_Backward=Y+math.sin(math.radians(A-180))*600
            X_New=X_Backward+math.cos(math.radians(A+90))*300
            Y_New=Y_Backward+math.sin(math.radians(A+90))*300
            #Back Right
            
            if(self.IsAccessible((X_New,Y_New))):
                print('Avoidance Back right',X_New, Y_New)
                return X_New,Y_New,0 , True, X_Backward,Y_Backward
            X_New=X+math.cos(math.radians(A-90))*300
            Y_New=Y+math.sin(math.radians(A-90))*300
            if(self.IsAccessible((X_New,Y_New))):
                print('Avoidance Back Left',X_New, Y_New)
                return X_New,Y_New,0 , True, X_Backward,Y_Backward
        X_New=X
        Y_New=Y
        print('Avoidance not found',X_New, Y_New)
        return X_New,Y_New,0 , False , X_Backward,Y_Backward

        


