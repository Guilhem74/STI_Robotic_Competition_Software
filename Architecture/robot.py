
import numpy as np
import math
import  time, sys , serial
import threading
from collections import deque
from Serial_ import *
from parse import *
import TCP_IP as Tcp_Ip
import cv2 as cv
import picamera
from fct import * 

SERIAL_PORT_ROBOT = '/dev/ttyACM0'
SERIAL_PORT_COllECTOR = '/dev/ttyACM1'
SERIAL_RATE = 115200
BUFFER_SIZE = 4096
class Robot_Class:
    def __init__(self,x=0,y=0,a=0):
        self.X_Pos = x
        self.Y_Pos = y
        self.X_Previous_Pos = x
        self.Y_Previous_Pos = y
        self.Angle_Deg = a
        self.Robot_Speed=750
        self.map= Map()
        self.Bottle_List=[]
        self.TCP_Conn=None
        self.TCP_Addr=None
        self.TCP_S=None
        self.Try_Single_Goal=0
        self.Max_Try_Single_Goal=2
        self.Sensor_Enabled=15
        self.Robot_Speed=150
        self.Bottle_Collected=0
        self.camera=picamera.PiCamera()
        self.camera.resolution = (1080, 720)
        self.camera.framerate = 60
        self.camera.shutter_speed = 5000
        self.stream = io.BytesIO()
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
    def Connect_TCP(self):
        if self.TCP_Conn == None:
            print('Trying to connect')
            self.TCP_Conn, self.TCP_Addr, self.TCP_S = Tcp_Ip.enable_tcp_com(BUFFER_SIZE)
            print('Connected')
    def Close_TCP(self):
        if self.TCP_S != None:
            self.TCP_S.close()
            self.TCP_Conn=None
            self.TCP_Addr=None
            self.TCP_S=None
    def Empty_TCP(self):
        self.Get_Data_TCP()
    def Get_Data_TCP(self):
        if self.TCP_Conn != None:
            return Tcp_Ip.check_data_received(self.TCP_Conn, 4096)
        print('Unconnected')
        return None
    def Parse_Data_TCP(self, Data):
        Array= []
        for unit in Data:
            str2=parse("{},{}",unit)
            if(str2 is not None):
                array_argument=np.zeros(2)
                for i in range(2):
                    array_argument[i]=float(str2[i])
                Array.append((array_argument[0],array_argument[1]))
        return Array
    def Send_Messages(self,All_Commands):
        for string_ in All_Commands:
            self.Send_Message(string_)#Send every messages
    def Send_Message(self,Single_Command):
            self.buf_stm.append(Single_Command)
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
                print('Message',Message)
                if(Valid is True):
                    return Valid, Output
    def Wait_For_M2_Answer(self, Timeout):
        Start_Time=time.time()
        while(True):
            Message=self.Collect_Message()
            if((time.time()-Start_Time)>Timeout and Message==None):
                return None, None
            elif(Message is not None):
                Valid, Output= self.Is_it_M2_Answer(Message)
                print('Message',Message)
                if(Valid is True):
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
    def Is_it_M2_Answer(self, Message):
        if(Message is None):
            return False, None
        str2=parse(" M0 H2 T{}\r\n",Message)
        print('PreParse',Message)
        if(str2 is not None):
            array_argument=np.zeros(1)
            print('PostParse',str2)
            for i in range(1):
                array_argument[i]=float(str2[i])
            if array_argument[0]==1:
                return True, 'Timeout'
            elif array_argument[0]==2:
                return True, 'Blocked'
            return True, 'Unknow'
            
        return False, None
    def Go_To(self,X,Y,R,level=0,Max_Level=3, Avoidance=True, Path_Checking=True):
        """WARNING: Recursive function"""
        if (level==0):
            self.Reset_Nb_Try_Goal()
        while(level<=Max_Level and self.Keep_Trying_Goal() and not(self.Has_reached_final_position((X,Y)))):
            X_Robot , Y_Robot, A_Robot = self.Get_Robot_Position()
            self.Set_Previous_Pos((X_Robot,Y_Robot))
            X_Des=X
            Y_Des=Y
            if(not(self.Check_Valid_Path((X_Robot,Y_Robot),(X_Des,Y_Des))) and Path_Checking):
                if(self.map.IsUpper_Normal_Zone((X_Robot,Y_Robot))):
                    Y_Des=Y_Robot#Not gonna change the Y coordinate
                else:
                    X_Des=X_Robot#Not gonna change the X coordinate
            Timeout = (math.sqrt((X_Des-X_Robot)*(X_Des-X_Robot)+(Y_Des-Y_Robot)*(Y_Des-Y_Robot))/self.Robot_Speed)*1000
            Set_Coordinate=[X_Des,Y_Des,self.Robot_Speed,max(Timeout*1.2,1000),R]
            self.Send_Go_To_Coordinate(Set_Coordinate)
            self.Increment_Nb_Try_Goal()
            M0_State, Output=self.Wait_For_M3_Answer((Timeout*1.2+2000)/1000)
            if(M0_State== False):
                print('STM32 Lack of answer')
            else:
                print(Output)
                self.Clean_All_On_Path((self.Get_Previous_Pos()), (self.Get_Robot_Position()));
                if(Output=='Blocked' and Avoidance==True ):
                    X_,Y_,R_, Extra, X_Backward, Y_Backward, R_Backward=self.Obstacle_Avoidance_Calculation((X_Des,Y_Des,R))
                    if Extra:
                        self.Go_To(X_Backward,Y_Backward,R_Backward,level=level+1,Max_Level=level+1,Avoidance=False)
                        self.Go_To(X_,Y_,R_,level+1,Max_Level)
                        #Move backward only and do not go further
                    else:
                        self.Go_To(X_,Y_,R_,level+1,Max_Level)
                elif(Output=='Timeout' and self.Has_reached_final_position((X_Des,Y_Des))):
                    break;
                elif(Output=='Timeout'):
                    self.Increment_Nb_Try_Goal()
                elif(Output=='Arrived'):
                    break;
                else:
                    #Avoidance not allowed
                    break;
    def Go_To_Speed(self,R,L,T,X_Cal=-9999, Y_Cal=-9999, A_Cal=-9999,Detection=True, Calibration=False, Level=0):
        """WARNING: Recursive function"""
        if(Level>=3):
            return 
        G1_String='G1 R' + str(round(R))+' L'+str(round(L))+' T'+str(round(T))+'\r\n'
        if(Detection==True):
            M3_String='M3 H2 S'+ str(15) +'\r\n'
        else:
            M3_String='M3 H2 S'+ str(0) +'\r\n'
        All_Commands = (G1_String,M3_String)
        print(All_Commands)
        self.Send_Messages(All_Commands)
        M0_State, Output=self.Wait_For_M2_Answer(max((T*1.2/1000),2.5))
        if(M0_State== False):
            print('STM32 Lack of answer')
        else:
            print('answer',Output)
            if(Output=='Blocked' and Calibration==False ):
                return
            elif(Output=='Blocked'  and Calibration==True):
                print('Setting Pos')
                self.Set_Robot_Position(X_Cal,Y_Cal,A_Cal,True)
            elif(Output=='Timeout' and Calibration==True):
                print('Setting pos')
                self.Set_Robot_Position(X_Cal,Y_Cal,A_Cal,True)
            elif(Output=='Timeout'):
                return
        #print('End of', level)
    def Set_Previous_Pos(self,Pos):
        self.X_Previous_Pos=Pos[0]
        self.Y_Previous_Pos=Pos[1]
    def Get_Previous_Pos(self):
        return self.X_Previous_Pos, self.Y_Previous_Pos
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
    def Set_Robot_Speed(self,x, Transmission=True):
        self.Robot_Speed=x
        if Transmission:
            M201_String='M201 H0 S' + str(x)+'\r\n'
            #print(M201_String)
            self.Send_Message(M201_String)
    def Get_Robot_Speed(self):
        return self.Robot_Speed
    def Send_Go_To_Coordinate(self,Set_Coordinate):
        X_Des=Set_Coordinate[0]
        Y_Des=Set_Coordinate[1]
        Max_Speed=Set_Coordinate[2]
        TimeOut=Set_Coordinate[3]
        Backward_Parameter=Set_Coordinate[4]
        """Create messages"""
        G0_String='G0 X' + str(round(X_Des))+' Y'+str(round(Y_Des))+' T'+str(round(TimeOut))+ ' R'+str(round(Backward_Parameter))+'\r\n'
        M3_String='M3 H3 S'+ str(self.Sensor_Enabled) +'\r\n'
        All_Commands = (G0_String,M3_String)
        #print(All_Commands)
        self.Send_Messages(All_Commands)
        
    def Set_Robot_Position(self,x=-9999,y=-9999,a=-9999, Transmission=False):
        command = 'G92 '
        if x!=-9999:
            self.X_Pos = x
            command = command + 'X' + str(round(self.X_Pos)) + ' '
        if y!=-9999:
            self.Y_Pos = y
            command = command + 'Y' + str(round(self.Y_Pos))+ ' '
        if a!=-9999:
            self.Angle_Deg = a
            command = command + 'A' + str(round(self.Angle_Deg))+ ' '
        if Transmission :
            command=command+'\r\n'
            print(command)
            self.Send_Message(command)
    
    def Get_Robot_Position(self):
        return self.X_Pos,self.Y_Pos,self.Angle_Deg

    def Get_Beacon_Position(self):
        start=time.time()
        self.camera.capture('Image.jpeg', format='jpeg')

        img = cv.imread('Image.jpeg')
        print("total time capture image: ", time.time() - start)
        Robot_Pos=self.beacon_main(img)
        print("total time post math: ", time.time() - start)
        return Robot_Pos
    def Disable_All_Sensors(self):
        self.Sensor_Enabled=0;
    def Enable_All_Sensors(self):
        self.Sensor_Enabled=15;
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
    def Get_Bottle_List(self):
        return self.Bottle_List
    def Set_Bottle_List(self,List):
        self.Bottle_List=List
    def Set_New_Bottles(self, New):
        self.Bottle_List.extend(New)
        return self.Bottle_List
    def Clear_Bottle_List(self):
        self.Bottle_List=[]
    def Clean_Bottle_Of_Position(self, Position):
        self.Bottle_List=list(filter(lambda x: self.Distance_to_Bottle(x,Position) > 400, self.Bottle_List))
        return self.Bottle_List
    def Clean_All_On_Path(self, Start_Position,End_Position,Step_Size=200):
        X_Start= Start_Position[0]
        Y_Start= Start_Position[1]
        X_End=End_Position[0]
        Y_End= End_Position[1]
        Error_X=X_End-X_Start;
        Error_Y=Y_End-Y_Start;
        Angle_Travel=math.atan2(Error_Y,Error_X)
        Distance= round(math.sqrt((Error_X) ** 2 + (Error_Y) ** 2))
        for Point in list(range(0,Distance,Step_Size)):
            X_Temp=X_Start+math.cos(Angle_Travel)*Point
            Y_Temp=Y_Start+math.sin(Angle_Travel)*Point
            self.Clean_Bottle_Of_Position((X_Temp,Y_Temp))
        self.Clean_Bottle_Of_Position((X_End,Y_End))
    def Check_Valid_Path(self, Start_Position,End_Position,Step_Size=200):
        X_Start= Start_Position[0]
        Y_Start= Start_Position[1]
        X_End=End_Position[0]
        Y_End= End_Position[1]
        Error_X=X_End-X_Start;
        Error_Y=Y_End-Y_Start;
        Angle_Travel=math.atan2(Error_Y,Error_X)
        Distance= round(math.sqrt((Error_X) ** 2 + (Error_Y) ** 2))
        for Point in list(range(0,Distance,Step_Size)):
            X_Temp=X_Start+math.cos(Angle_Travel)*Point
            Y_Temp=Y_Start+math.sin(Angle_Travel)*Point
            if(self.map.IsAccessible((X_Temp,Y_Temp))):
                pass
            else:
                return False
        return True
    def Add_Bottle_Collected(self):
        self.Bottle_Collected=self.Bottle_Collected+1        
    def Get_Bottle_Collected(self):
        return self.Bottle_Collected
    def Reset_Bottle_Collected(self):
        self.Bottle_Collected=0
    def Update_Bottle_List(self,Data):
        if Data!=None:
            Data=self.Parse_Data_TCP(Data)
            List=self.Set_New_Bottles(Data)
        else:
            List=self.Get_Bottle_List()
        self.Clean_Bottle_Of_Position(self.Get_Robot_Position())
        self.Set_Bottle_List(self.Sort_Bottle_List(List))
        return List
    def Sort_Bottle_List(self,List):
        if List != []:
            List.sort(key=lambda x: self.Ponderation_Bottle(x, self.Get_Robot_Position()))
            return List
        else:
            return []
    def Extend_Coordinate(self,Bottle_Position,Distance=800):
        X,Y,A=self.Get_Robot_Position()
        X_Bottle, Y_Bottle= Bottle_Position
        Error_X=X_Bottle-X;
        Error_Y=Y_Bottle-Y;
        Angle_Des=math.atan2(Error_Y,Error_X)
        X_New=X_Bottle+math.cos((Angle_Des))*Distance
        Y_New=Y_Bottle+math.sin((Angle_Des))*Distance
        if(self.map.IsAccessible((X_New,Y_New))):
            return X_New,Y_New, True
        X_New=X_Bottle+math.cos((Angle_Des))*Distance/2
        Y_New=Y_Bottle+math.sin((Angle_Des))*Distance/2
        if(self.map.IsAccessible((X_New,Y_New))):
            return X_New,Y_New, True
        return X_Bottle, Y_Bottle, False
        
    def Distance_to_Bottle(self,Bottle_Position, Position):
        X=Position[0]
        Y =Position[1]
        Distance=math.sqrt((Bottle_Position[0] - X) ** 2 + (Bottle_Position[1] - Y) ** 2)
        return Distance
    def Get_Nearest_Bottle(self):
        if self.Bottle_List != []:
            Bottle=self.Bottle_List.pop(0)
            return Bottle[0],Bottle[1], self.Goal_Score(Bottle, self.Get_Robot_Position())
        else:
            return None, None, None
    def Ponderation_Bottle(self,Bottle_Position, Robot_Position):
        X,Y,A=Robot_Position
        Distance=math.sqrt((Bottle_Position[0] - X) ** 2 + (Bottle_Position[1] - Y) ** 2)
        Error_Angle=(math.degrees(math.atan2((Bottle_Position[1] - Y),(Bottle_Position[0] - X)))-A)
        C=6#Coefficient to favorise distance over angle
        Angle=min((-Error_Angle)%360,Error_Angle%360)
        return Distance +Angle*C
    def Goal_Score(self,Bottle_Position, Robot_Position):
        return self.Ponderation_Bottle(Bottle_Position, Robot_Position)
    def Has_reached_final_position(self, Final_Pos):
        #Coordinate must be in mm
        X,Y,_=self.Get_Robot_Position()
        Distance_X=Final_Pos[0]-X
        Distance_Y=Final_Pos[1]-Y
        if(math.sqrt(Distance_X*Distance_X+Distance_Y*Distance_Y)<200):
            return True
        return False
    def Get_Free_Space_Around(self,Sensor_State):
            
        #Get information from sensor and map
        Space_Free = {'Front':1,'Left':1,'Right':1,'Back':1 ,'FrontLeft':1,'FrontRight':1, 'BackLeft':1, 'BackRight':1, }
        if(Sensor_State['m'] or (Sensor_State['k'] and Sensor_State['l'])):
            Space_Free['Front']=0;
        if(Sensor_State['l'] or(Sensor_State['e'] and not(Sensor_State['i']))):
            Space_Free['FrontRight']=0;
        if(Sensor_State['k'] or(Sensor_State['g'] and not(Sensor_State['j']))):
            Space_Free['FrontLeft']=0;
        if(Sensor_State['h'] or Sensor_State['j']):
            Space_Free['Left']=0;
        if(Sensor_State['f'] or Sensor_State['i']):
            Space_Free['Right']=0;
        if(Sensor_State['a'] or Sensor_State['b'] or Sensor_State['c'] or Sensor_State['d']):
            Space_Free['Back']=0;
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
    def Obstacle_Avoidance_Calculation(self, Goal):
        return self.map.Calculate_Side_Checkpoint(self.Get_Robot_Position(),Goal,self.Get_Free_Space_Around(self.Get_Sensor_State()))
    def Start_Collector(self):
        self.ser_Collector.write(b'G1 R1500 L2100\r\n')
    def Reverse_Collector(self):
        self.ser_Collector.write(b'G1 R-1500 L-2100\r\n')
    def Stop_Collector(self):
        self.ser_Collector.write(b'G1 R0 L0\r\n')
    def beacon_main(self,rawImage):
        boundaries = [
         ([0, 0, 180], [255, 153, 255], 'r', (0,0,255), (0,8000)),
         ([230, 141, 0], [255, 225, 255], 'b', (255,0,0), (8000,0)),
         ([0, 200, 97], [255, 255, 255], 'y', (0,255,255), (0,0)),
         ([77, 235, 0], [220, 244, 255], 'g', (0,255,0), (8000,8000))
        ]
        start = time.time()
        center_circles = (532, 357)
        center_beacon = (541,341)

        height,width,depth = rawImage.shape
        imgWithCircle  = np.zeros((height,width), np.uint8)
        cv2.circle(imgWithCircle,center_circles,207,(255,255,255),thickness=-1)
        cv2.circle(imgWithCircle,center_circles,165,(0,0,0),thickness=-1)
        cv2.circle(imgWithCircle,center_circles,75,(255,255,255),thickness=-1)
        cv2.circle(imgWithCircle,center_circles,45,(0,0,0),thickness=-1)
        imask = imgWithCircle>0
        img = np.zeros_like(rawImage, np.uint8)

        img[imask] = rawImage[imask]

        print(" batch circle: ", time.time() - start)
        start = time.time()

        ret,thresh_img = cv2.threshold(img,160,0,cv2.THRESH_TOZERO)
        angles,lights_coordinates = find_angles(thresh_img,center_beacon,boundaries)
        print(angles,lights_coordinates)

        print("Find angle: ", time.time() - start)
        start = time.time()

        #thresh_img[np.where((thresh_img==[0,0,0]).all(axis=2))] = [160,160,160]

        if len(angles) < 3:
            print("less than 3 lights found")
            x, y, a = self.Get_Robot_Position()
            return float(x), float(y), float(a),thresh_img
        if len(angles) == 4:
            x, y, a = self.Get_Robot_Position()
            x,y,z = float(x), float(y), float(a)
            if math.sqrt((x-0)**2 + (y-0)**2) < math.sqrt((x-8000)**2 + (y-8000)**2):
                a1,a2,a3 = angles[0],angles[1],angles[2]
                angles = a1,a2,a3
                l1,l2,l3 = lights_coordinates[0],lights_coordinates[1],lights_coordinates[2]
                lights_coordinates = l1,l2,l3
            else:
                a1,a2,a3 = angles[0],angles[1],angles[3]
                angles = a1,a2,a3
                l1,l2,l3 = lights_coordinates[0],lights_coordinates[1],lights_coordinates[3]
                lights_coordinates = l1,l2,l3
        xr,yr,ar = find_robot_pos(angles,lights_coordinates)  
        print("Find robot pos: ", time.time() - start)
        start = time.time()

        if xr == -1 or yr == -1 or ar == -1:
            x, y, a = self.Get_Robot_Position()
            return float(x), float(y), float(a), thresh_img
        print("Beacon worked!")
        return xr,yr,ar,thresh_img



class Map:
    def IsRockZone(self,Objectif):
        Objectif_X, Objectif_Y = Objectif
        if(Objectif_X<3500 and Objectif_Y>4500):
            return True
        return False
    def IsPlatformZone(self,Objectif):
        Objectif_X, Objectif_Y = Objectif
        if(Objectif_X>4750 and Objectif_Y>5750):
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
    def IsUpper_Normal_Zone(self,Objectif): #Between rock and platform
        Objectif_X, Objectif_Y = Objectif
        if (self.IsNormalZone(Objectif)):
            if(Objectif_X>3000 and Objectif_Y>5000 and Objectif_X<5000):
                return True
        return  False
    def Calculate_Side_Checkpoint(self,Robot_Pos,Goal, Space_Around):
        X, Y, A = Robot_Pos
        X_Des=Goal[0]
        Y_Des=Goal[1]
        R_Param=Goal[2]
        Backward=0;
        Error_X=X_Des-X;
        Error_Y=Y_Des-Y;
        Angle_Des=math.atan2(Error_Y,Error_X)
        Error_Angle = math.degrees(Angle_Des) - A
        while(Error_Angle>=180): 
            Error_Angle=Error_Angle-360 
        while(Error_Angle<-180): 
            Error_Angle=Error_Angle+360  
        if(R_Param==1 and(Error_Angle>110 or Error_Angle<-110)):
            Backward=1;
        else:
            Backward=0
        X_Backward=X+math.cos(math.radians(A-180))*300
        Y_Backward=Y+math.sin(math.radians(A-180))*300
        #print(Space_Around)
        if((not(Space_Around['Front']) or not(Space_Around['FrontRight']) or not(Space_Around['FrontLeft'])) 
           and abs(Error_Angle)<45):
            #Obstacle is in front of the displacement and annoy us
            if(Space_Around['FrontRight'] and Space_Around['Right'] ): #Right is free
                X_New=X+math.cos(math.radians(A-90))*300
                Y_New=Y+math.sin(math.radians(A-90))*300
                if(self.IsAccessible((X_New,Y_New))):
                    #print('Avoidance 9__')
                    return X_New,Y_New,0 , True, X_Backward,Y_Backward , 1
            if(Space_Around['FrontLeft'] and Space_Around['Left'] ): #Left is free
                X_New=X+math.cos(math.radians(A+90))*300
                Y_New=Y+math.sin(math.radians(A+90))*300
                if(self.IsAccessible((X_New,Y_New))):
                    #print('Avoidance A__')
                    return X_New,Y_New,0 , True, X_Backward,Y_Backward , 1
            #Full front busy
            if(Space_Around['Right'] ): #Right is free
                X_Backward=X+math.cos(math.radians(A-180))*450
                Y_Backward=Y+math.sin(math.radians(A-180))*450
                X_New=X+math.cos(math.radians(A-90))*600
                Y_New=X+math.sin(math.radians(A-90))*600
                if(self.IsAccessible((X_New,Y_New))):
                    #print('Avoidance B__')
                    return X_New,Y_New,0 , True, X_Backward,Y_Backward , 1
            if(Space_Around['Left'] ): #Right is free
                X_Backward=X+math.cos(math.radians(A-180))*450
                Y_Backward=Y+math.sin(math.radians(A-180))*450
                X_New=X+math.cos(math.radians(A+90))*600
                Y_New=Y+math.sin(math.radians(A+90))*600
                if(self.IsAccessible((X_New,Y_New))):
                    #print('Avoidance C__')
                    return X_New,Y_New,0 , True, X_Backward,Y_Backward , 1
            #No position accessible found
            X_Backward=X+math.cos(math.radians(A-180))*600
            Y_Backward=Y+math.sin(math.radians(A-180))*600
            X_New=X_Backward+math.cos(math.radians(A-90))*300
            Y_New=Y_Backward+math.sin(math.radians(A-90))*300
            if(self.IsAccessible((X_New,Y_New))):
                #print('Avoidance D__')
                return X_New,Y_New,0 , True, X_Backward,Y_Backward , 1
            X_New=X+math.cos(math.radians(A+90))*300
            Y_New=Y+math.sin(math.radians(A+90))*300
            if(self.IsAccessible((X_New,Y_New))):
                #print('Avoidance E__')
                return X_New,Y_New,0 , True, X_Backward,Y_Backward , 1
        if((not(Space_Around['Right']) or not(Space_Around['Left']) or not(Space_Around['FrontRight']) or not(Space_Around['FrontLeft'])) and abs(Error_Angle)>45 ):
            if(Space_Around['Front'] and Space_Around['FrontRight'] and Space_Around['FrontLeft']):
                X_New=X+math.cos(math.radians(A))*800
                Y_New=Y+math.sin(math.radians(A))*800
                if(self.IsAccessible((X_New,Y_New))):
                    #print('Avoidance F__')
                    return X_New,Y_New,0 , False, X_New,Y_New , 0
            if(Space_Around['Back']):
                X_New=X+math.cos(math.radians(A-180))*1100
                Y_New=Y+math.sin(math.radians(A-180))*1100
                if(self.IsAccessible((X_New,Y_New))):
                    #print('Avoidance G__')
                    return X_New,Y_New,1 , False, X_New,Y_New , 0
       
        X_New=X
        Y_New=Y
        print('Avoidance H')
        return X_New,Y_New,0 , False , X_Backward,Y_Backward,0 

