


##Server

import socket, time, sys , serial
import threading
import pprint
from TCP_server import *
from Serial import *
from Parser import *
SERIAL_PORT = '/dev/ttyACM0'
SERIAL_RATE = 115200
import threading
from collections import deque

clientCount = 0


buf_broadcast = deque(["G92 X300"]) # Queue of message for broadcasting via TCP IP after being parsed
buf_stm = deque(["G92 X400"])



if __name__ == '__main__':
    ser = serial.Serial(SERIAL_PORT, SERIAL_RATE) # Connection to STM32
    #p = Parser()
    s = server() #create new server listening for connections
    serialT = serialTread(ser)
    

    
    threading.Thread(target=s.startServer,args=(buf_broadcast,buf_stm,clientCount)).start()#create server thread
    threading.Thread(target=serialT.run, args=(buf_broadcast,buf_stm)).start()#create serial thread

    while(1):
        print("buf_stm")
        print(buf_stm)
        print("buf_broadcast")
        print(buf_broadcast)
        
        
        time.sleep(10)
#Changer les action_In et action_Out en queue de dictionaire

