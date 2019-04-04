


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

broadBuf_IN = deque(["G92 X100"]) # Queue of message for broadcasting via TCP IP before being parsed
stmBuf_IN = deque(["G92 X200"]) # Queue of message for communication to STM32 
broadBuf_OUT = deque(["G92 X300"]) # Queue of message for broadcasting via TCP IP after being parsed
stmBuf_OUT = deque(["G92 X400"])
action_In = deque({})
action_Out = deque({})


if __name__ == '__main__':
    ser = serial.Serial(SERIAL_PORT, SERIAL_RATE) # Connection to STM32
    p = Parser()
    s = server() #create new server listening for connections
    serialT = serialTread(ser)
    

    threading.Thread(target=p.run, args=(broadBuf_IN,stmBuf_IN,broadBuf_OUT,stmBuf_OUT,action_In,action_Out)).start()#create parser thread
    threading.Thread(target=s.startServer,args=(broadBuf_IN,broadBuf_OUT,clientCount)).start()#create server thread
    threading.Thread(target=serialT.run, args=(stmBuf_IN,stmBuf_OUT)).start()#create serial thread

    while(1):
        print("broadBuf_IN")
        print(broadBuf_IN)
        print("broadBuf_OUT")
        print(broadBuf_OUT)
        print("stmBuf_IN")
        print(stmBuf_IN)
        print("stmBuf_OUT")
        print(stmBuf_OUT)
        #s._broadcast()
        #pprint.pprint(s.CLIENTS)
        #print(broadBuf_IN)
        #print(broadBuf_OUT)
        
        time.sleep(10)
#Changer les action_In et action_Out en queue de dictionaire

