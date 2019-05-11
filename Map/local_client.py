
import numpy as np
from PIL import Image
from heapq import *
import socket, time, sys
import math
import threading
from collections import deque

class client:


        
    def run(self,buf_IN,buf_OUT,connected):
        connected = True
        
        buff = ""
        #connect to server
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect(('127.0.01',65432))
        client_socket.settimeout(0.01)
        
        
        while connected:
            try:
                client_socket.sendall(buf_OUT.popleft().encode('utf-8'))
            except:
                pass
            
            try:
                data = client_socket.recv(1024)
                buf_IN.append(data.decode())
            except socket.timeout:
                pass
                
            
                
            
        
        client_socket(close)
        connected = False
