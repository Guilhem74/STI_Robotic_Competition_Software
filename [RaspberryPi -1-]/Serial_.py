
import serial
import time
import threading
from collections import deque

   
exitFlag = 0
class serialTread():
    
    def __init__(self, ser):
        self.maxLen = 100
        self.ser = ser
        
    def run(self,stmBuf_IN,stmBuf_OUT):
        while(True):
            if self.ser.in_waiting > 0  : 
                re = self.ser.readline().decode('utf-8')
                print(re)
                stmBuf_IN.append(re)
            try:
                m = stmBuf_OUT.popleft().encode('UTF-8')
                self.ser.write(m)
                print(m)
            except:
                pass

