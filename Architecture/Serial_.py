
import serial
import time
import threading
from collections import deque

   
exitFlag = 0
class serialTread():
    
    def __init__(self, ser):
        self.maxLen = 100
        self.ser = ser
        self.buf = bytes()
        
    def run(self,buf_broadcast,buf_stm):
        thread = threading.Thread(target=self.read_from_port, args=(buf_broadcast,))
        thread.start()   
        while(True):
            
            if buf_stm:
                m = buf_stm.popleft().encode('UTF-8')
                self.ser.write(m)
                time.sleep(0.005)

             


    def read_from_port(self,buf_broadcast):
        while True:
            
            reading = self.ser.readline().decode('utf-8')
            
            buf_broadcast.append(reading)
            
                    
              
    
       

        