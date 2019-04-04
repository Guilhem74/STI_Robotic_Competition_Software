
import threading
from collections import deque
from Main import *
import copy

class Parser ():


    def run(self,broadBuf_IN,stmBuf_IN,broadBuf_OUT,stmBuf_OUT,action_In,action_Out):
        while(True):
            if(len(broadBuf_IN) > len(stmBuf_IN)):
                try:
                    action_In.append(copy.deepcopy(self.parse(broadBuf_IN[0]))) #Copy the message in dict for future logic reasoning
                   
                      #Do action  or register qui action in queue
                    m = broadBuf_IN.popleft()
                    stmBuf_OUT.append(m) 
                    broadBuf_OUT.append(m)
                    
                except  IndexError as e:
                    pass 
                    
            else:
                try:
                    action_In.append(copy.deepcopy(self.parse(stmBuf_IN[0]))) #Copy the message in dict for future logic reasoning
                     #Do action  or register qui action in queue
                    broadBuf_OUT.append(stmBuf_IN.popleft())
                except IndexError as e:
                    pass
                    
            if(action_Out):
                message = build_com(self,action_Out.popleft())
                stmBuf_OUT.append(message)
                broadBuf_OUT.append(message)


      


    def parse(self,raw_message):
        receivedCom = {}
        com = raw_message.split()
        for i in range(len(com)):
            if(com[i][0] == "G"):
                receivedCom["G"] = com[i][1:]
            elif(com[i][0] == "O"):
                receivedCom["O"] = com[i][1:]    
            elif(com[i][0] == "M"):
                receivedCom["M"] = com[i][1:]
            elif(com[i][0] == "X"):
                receivedCom["X"] = com[i][1:]
            elif(com[i][0] == "Y"):
                receivedCom["Y"] = com[i][1:]  
            elif(com[i][0] == "Z"):
                receivedCom["Z"] = com[i][1:]
            elif(com[i][0] == "F"):
                receivedCom["F"] = com[i][1:]
            elif(com[i][0] == "T"):
                receivedCom["T"] = com[i][1:]

            else:
                pass

        return receivedCom

    def build_com(self,nextOrder_com):
        command_L = []
        if("G" in nextOrder_com):  
            command_L.append("G" + str(nextOrder_com['G']))     
        if("M" in nextOrder_com):  
            command_L.append("M" + str(nextOrder_com['M']))
        if("X" in nextOrder_com):  
            command_L.append("X" + str(nextOrder_com['X']))
        if("Y" in nextOrder_com):  
            command_L.append("Y" + str(nextOrder_com['Y']))
        if("Z" in nextOrder_com):  
            command_L.append("Z" + str(nextOrder_com['Z']))
        if("F" in nextOrder_com ):  
            command_L.append("F" + str(nextOrder_com['F']))
        if("T" in nextOrder_com ):  
            command_L.append("T" + str(nextOrder_com['T']))
        message = " ".join(command_L)
        #message = message + "\r\n"
        return message
               


