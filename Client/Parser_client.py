
import threading
from collections import deque
import copy

class Parser_client ():


    def run(self,buf_IN,buf_OUT,action_In,action_Out):
        while(True):
            if(buf_IN):
                try:
                    action_In.append(self.parse(buf_IN.popleft())) #Copy the message in dict for future logic reasoning
                    
                except  IndexError as e:
                    pass 
                    
            if(buf_OUT):
                try:
                    buf_OUT.append(self.build_com(action_Out.popleft()))

                except IndexError as e:
                    pass
                    
            


      


    def parse(self,raw_message):
        receivedCom = {}
        raw_message = raw_message.replace("=","")
        raw_message = raw_message.replace("\r","")
        raw_message = raw_message.replace("\n","")
        raw_message = raw_message.replace("OK","")
        com = raw_message.split()
        try:
            for i in range(len(com)):
                if(com[i][0] == "G"):
                    receivedCom["G"] = float(com[i][1:])
                elif(com[i][0] == "O"):
                    receivedCom["O"] = float(com[i][1:])    
                elif(com[i][0] == "M"):
                    receivedCom["M"] = float(com[i][1:])
                elif(com[i][0] == "X"):
                    receivedCom["X"] = float(com[i][1:])
                elif(com[i][0] == "Y"):
                    receivedCom["Y"] = float(com[i][1:])
                elif(com[i][0] == "A"):
                    receivedCom["A"] = float(com[i][1:])
                elif(com[i][0] == "Z"):
                    receivedCom["Z"] = float(com[i][1:])
                elif(com[i][0] == "F"):
                    receivedCom["F"] = float(com[i][1:])
                elif(com[i][0] == "T"):
                    receivedCom["T"] = float(com[i][1:])

                else:
                    pass
        except:
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
               


