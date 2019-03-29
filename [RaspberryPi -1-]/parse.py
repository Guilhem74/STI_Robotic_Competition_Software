import serial

SERIAL_PORT = '/dev/ttyS0'

SERIAL_RATE = 115200
xs = []
ys = []
   

def parse(raw_message):
    receivedCom = {}
    com = raw_message.split()
    for i in range(len(com)):
        if(com[i][0] == "G"):
            receivedCom["G"] = com[i][1:]
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
            print("Unknown command")
    
    return receivedCom

def build_com(nextOrder_com):
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
    return message
                 

    
    
def main():
    received_com = {} #Dictonnary that keep the last command received from STM32
    nextOrder_com = {} #Dictonnary that store the next command to emit toSTM32
    ser = serial.Serial(SERIAL_PORT, SERIAL_RATE)

    while True:
        
        ser.flush()
        if(ser.in_waiting > 0 ):
            reading = ser.readline().decode('utf-8')
            
            received_com = parse(reading)
            
            print(reading)
            message = build_com(received_com)
            ser.write(message.encode())
            


main()