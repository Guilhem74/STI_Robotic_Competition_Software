
import sys, serial, argparse
import numpy as np
from time import sleep
from collections import deque

import matplotlib.pyplot as plt 
import matplotlib.animation as animation
import matplotlib.patches as mpatches

SERIAL_PORT = 'COM9'
SERIAL_RATE = 115200

xs = []
ys = []
xs1 = []
ys1 = []
xs2 = []
ys2 = []
comIN = {} #Dictonnary that keep the last command received from STM32
comOUT = {} #Dictonnary that store the next command to emit toSTM32   


def send(message):
  global ser
  ser.flush()
  ser.write(message.encode())
  
def read():
  global ser, comIN
  try:
    line = self.ser.readline().decode('utf-8')
    comIN = parse(line)
  
  except:
    print("no input")

  print(comIN)



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
                  


    
# plot class

class AnalogPlot:
  # constr
  def __init__(self,ser, maxLen, nb_curve):
      self.ser = ser
      
      self.nb_curve = nb_curve
      self.ax = deque([0.0]*maxLen)
      self.ay = deque([0.0]*maxLen)
      
      self.ay1 = deque([0.0]*maxLen)
      
      self.ay2 = deque([0.0]*maxLen)
      self.maxLen = maxLen

  # add to buffer
  def addToBuf(self, buf, val):
      if len(buf) < self.maxLen:
          buf.append(val)
      else:
          buf.pop()
          buf.appendleft(val)

  # add data
  def add(self, data):
      assert(len(data) == self.nb_curve*2)
      if(self.nb_curve == 1):
        self.addToBuf(self.ax, data[0])
        self.addToBuf(self.ay, data[1])
      elif(self.nb_curve == 2):
        self.addToBuf(self.ax, data[0])
        self.addToBuf(self.ay, data[1])
        self.addToBuf(self.ay1, data[3])
      else:
        self.addToBuf(self.ax, data[0])
        self.addToBuf(self.ay, data[1])
        self.addToBuf(self.ay1, data[3])
        self.addToBuf(self.ay2, data[5])


  # update plot
  def update(self, frameNum, a0, a1, a2, a3):
      try:
          
          line = self.ser.readline().decode('utf-8')
          
          line = line.replace('\x00','')

          
          data = [float(val) for val in line.split()]
          data = data[0:self.nb_curve*2]
          if(len(data) == self.nb_curve*2):
            self.add(data[0:2*self.nb_curve])
            if(self.nb_curve == 1):
              xs.append(line.split()[0])
              ys.append(line.split()[1])
              
              a0.set_data(range(self.maxLen), self.ax)
              a1.set_data(range(self.maxLen), self.ay)
            elif(self.nb_curve == 2):
              xs.append(line.split()[0])
              ys.append(line.split()[1])
              a0.set_data(range(self.maxLen), self.ax)
              a1.set_data(range(self.maxLen), self.ay)
              xs1.append(line.split()[2])
              ys1.append(line.split()[3])
              a2.set_data(range(self.maxLen), self.ay1)

            else:
              xs.append(line.split()[0])
              ys.append(line.split()[1])
              a0.set_data(range(self.maxLen), self.ax)
              a1.set_data(range(self.maxLen), self.ay)
              xs1.append(line.split()[2])
              ys1.append(line.split()[3])
              a2.set_data(range(self.maxLen), self.ay1)
              xs2.append(line.split()[4])
              ys2.append(line.split()[5])
              a3.set_data(range(self.maxLen), self.ay2)
            
          
      except KeyboardInterrupt:
          print('exiting')
      
      return a0,

  # clean up
  def close(self):
      # close serial
      self.ser.flush()
          


def graph(ser,nb_curves,case):
  
  analogPlot = AnalogPlot(ser, 100,nb_curves)

	# set up animation
  fig = plt.figure()
  ax = plt.axes(xlim=(0, 100), ylim=(0, 1023))
  
  a0, = ax.plot([], [])
  a1, = ax.plot([], [])
  a2, = ax.plot([], [])
  a3, = ax.plot([], [])
  anim = animation.FuncAnimation(fig, analogPlot.update, 
	                         fargs=(a0, a1, a2, a3), 
	                         interval=10)


	# show plot
  plt.show()

	# clean up
  analogPlot.close()



# main() function
def main():
  global ser
  filename = "file.txt"
#COMMAND Definition
  print("(Press H for help)")
  print("Press Q to Quit")
  print("\n")
  while(True):
 
    
    message_out = input("COMMAND : ")
    # plot parameters
    try:
      if(message_out[0] == "G"):
        send(message_out)
        read()

        
        


      if(message_out[0] == "O"):
        case = 0
        try:
          nb_curves = int(message_out[1])
        except:
          nb_curves = 1
        if("H" in message_out): 
          print('Requesting PID test')
          filename = "PID_test.txt"
          case = 0

        if("S" in message_out): 
          print('Requesting Sensor output')
          filename = "Sensor_output.txt"
          case = 1

        graph(ser,nb_curves,case)
        print('Saving data in %s'% filename)
        with open('result.txt', 'w') as f:
          for i in range(len(xs)):
            f.write("%s %s\n" % (xs[i], ys[i]) )



      if(message_out[0] == "H"):
        print("G[0-1] : X[Position] Y[Position] F[Speed] T[Timeout] ")
        print("M[0-9] : H[Motor] P[PWM] S[Sensor] Q[SensitivitySensor]")
        print("O[1-3](nb_curves to display) : H[Motor] F[Speed] S[Sensor]")

      if(message_out[0] == "Q" or message_out[0] == "q" ):
        break

    except:
      print(" ")

    print(" ")
  

	  
 	  
  

# call main
if __name__ == '__main__':
  ser = serial.Serial(SERIAL_PORT, SERIAL_RATE)
  print('reading from serial port %s...' % SERIAL_PORT)
  print("\n")
  main()
  ser.close()