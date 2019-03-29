
import sys, serial, argparse
import numpy as np
from time import sleep
from collections import deque

import matplotlib.pyplot as plt 
import matplotlib.animation as animation

SERIAL_PORT = 'COM5'
SERIAL_RATE = 115200

xs = []
ys = []
comIN = {} #Dictonnary that keep the last command received from STM32
comOUT = {} #Dictonnary that store the next command to emit toSTM32   

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
  def __init__(self, SERIAL_PORT, maxLen):
      # open serial port
      self.ser = serial.Serial(SERIAL_PORT, SERIAL_RATE)

      self.ax = deque([0.0]*maxLen)
      self.ay = deque([0.0]*maxLen)
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
      assert(len(data) == 2)
      self.addToBuf(self.ax, data[0])
      self.addToBuf(self.ay, data[1])

  # update plot
  def update(self, frameNum, a0, a1):
      try:
          
          line = self.ser.readline().decode('utf-8')
          line = line.replace('\x00','')

          xs.append(1)
          ys.append(2)
          data = [float(val) for val in line.split()]
          
          if(len(data) == 2):
              self.add(data)
              a0.set_data(range(self.maxLen), self.ax)
              a1.set_data(range(self.maxLen), self.ay)
      except KeyboardInterrupt:
          print('exiting')
      
      return a0, 

  # clean up
  def close(self):
      # close serial
      self.ser.flush()
      self.ser.close()    


def graph():
	analogPlot = AnalogPlot(SERIAL_PORT, 100)

	print('Requesting PID test')

	# set up animation
	fig = plt.figure()
	ax = plt.axes(xlim=(0, 100), ylim=(0, 1023))
	a0, = ax.plot([], [])
	a1, = ax.plot([], [])
	anim = animation.FuncAnimation(fig, analogPlot.update, 
	                         fargs=(a0, a1), 
	                         interval=50)

	# show plot
	plt.show()

	# clean up
	analogPlot.close()



# main() function
def main():
  



  print('reading from serial port %s...' % SERIAL_PORT)

  message_out = input("Type something to test this out: ")
  # plot parameters
  if(message_out[0] == "O"):
  	graph()
	  

  filename = "PID_test.txt"
  print('Saving data in %s'% filename)
  with open('your_file.txt', 'w') as f:
  	for i in range(len(xs)):
  		f.write("%s %s\n" % (xs[i], ys[i]) )

	  
 	  
  

# call main
if __name__ == '__main__':
  main()