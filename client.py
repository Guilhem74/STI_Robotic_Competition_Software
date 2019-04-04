##Client

import socket
import sys
import time
from collections import deque
import threading
buf_IN = deque([''])
buf_OUT = deque([''])
connected = False
class client:


	    
	def run(self,buf_IN,buf_OUT,connected):
		connected = True
		
		buff = ""
		#connect to server
		client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		client_socket.connect(('192.168.0.23',51331))
		client_socket.settimeout(0.01)
		
		
		while connected:
			try:
				client_socket.sendall(buf_OUT.popleft().encode('utf-8'))
			except:
				pass
			
			try:
				data = client_socket.recv(1024)
				buf_IN.append(data)
			except socket.timeout:
				pass	
				
			
				
			
		
		client_socket(close)
		connected = False
            
	#wait for server commands to do things, now we will just display things
	     
class console:
	def main(self,buf_IN,buf_OUT,connected):
		
		filename = "file.txt"
		#COMMAND Definition
		print("(Press H for help)")
		print("Press Q to Quit")
		print("\n")
		while True:


			message_out = input("COMMAND : ")
			# plot parameters
			message_out = message_out + "\r\n"
			try:
				if(message_out[0] == "G"):
					buf_OUT.append(message_out)
					print(buf_OUT)
				if(message_out[0] == "M"):
					buf_OUT.append(message_out)
					print(buf_OUT)
				if(message_out[0] == "O"):
					print(buf_IN)
		
				if("H" in message_out): 
					print('Requesting PID test')
					filename = "PID_test.txt"
					case = 0

				if("S" in message_out): 
					print('Requesting Sensor output')
					filename = "Sensor_output.txt"
					case = 1
			    



				if(message_out[0] == "H"):
					print("G[0-1] : X[Position] Y[Position] F[Speed] T[Timeout] ")
					print("M[0-9] : H[Motor] P[PWM] S[Sensor] Q[SensitivitySensor]")
					print("O[1-3](nb_curves to display) : H[Motor] F[Speed] S[Sensor]")
					print("S, Display one Serial input")
				if(message_out[0] == "S"):
					print("S")
			    

				if(message_out[0] == "Q" or message_out[0] == "q" ):
					connected = False
					break
			  
			except:
				print(" ")

			print(" ")
		print("EXIT CONSOLE")

	  
 	  
  

# call main
if __name__ == '__main__':
	c = client()
	cons = console()
	threading.Thread(target=c.run, args=(buf_IN,buf_OUT,connected)).start()
	threading.Thread(target=cons.main, args=(buf_IN,buf_OUT,connected)).start()
	
	