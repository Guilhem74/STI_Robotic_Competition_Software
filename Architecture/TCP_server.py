
import socket, time, sys
import threading
import pprint
TCP_IP = '127.0.0.1'
TCP_PORT = 51331 
BUFFER_SIZE = 1024
from collections import deque


class server():

    def __init__(self):
        self.CLIENTS = []        
        

    def startServer(self,buf_broadcast,buf_stm,clientCount):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.bind((TCP_IP,TCP_PORT))
            s.listen(10)
            while 1:
                client_socket, addr = s.accept()
                print ('Connected with ' + addr[0] + ':' + str(addr[1]))
                
                clientCount = clientCount+1
                print (clientCount)
                # register client
                self.CLIENTS.append(client_socket)
                threading.Thread(target=self.playerHandler, args=(client_socket,buf_broadcast,buf_stm)).start()
            s.close()
        except socket.error as msg:
            print ('Could Not Start Server Thread. Error Code : ') #+ str(msg[0]) + ' Message ' + msg[1]
            sys.exit()


   #client handler :one of these loops is running for each thread/player   
    def playerHandler(self, client_socket,buf_broadcast,buf_stm):
        #send welcome msg to new client
        client_socket.send(bytes('Connected to Server', 'UTF-8'))
        client_socket.settimeout(0.01)#if buffer empty after 0.01 sec, move on
        while 1:
            try:
                data = client_socket.recv(BUFFER_SIZE)
                data = data.decode("UTF-8")
                buf_broadcast.append(data)
                buf_stm.append(data)
            except socket.timeout:
                pass
            try:
                while buf_broadcast:
                    m = buf_broadcast.popleft()
                    if m != '':
                        for c in self.CLIENTS:
                            c.send(bytes(m,'UTF-8'))
            except IndexError:
                pass

         # the connection is closed: unregister
        self.CLIENTS.remove(client_socket)
        #client_socket.close() #do we close the socket when the program ends? or for ea client thead?



