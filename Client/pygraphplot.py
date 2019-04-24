
import socket
import sys
import time
from collections import deque
import itertools
import threading
from Parser_client import *
buf_IN = deque([''])
buf_OUT = deque([''])
connected = False
action_In = deque([''])
action_Out = deque([''])

class client:


        
    def run(self,buf_IN,buf_OUT,connected):
        connected = True
        
        buff = ""
        #connect to server
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect(('192.168.1.30',51331))
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
            
    #wait for server commands to do things, now we will just display things
         


      
      
from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg
from pyqtgraph.dockarea import *
#QtGui.QApplication.setGraphicsSystem('raster')
app = QtGui.QApplication([])
mw = QtGui.QMainWindow()
mw.setWindowTitle('pyqtgraph example: PlotWidget')
mw.resize(800,800)
#cw = QtGui.QWidget()

area = DockArea()
mw.setCentralWidget(area)
d1 = Dock("Dock1", size=(1, 1), closable=True) 
d2 = Dock("Dock2", size=(1, 1)) 
area.addDock(d1, 'left')  
area.addDock(d2, 'right')  
#cw.setLayout(l)

pw = pg.PlotWidget(name='Plot1')  ## giving the plots names allows us to link their axes together
#l.addWidget(pw)

i = 0
data = []
data0 = []
mw.show()

## Create an empty plot curve to be filled later, set its pen
p1 = pw.plot()
p1.setPen((200,200,100))
p2 = pw.plot()
p2.setPen((0,200,100))



pw.setLabel('left', 'Value', units='V')
pw.setLabel('bottom', 'Time', units='s')
d1.addWidget(pw)
serveur_co = QtGui.QPushButton('Connect Serveur')
send_b = QtGui.QPushButton('Send Command')
d2.addWidget(serveur_co)

Y_display = QtGui.QCheckBox(mw)
Y_display.setText("Y")
d1.addWidget(Y_display)
A_display = QtGui.QCheckBox(mw)
Y_display.setText("A")
d1.addWidget(A_display)

connect_ini = False


list_cmd = []
list_input = []

    
    

def circular(li,a):
    if(len(li) >=25):
        li.pop(0)
        li.append(a)
    else:
        li.append(a)
    return li


def maxdisplay(my_deque):
    deque_slice  = deque([''])
    max_len = 25
    if(len(my_deque) >=max_len):
        deque_slice = deque(itertools.islice(my_deque, len(my_deque) - max_len, len(my_deque)))
    return deque_slice

label = QtGui.QLabel(("Enter command and press Return"))
com = QtGui.QLineEdit()
def run_command():
    global list_cmd
    global buf_OUT
    cmd = str(com.text())
    buf_OUT.append(cmd)
    list_cmd = circular(list_cmd,cmd)
    com.setText("")
    hist = "\n".join(list_cmd)
    label.setText(hist)


d2.addWidget(label)
d2.addWidget(com)
send_b.clicked.connect(run_command)
d2.addWidget(send_b)
label2 = QtGui.QLabel(("INPUT"))
d2.addWidget(label2)


def updateCurves():
    global action_In
    try:
        dic = action_In.popleft()
        x = dic['X']
        y = dic['Y']
        a = dic['A']
    except:
        pass
    data0.append(y)
    data.append(a)    
    return data, np.arange(0, len(data)) ,data0 , np.arange(0, len(data0))
    

def updateData():
    global i
    global action_In
    global connect_ini
    hist = ""
    try:
        yd, xd ,yd1,xd1= updateCurves()
        if Y_display.isChecked():
            p1.setData(y=yd, x=xd)
        if A_display.isChecked():
            p2.setData(y=yd1, x=xd1)
    except:
        print("Can't load new data")
        pass

    list_input = list(maxdisplay(action_In))
    for i in list_input:
        hist+=str(i)
        hist+="\n"

    label2.setText(hist)






## Start a timer to rapidly update the plot in pw
t = QtCore.QTimer()
t.timeout.connect(updateData)
t.start(50)






## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys
    c = client()   
    p = Parser_client()
    threading.Thread(target=c.run, args=(buf_IN,buf_OUT,connected)).start()
    threading.Thread(target=p.run, args=(buf_IN,buf_OUT,action_In,action_Out)).start() 
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
        