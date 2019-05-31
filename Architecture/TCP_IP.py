import socket, select

def enable_tcp_com(BUFFER_SIZE):
    TCP_IP = '192.168.43.58' 
    TCP_PORT = 5005

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((TCP_IP, TCP_PORT))
    s.listen(1)
    #s.setblocking(0)
    conn, addr = s.accept()
    return conn, addr

def check_data_received(conn, BUFFER_SIZE):
    ready = select.select([conn], [], [], 0.2)
    if ready[0]:
        data = conn.recv(BUFFER_SIZE)
        conn.send(data)  # echo
        return data.decode().split(';')[:-1]
    else :
        
        return None
    
def close_connection(conn):
    conn.close()