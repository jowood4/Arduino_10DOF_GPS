import socket

#UDP_IP = "127.0.0.1"
UDP_IP = "192.168.240.255"
#UDP_IP = "192.168.1.243"
UDP_PORT = 49002

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP


sock.bind(('', UDP_PORT))
#sock.bind(('',0))
#sock.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)

while True:
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    print "received message:", data
