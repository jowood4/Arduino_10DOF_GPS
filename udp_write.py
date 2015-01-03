#!/usr/bin/python

from socket import *
import sys, os.path, time
#sys.path.insert(0, '/usr/lib/python2.7/bridge/')
#from bridgeclient import BridgeClient as bridgeclient
import serial

file_num = 0
file_name = 'gps_data'
#print 'test'
#print ('mnt/sda1/arduino/gps_data/' + file_name + str(file_num))
while os.path.isfile("/mnt/sda1/arduino/gps_data/" + file_name + str(file_num) + '.txt'):
	file_num = file_num + 1

fileio = open(("/mnt/sda1/arduino/gps_data/" + file_name + str(file_num) + '.txt'), "w", 1)

ser = serial.Serial('/dev/ttyATH0',9600)
#BC = bridgeclient()
UDP_IP = "192.168.240.255"
UDP_PORT = 49002
#MESSAGE = sys.argv[1]

#print "UDP target IP:", UDP_IP
#print "UDP target port:", UDP_PORT
#print "message:", MESSAGE

sock = socket(AF_INET, # Internet
                     SOCK_DGRAM) # UDP

sock.bind(('', 0))
sock.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)

while True:
	#MESSAGE = BC.get('string')
	MESSAGE = ser.readline()
	#print MESSAGE
	fileio.write(time.asctime(time.localtime(time.time())))
	fileio.write(MESSAGE)
	sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))

fileio.close()
sock.close()
