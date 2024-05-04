import socket
import time
from Socket_fun import *

def location_read(conn: socket.socket = None): #处理定位用的回调程序
	while True:
		data = conn.recv(1024)
		if data:
			print(f'Received from Location: ', data)

def yolo_read(conn: socket.socket = None):  #处理yolo用的回调程序
	while True:
		data = conn.recv(1024)
		if data:
			print(f'Received from yolo: ', data)

if __name__ == '__main__':
	kill_port(23333)#关闭占用23333端口的程序
	kill_port(23334)#关闭占用23334端口的程序
	location_socket = Server(23333, location_read)#创建定位程序数据接收socket
	yolo_socket = Server(23334, yolo_read)#创建定位程序数据接收socket
	while True:
		# 这里可以添加你的主循环代码
		location_socket.write("turn right 90")#发送给定位程序的数据，例如自旋90°
		time.sleep(5)