import socket
import threading
import time

class SocketHandler:
	def __init__(self, port, handler):
		# 创建socket对象
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		# 绑定到本地主机和指定端口
		self.sock.bind(('localhost', port))
		# 开始监听连接
		self.sock.listen(1)
		# 创建并启动处理连接的线程
		self.thread = threading.Thread(target=handler, args=(self.sock, port))
		self.thread.daemon = True
		self.thread.start()

def location_handle(sock, port): #处理定位用的回调程序
	conn, addr = sock.accept()
	while True:
		data = conn.recv(1024)
		if data:
			print(f'Received from {port}: ', data)

def yolo_handle(sock, port):  #处理yolo用的回调程序
	conn, addr = sock.accept()
	while True:
		data = conn.recv(1024)
		if data:
			print(f'Received from {port}: ', data)

def send_to_location(client_socket, text): #发送给定位程序的数据，例如自旋180°
	try:
		client_socket.send(text.encode())
	except (BrokenPipeError, OSError):
		print(f"Send data error in port 23334")

def connect_to_location(): #死循环连接定位程序socket，主要用于给定位程序发送反馈，，例如自旋180°
	client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	client_socket.setblocking(True)#阻塞模式
	connected = False
	while not connected:
		try:
			client_socket.connect(('localhost', 23334))
			connected = True
			print("connect to location success")
		except :
			time.sleep(1)
	return client_socket

if __name__ == '__main__':
	location_socket = SocketHandler(23333, location_handle)#创建定位程序数据接收socket
	yolo_socket = SocketHandler(23335, yolo_handle)#创建yolo数据接收socket
	client_socket = connect_to_location()#连接定位程序反馈socket
	while True:
		# 这里可以添加你的主循环代码
		time.sleep(5)
		send_to_location(client_socket,"turn right 90")#发送给定位程序的数据，例如自旋90°