import socket
import threading
# from Move_contorller import *
import time

#处理定位用的回调程序
def location_handle(sock, port):
	conn, addr = sock.accept()
	while True:
		data = conn.recv(1024)
		if data:
			print(f'Received from {port}: ', data)

#处理yolo用的回调程序
def yolo_handle(sock, port):
	conn, addr = sock.accept()
	while True:
		data = conn.recv(1024)
		if data:
			print(f'Received from {port}: ', data)

# 创建两个socket对象
sock1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock1.bind(('localhost', 23333))# 绑定到对应的端口
sock2.bind(('localhost', 23335))
sock1.listen(1)# 开始监听
sock2.listen(1)

# 创建并启动两个线程，分别处理两个连接
threading.Thread(target=location_handle, args=(sock1, 23333)).start()
threading.Thread(target=yolo_handle, args=(sock2, 23335)).start()

#发送给定位程序的数据
def send_to_location(text):
	try:
		client_socket.send(text.encode())
	except (BrokenPipeError, OSError):
		# 如果连接被关闭，或者其他网络错误，打印错误信息并退出循环
		print(f"Send data error in port 23334")

# 创建一个socket对象
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.setblocking(False)# 设置为非阻塞模式
#下面的代码是为了保证连接定位成功
connected = False
while not connected:
	try:
		# 连接到服务器，后续把数据都发送到这个socket
		client_socket.connect(('localhost', 23334))
		connected = True
	except :
		# 如果立即连接不上，忽略错误
		pass


if __name__ == '__main__':
	while True:
		time.sleep(5)
		send_to_location("turn right 90")