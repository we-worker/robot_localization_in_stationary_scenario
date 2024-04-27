import socket
import threading
import os



class Server:
	def __init__(self, port, read_handler=None):
		# 创建socket对象
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		# 绑定到本地主机和指定端口
		self.sock.bind(('localhost', port))
		self.read_handler = read_handler
		self.conn = None
		# 创建并启动处理连接的线程
		threading.Thread(target=self.run,daemon=True).start()
	def run(self):
		self.sock.listen()# 开始监听连接
		print("Serving...")
		# while True:
		print("Waiting for connection...")
		self.conn, addr = self.sock.accept()
		print("Recived new conn: %s from %s", self.conn, addr)
		if self.read_handler is not None:
			threading.Thread(target=self.read_handler, args=(self.conn,),daemon=True).start()
	def write(self,msg):
		""" 向连接中发送 """
		try:
			self.conn.send(msg.encode())
		except:
			print("Send data error")

class Client:
	""" socket 客户端 """
	def __init__(self, port):
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.addr = ('localhost', port)
		self.port=port

		try:
			self.sock.connect(self.addr)
			threading.Thread(target=self.read,daemon=True).start()
		except:
			print("Connect failed")

	def read(self):
		""" 向连接中接受数据 """
		while True:
			try:
				data = self.sock.recv(1024).decode()
				global Main_prosses_data
				Main_prosses_data=data
				print(f'Received from {self.port}: ', data)
			except Exception as e:
				print("recv failed: %s", e)
				return

	def write(self,msg):
		""" 向连接中发送 """
		try:
			self.sock.send(msg.encode())
		except:
			print("Send data error")

def kill_port(port):
	# 下面代码是用来关闭占用端口的程序，注意处理意外关闭程序导致的，后台进程无法关闭，占用端口问题
	with os.popen(f'netstat -aon|findstr "{port}"') as res:
		res = res.read().split('\n')
		result = []
		try:
			for line in res:
				temp = [i for i in line.split(' ') if i != '']
				if len(temp) > 4:
					print({'pid': temp[4], 'address': temp[1], 'state': temp[3]})
					os.popen(f"taskkill -pid {temp[4]} -f")
		except:
			pass