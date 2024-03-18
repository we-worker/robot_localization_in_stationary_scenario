import numpy as np
import cv2
from LikelihoodField import *
import time
import serial
from LIDAR_LD06_python_loder.CalcLidarData import * 
from multiprocessing import Process ,Manager
import copy
import json
import yaml
import datetime
import socket
import threading

Main_prosses_data=None #全局变量，用于存储主进程回传给定位程序的数据，例如机器人右转90度
# 读取配置文件
with open('config.yaml', 'r',encoding='utf-8') as file:
	config = yaml.safe_load(file)
if config['test']['save_log']:
		# 获取当前时间并格式化为字符串
	now = datetime.datetime.now()
	formatted_now = now.strftime("%m%d%H%M%S")

#-------------------------------------处理与主逻辑程序之间的socket通信-------------------------------------
def handle_socket(sock, port):
	conn, addr = sock.accept()
	while True:
		data = conn.recv(1024)
		if data:
			global Main_prosses_data
			Main_prosses_data=data
			print(f'Received from {port}: ', data)

sock1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)# 创建1个socket对象
sock1.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock1.bind(('localhost', config['socket']['recv_port']))# 绑定到对应的端口
sock1.listen(1)# 开始监听
thread1 = threading.Thread(target=handle_socket, args=(sock1,  config['socket']['recv_port']))
thread1.daemon = True
thread1.start()

#发送给主逻辑程序的数据，建立socket链接
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.setblocking(False)# 设置为非阻塞模式
try:
	client_socket.connect(('localhost', config['socket']['send_port']))
except:
	pass

def send(text):
	try:
		client_socket.send(text.encode())
	except :
		# 如果连接被关闭，或者其他网络错误，打印错误信息并退出循环
		print(f"Send data error in port:{config['socket']['send_port']}")

#处理激光雷达数据的进程
def lidar_pross(dist_angles):
    ser = serial.Serial(port=config['serial']['port'],
                        baudrate=230400,
                        timeout=5.0,
                        bytesize=8,
                        parity='N',
                        stopbits=1)
    lidar=Lidar_Datas(ser, dist_angles)
    lidar.read_and_parse_data()


#-------------------------------------下面开始是定位的主程序-------------------------------------
# 主函数
if __name__ == '__main__':
	# 初始化OpenCV窗口
	cv2.namedWindow('scan', cv2.WINDOW_AUTOSIZE)

	# 定义机器人的初始位置，包括x坐标，y坐标和角度
	bot_pos = np.array(config['robot']['init_pose'])

	if config['likelihood_field']['cache']: #如果使用缓存似然场
		lf = LikelihoodField(config,config['likelihood_field']['path'],bot_pos)
	else:
		im = cv2.imread(config['map']['path'])
		m = np.asarray(im)
		m = cv2.cvtColor(m, cv2.COLOR_RGB2GRAY)
		m = m.astype(float) / 255.
		lf = LikelihoodField(config)
		lf.SetFieldImageFromMap(m)
		for l in range(lf.levels_):
			cv2.imshow('LikelihoodField{}'.format(l),lf.field_[l])
	

	sensorDatas={
		'ranges': list(),  # 测量值数组
		'angles': list()
	}

	# 获取机器人的传感器数据
	with Manager() as manager:
		# distances = manager.list()  # 空字典
		# angles=manager.list()
		dist_angles = manager.dict()
		dist_angles['ranges'] = list()
		dist_angles['angles'] = list()
		dist_angles['is_new']=False

		lidar = Process(target=lidar_pross, args=(dist_angles,))
		lidar.start()

		frame_count=0
		# 主循环
		while(1):

			if dist_angles['is_new']:
					dist_angles['is_new']=False
					sensorDatas=copy.deepcopy(dist_angles)
			else :
				continue
			if  len(sensorDatas['ranges']) == 0:
				continue

			if len(sensorDatas['angles']) !=len(sensorDatas['ranges']):
				print("error",len(sensorDatas['angles']),len(sensorDatas['ranges']))
				continue
			# # 把sensorDatas按照python字典的格式保存到文件中，多次保存，不要覆盖
			if config['test']['save_log']:
				with open(f"{config['test']['save_path']}/log_{formatted_now}.json", 'a+') as f:
					f.write(json.dumps(sensorDatas) + '\n')

		 # 绘制机器人在地图上的位置和传感器数据
			sensorDatas['ranges'] = [x *10 for x in sensorDatas['ranges']]

				# start_time = time.time()
			if lf.Align(sensorDatas,Main_prosses_data): #测试结/s果0.01-0.05ms执行一次

				#保留了测用时的代码
				# end_time = time.time()
				# elapsed_time = end_time - start_time
				# if elapsed_time>0.01:
				# 	print("AlignGaussNewton took {:.2f} seconds to process.".format(elapsed_time))
				position_text = "Position: ({:.2f}, {:.2f}, {:.2f})".format(lf.current_pose_[0], lf.current_pose_[1], np.rad2deg(lf.current_pose_[2]))
				print(position_text)
				send(position_text)
			else:
				send("position_Error!")
			Main_prosses_data=None
			
			img=cv2.addWeighted(lf.scan_map[lf.levels_-1], 0.7, lf.field_[lf.levels_-1], 0.5, 0)
			# img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR) #可以多颜色显示代码
			cv2.circle(img,(int(lf.current_pose_[0]+lf.scan_map[lf.levels_-1].shape[1]/3), int(lf.current_pose_[1]+lf.scan_map[lf.levels_-1].shape[0]/3)), int(3), (0,255,255), -1)
			cv2.imshow('scan',img)

			cv2.waitKey(1)

	cv2.destroyAllWindows()

