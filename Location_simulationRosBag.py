import rosbag
from sensor_msgs.msg import LaserScan
import numpy as np
import cv2
import math

from LikelihoodField import *
import time

from LIDAR_LD06_python_loder.CalcLidarData import * 
import yaml
import socket
import threading

Main_prosses_data=None #全局变量，用于存储主进程回传给定位程序的数据，例如机器人右转90度
def handle_socket(sock, port):
	conn, addr = sock.accept()
	while True:
		data = conn.recv(1024)
		if data:
			global Main_prosses_data
			Main_prosses_data=data
			# print(f'Received from {port}: ', data)


with open('config.yaml', 'r') as file:
	config = yaml.safe_load(file)
# 创建1个socket对象
sock1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# 绑定到对应的端口
sock1.bind(('localhost', config['socket']['recv_port']))
# 开始监听
sock1.listen(1)
# 创建并启动两个线程，分别处理两个连接
threading.Thread(target=handle_socket, args=(sock1,  config['socket']['send_port'])).start()

def send(text):
	try:
		client_socket.send(text.encode())
	except (BrokenPipeError, OSError):
		# 如果连接被关闭，或者其他网络错误，打印错误信息并退出循环
		print(f"Send data error in port:{config['socket']['send_port']}")

# 创建一个socket对象
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# 设置为非阻塞模式
client_socket.setblocking(False)
try:
    # 连接到服务器，后续把数据都发送到这个socket
    client_socket.connect(('localhost', 23333))
except :
    # 如果立即连接不上，忽略错误，后续会使用select来检查连接状态
    pass

# 主函数
if __name__ == '__main__':
	# 初始化OpenCV窗口
	cv2.namedWindow('view', cv2.WINDOW_AUTOSIZE)
	cv2.namedWindow('scan', cv2.WINDOW_AUTOSIZE)
   
	# 定义机器人的初始位置，包括x坐标，y坐标和角度
	bot_pos = np.array(config['robot']['init_pose'])

	if config['likelihood_field']['cache']:
		lf = LikelihoodField(config,config['likelihood_field']['path'],bot_pos)
	else:
		im = cv2.imread("maps/map1.png")
		m = np.asarray(im)
		m = cv2.cvtColor(m, cv2.COLOR_RGB2GRAY)
		m = m.astype(float) / 255.
		lf = LikelihoodField(config)
		lf.SetFieldImageFromMap(m)
		for l in range(lf.levels_):
			cv2.imshow('LikelihoodField{}'.format(l),lf.field_[l])
	

	   
	# 打开bag文件
	with rosbag.Bag('/home/arc/works/robot_2024/2D_slam/scan_bag/1.bag') as bag:
		frame_counter = 0  # 初始化帧计数器
    # 遍历bag文件中的每一帧
		for topic, msg, t in bag.read_messages():
			frame_counter += 1  # 每次循环，计数器加1
			if frame_counter % 1 != 0:  # 如果计数器不是5的倍数，跳过当前帧
				continue
			# 如果当前话题是激光雷达扫描数据
			print("=======================frame_counter:",frame_counter)
			if topic == '/LiDAR/LD06':
				# 将激光雷达扫描数据保存到字典中
				scan = {
					'ranges': msg.ranges,
					'intensities': msg.intensities,
					"angle_min": msg.angle_min,
					"angle_max": msg.angle_max,
					"angle_increment": msg.angle_increment,
					"rangle_min": msg.range_min,
					"range_max": msg.range_max,
					"intensity": msg.intensities
				}
			# # 获取激光雷达扫描数据

				# 创建新的字典
				sensorDatas = {
					'ranges': [r for r in scan['ranges'] if not math.isnan(r)],
					'angles': [scan['angle_max'] - i * scan['angle_increment'] for i, r in enumerate(scan['ranges']) if not math.isnan(r)]
				}
					# 运行你的显示代码

				if  len(sensorDatas['ranges']) == 0:
					continue

				if len(sensorDatas['angles']) !=len(sensorDatas['ranges']):
					print("error",len(sensorDatas['angles']),len(sensorDatas['ranges']))
					continue

				# 绘制机器人在地图上的位置和传感器数据
				sensorDatas['ranges'] = [x *100 for x in sensorDatas['ranges']]

				start_time = time.time()
				lf.Align(sensorDatas,Main_prosses_data)#测试结果20-60ms执行一次
				Main_prosses_data=None

				end_time = time.time()
				elapsed_time = end_time - start_time
				# print("AlignGaussNewton took {:.4f} seconds to process.".format(elapsed_time))
				position_text = "Position: ({:.2f}, {:.2f}, {:.2f})".format(lf.current_pose_[0], lf.current_pose_[1], np.rad2deg(lf.current_pose_[2]))
				print(position_text)
				# cv2.imshow('scan',lf.scan_map[2])
				img=cv2.addWeighted(lf.scan_map[lf.levels_-1], 0.5, lf.field_[lf.levels_-1], 0.5, 0)
				cv2.circle(img,(int(lf.current_pose_[0]+lf.scan_map[lf.levels_-1].shape[1]/3), int(lf.current_pose_[1]+lf.scan_map[lf.levels_-1].shape[0]/3)), int(3), (0,255,255), -1)
				cv2.imshow('scan',img)
				# cv2.imwrite("sacn.png",utils.Map2Image(lf.scan_map[2]))env

				print("=======================end=================")

				cv2.waitKey(60)






cv2.destroyAllWindows()