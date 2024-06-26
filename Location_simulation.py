import numpy as np
import cv2
from LikelihoodField import *
import time

import json
import yaml
from Socket_fun import *

Main_prosses_data=None #全局变量，用于存储主进程回传给定位程序的数据，例如机器人右转90度
# 读取配置文件
with open('config.yaml', 'r',encoding='utf-8') as file:
	config = yaml.safe_load(file)
if config['test']['save_log']:
		# 获取当前时间并格式化为字符串
	now = datetime.datetime.now()
	formatted_now = now.strftime("%m%d%H%M%S")

#-------------------------------------处理与主逻辑程序之间的socket通信-------------------------------------

client2main = Client(config['socket']['port'])

# 主函数
if __name__ == '__main__':
	# 初始化OpenCV窗口
	cv2.namedWindow('scan', cv2.WINDOW_AUTOSIZE)
	# 定义机器人的初始位置，包括x坐标，y坐标和角度
	bot_pos = np.array(config['robot']['init_pose'])

	if config['likelihood_field']['cache']:
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
	with open(config['test']['use_log_path'], 'r') as f:


		frame_count=0
		# 主循环
		while(1):
			frame_count=frame_count+1
			# print(frame_count)
			#!!!!!!!!!!!!!!注意可以通过frame_count%n来控制模拟丢数据帧的情况，这对于一些极端测试比较由于!!!!!!!!!!!!!
			if frame_count % 1 != 0:  #frame_count %1为不丢失数据帧
				continue
			line = f.readline()
					# 解析json格式的数据
			sensorDatas = json.loads(line)
			if  len(sensorDatas['ranges']) == 0:
				continue

			if len(sensorDatas['angles']) !=len(sensorDatas['ranges']):
				print("error",len(sensorDatas['angles']),len(sensorDatas['ranges']))
				continue


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
				client2main.write(position_text)
			else:
				client2main.write("position_Error!")
			Main_prosses_data=None

			#self.scan_map.append(np.full((self.field_[l].shape[0],self.field_[l].shape[1]), 1, dtype=np.float32))
			img=cv2.addWeighted(lf.scan_map[lf.levels_-1], 0.7, lf.field_[lf.levels_-1], 0.5, 0)
			# img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR) #可以多颜色显示代码
			cv2.circle(img,(int(lf.current_pose_[0]+lf.scan_map[lf.levels_-1].shape[1]/3), int(lf.current_pose_[1]+lf.scan_map[lf.levels_-1].shape[0]/3)), int(3), (0,0,255), -1)
			cv2.imshow('scan',img)

			if cv2.waitKey(60) & 0xFF == 27:
				break

	cv2.destroyAllWindows()





			