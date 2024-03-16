import rosbag
from sensor_msgs.msg import LaserScan
import numpy as np
import cv2
import math
import utils


from SingleBotLaser2D import *
from LikelihoodField import *
import time

from LIDAR_LD06_python_loder.CalcLidarData import * 





# 绘制地图和机器人位置的函数
def Draw(img_map, scale, bot_pos, sensor_data, bot_param):
	img = img_map.copy()
	img = cv2.resize(img, (round(scale*img.shape[1]), round(scale*img.shape[0])), interpolation=cv2.INTER_LINEAR)
	img = utils.Map2Image(img)
	plist = utils.EndPoint(bot_pos, bot_param, sensor_data)
	for pts in plist:
		cv2.line(
			img, 
			(int(scale*bot_pos[0]), int(scale*bot_pos[1])), 
			(int(scale*pts[0]), int(scale*pts[1])),
			(255,0,0), 1)

	cv2.circle(img,(int(scale*bot_pos[0]), int(scale*bot_pos[1])), int(3*scale), (0,0,255), -1)
	return img







# 主函数
if __name__ == '__main__':
	# 初始化OpenCV窗口
	cv2.namedWindow('view', cv2.WINDOW_AUTOSIZE)
	cv2.namedWindow('scan', cv2.WINDOW_AUTOSIZE)
   # 


	# 初始化2D环境
	# 定义机器人参数，包括传感器点数，起始角度，结束角度，最大距离，速度和角速度
	bot_param = [468,0, 360, 800.0, 6.0, 6.0]
	# 定义机器人的初始位置，包括x坐标，y坐标和角度
	bot_pos = np.array([100, 250.00, -3.14/2])
	# 创建一个2D激光机器人环境，输入参数为机器人位置，机器人参数和地图图片
	env = SingleBotLaser2Dgrid(bot_pos, bot_param, 'maps/map1.png')
	# 初始化GridMap，定义地图参数，包括占用概率，空闲概率，最大概率和最小概率
	map_param = [0.4, -0.4, 5.0, 0] 
	


	# 显示地图的概率分布

	# lf = LikelihoodField()
	# lf.SetFieldImageFromMap(env.img_map)
	lf = LikelihoodField('maps/field',bot_pos)
	#  设置场图像,env.img_mapw为0-1的占用概率图像

	cv2.imshow('LikelihoodField0',lf.field_[0])
	cv2.imshow('LikelihoodField1',lf.field_[1])
	# cv2.imshow('LikelihoodField2',lf.field_[2])
   

	   
	# 打开bag文件
	with rosbag.Bag('/home/arc/works/robot_2024/2D_slam/scan_bag/super_bad.bag') as bag:
		# 遍历bag文件中的每一帧
		for topic, msg, t in bag.read_messages():
			# 如果当前话题是激光雷达扫描数据
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

				img = Draw(env.img_map, 1, lf.current_pose_, sensorDatas, env.bot_param)
			
				cv2.circle(img,(int(lf.current_pose_[0]), int(lf.current_pose_[1])), int(3), (0,255,255), -1)
				cv2.imshow('view',img)


				start_time = time.time()
				lf.Align(sensorDatas)#测试结果20-60ms执行一次
				# lf.AlignInLevelGaussNewton(sensorDatas, 0)
				# lf.AlignInLevelGaussNewton(sensorDatas, 1)
				# lf.AlignInLevelGaussNewton(sensorDatas, 2)
				# lf.AlignInLevelGaussNewton(sensorDatas, 2)
				end_time = time.time()
				elapsed_time = end_time - start_time
				print("AlignGaussNewton took {:.2f} seconds to process.".format(elapsed_time))
				position_text = "Position: ({:.2f}, {:.2f}, {:.2f})".format(lf.current_pose_[0], lf.current_pose_[1], lf.current_pose_[2])
				print(position_text)
				# cv2.imshow('scan',lf.scan_map[2])
				cv2.imshow('scan',cv2.addWeighted(lf.scan_map[2], 0.5, lf.field_[2], 0.5, 0))
				# cv2.imwrite("sacn.png",utils.Map2Image(lf.scan_map[2]))


				cv2.waitKey(1)






cv2.destroyAllWindows()