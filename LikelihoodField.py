import numpy as np
import math
import cv2

class LikelihoodField:
	def __init__(self,config,path=None, robotpose=None):
		# 初始化模型、场、层级、比例尺、当前位置和源
		self.model_ = [] # 似然场模板
		self.field_ = [] # 生成地图似然场
		self.levels_ = config['likelihood_field']['levels'] # 似然场层级
		self.ratios_ = config['likelihood_field']['ratios'] # 相比占据栅格图的比例尺,可以代表每层缩小多少倍
		self.current_pose_ = config['robot']['init_pose'].copy() #保存机器人当前位资
		self.original_pose= config['robot']['init_pose'].copy()
		self.pose_last= None
		self.pose_last_last= None
		self.source_ = None #传感器数据（待匹配的源）
		self.scan_map =list() #保存激光雷达扫描的扫描结果
		self.range_ = config['likelihood_field']['range']

		self.xy_limit = config['robot']['xy_limit']
		self.filter_angles=config['lidar']['filter_angles']
		self.trust_distance_range=config['lidar']['trust_distance_range']
		self.move_limit=config['robot']['move_limit']

		# 如果提供了路径，读取并处理之前已经保存的似然场图像
		if config['likelihood_field']['cache'] :
			for i in range(self.levels_):
				image = cv2.imread(f"{config['likelihood_field']['path']}/field_{i}.png", cv2.IMREAD_GRAYSCALE)
				image = image.astype(np.float32) / 255.0  # 将图像转换为浮点数并归一化
				self.field_.append(image)  # 将图像保存到self.field_中
			for l in range(self.levels_):
				self.scan_map.append(np.full((self.field_[l].shape[0],self.field_[l].shape[1]), 1, dtype=np.float32))
		

	def BuildModel(self):
		#建立似然场模板
		k=1/self.range_ #概率折算，最大概率除以最大范围
		for x in range(-self.range_, self.range_ + 1):
			for y in range(-self.range_, self.range_ + 1):
				self.model_.append((x, y, k*np.sqrt(x * x + y * y)))

	def SetFieldImageFromMap(self, occu_map):
		#建立似然场地图
		self.BuildModel()

		#初始化为所有像素点最大概率1，地图大小为3倍，以保证扫描结果能完全
		self.field_ = [
			np.full(( int(occu_map.shape[0]*self.ratios_[0]*3), int(occu_map.shape[1]*self.ratios_[0]*3)), 1, dtype=np.float32),
			np.full(( int(occu_map.shape[0]*self.ratios_[1]*3),  int(occu_map.shape[1]*self.ratios_[1]*3)), 1, dtype=np.float32),
			np.full(((occu_map.shape[0]*3, occu_map.shape[1]*3)), 1, dtype=np.float32),
		]

		for x in range(occu_map.shape[1], occu_map.shape[1]*2):		#遍历地图，对占用的点做一个数值上的凹陷模板，被占用的地方为0
			for y in range(occu_map.shape[0], occu_map.shape[0]*2):
				if occu_map[y-occu_map.shape[0], x-occu_map.shape[1]] < 0.5:	#占用概率小于0.5(黑色)的点为占用
					for l in range(self.levels_):
						for model_pt in self.model_:
							xx = int(x * self.ratios_[l] + model_pt[0])
							yy = int(y * self.ratios_[l] + model_pt[1])
							if 0 <= xx < self.field_[l].shape[1] and 0 <= yy < self.field_[l].shape[0] and self.field_[l][yy, xx] > model_pt[2]:
								self.field_[l][yy, xx] = model_pt[2]
		for l in range(self.levels_):
			field_mapped = (self.field_[l] * 255).astype(np.uint8)
			cv2.imwrite(f"maps/field_{l}.png", field_mapped)	#保存到地图
						
			self.scan_map.append(np.full((self.field_[l].shape[0],self.field_[l].shape[1]), 1, dtype=np.float32))
	

	def AlignInLevelGaussNewton(self,source,level,init_pose=None,iterations=20):
		# iterations = 20  # 迭代次数
		cost = 0  # 当前代价
		lastCost = 0  # 上一次的代价
		self.original_pose=self.current_pose_.copy()
		if init_pose != None:
			self.current_pose_ = init_pose  # 当前姿态，x，y，theta
		
		self.source_ = source
		min_effect_pts = 50  # 最小有效点数
		image_boarder = 1  # 预留图像边界

		self.has_outside_pts_ = False  # 是否有超出边界的点

		for iter in range(iterations):  # 迭代次数控制
			self.scan_map[level].fill(1)
			H = np.zeros((3, 3))  # Hessian矩阵
			b = np.zeros(3)  # 梯度
			cost = 0  # 重置代价

			effective_num = 0  # 有效点数

			for i in range(len(self.source_['ranges'])):  # 遍历所有的测量值
				r = self.source_['ranges'][i]  # 当前测量值

				if r < self.trust_distance_range[0] or r > self.trust_distance_range[1]:  # 如果测量值超出范围（3cm-800cm），则跳过
					continue
				angle = np.rad2deg(self.source_['angles'][i])  # 计算角度
				# 过滤掉不需要的角度
				skip = False
				for j in range(len(self.filter_angles)):
					if angle > self.filter_angles[j][0] and angle < self.filter_angles[j][1]:
						skip = True
						break
				if skip:
					continue

				angle=np.deg2rad(angle)
				theta = self.current_pose_[2] + angle # 计算当前姿态的角度
				#pf代表在地图图像上的坐标
				pf=[int(round(self.current_pose_[0]+r*np.cos(theta))*self.ratios_[level]+self.field_[level].shape[1]/3),
				int(round(self.current_pose_[1]+r*np.sin(theta))*self.ratios_[level]+self.field_[level].shape[0]/3) ]

				blober=2
				if pf[1]>self.field_[level].shape[0]/3*2+blober or pf[0]>self.field_[level].shape[1]/3*2+blober \
					or pf[1]<self.field_[level].shape[0]/3-blober or pf[0]<self.field_[level].shape[1]/3-blober:
					cost+=20
				if pf[1]>self.field_[level] .shape[0]-1 or pf[0]>self.field_[level] .shape[1]-1 \
					or pf[1]<0 or pf[0]<0:
					continue
				self.scan_map[level] [pf[1],pf[0]]=0 #扫描地图画上黑色点

				if image_boarder <= pf[0] < self.field_[level].shape[1] - image_boarder and image_boarder <= pf[1] < self.field_[level].shape[0] - image_boarder:  # 如果在有效范围内
					effective_num += 1  # 有效点数加一

					dx = 0.5 * (self.field_[level][pf[1], pf[0] + 1] - self.field_[level][pf[1], pf[0] - 1])  # 计算x方向的梯度
					dy = 0.5 * (self.field_[level][pf[1] + 1, pf[0]] - self.field_[level][pf[1] - 1, pf[0]])  # 计算y方向的梯度

					J = np.array([self.ratios_[level] * dx, self.ratios_[level] * dy, -self.ratios_[level] * dx * r * math.sin(theta) + self.ratios_[level]* dy * r * math.cos(theta)])  # 计算雅可比矩阵
					H += np.outer(J, J)  # 更新Hessian矩阵

					e = self.field_[level][pf[1], pf[0]]  # 计算误差
					b += -J * e  # 更新梯度
					cost += e * e  # 更新代价
				else:
					self.has_outside_pts_ = True

			if effective_num < min_effect_pts:
				return -1
			try:
				dx = np.linalg.solve(H, b)
			except:
				dx=np.dot(np.linalg.pinv(H),b)
				# print("singular H")
				# return -1
			if np.isnan(dx[0]):
				break

			cost /= effective_num
			if iter > 0 and cost >= lastCost:
				break

			# print(f"iter {iter} cost = {cost}, effect num: {effective_num}")
			#更新机器人姿态
			self.current_pose_[:2] += dx[:2]
			self.current_pose_[2] += dx[2]
			# 手动限制机器人的位置
			self.current_pose_[0] = max(2, min(self.current_pose_[0], self.xy_limit[0]))
			self.current_pose_[1] = max(2, min(self.current_pose_[1], self.xy_limit[1]))
			if(abs(self.current_pose_[0]-self.original_pose[0])>self.move_limit \
	  			or abs(self.current_pose_[1]-self.original_pose[1])>self.move_limit):
				print("point move too far")
				self.current_pose_[0] = max(self.original_pose[0]-self.move_limit, min(self.current_pose_[0], self.original_pose[0]+self.move_limit))
				self.current_pose_[1] = max(self.original_pose[1]-self.move_limit, min(self.current_pose_[1],self.original_pose[1]+self.move_limit))
				return -1
			lastCost = cost

		return cost


	def Align(self, source_,Main_prosses_data=None):
		# # 先粗略计算机器人位置差
		if self.pose_last is not None and self.pose_last_last is not None:
			self.current_pose_[0]+=(self.pose_last[0]-self.pose_last_last[0])/2
			self.current_pose_[1]+=(self.pose_last[1]-self.pose_last_last[1])/2
			self.current_pose_[2]+=(self.pose_last[2]-self.pose_last_last[2])/2
		# 记录当前位置
		original_pose = self.current_pose_.copy()
		# self.AlignInLevelGaussNewton(source_, self.levels_-2)  # 从最后一层开始配准
		cost=self.AlignInLevelGaussNewton(source_, self.levels_-1)  # 直接从第二层开始配准
		print(Main_prosses_data)
		if cost>3:
			print("cost too large")
			# self.current_pose_=self.original_pose.copy()
			# 地图感知漂移了，需要其他方式重定位。


		# # print(f"cost = {cost}")
		if self.pose_last is not None:
			self.pose_last_last = self.pose_last.copy()
		self.pose_last=self.current_pose_.copy()


		