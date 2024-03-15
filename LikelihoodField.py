import numpy as np
import math
import cv2

class LikelihoodField:
	def __init__(self, path=None, robotpose=None):
		# 初始化模型、场、层级、比例尺、当前位置和源
		self.model_ = [] # 似然场模板
		self.field_ = [] # 生成地图似然场
		self.levels_ = 3 # 似然场层级
		self.ratios_ = [0.05,0.2, 1]  # 相比占据栅格图的比例尺,可以代表每层缩小多少倍
		self.current_pose_ = [0, 0, 0] #保存机器人当前位资
		self.original_pose= [0,0,0]
		self.pose_last= None
		self.pose_last_last= None
		self.source_ = None #传感器数据（待匹配的源）
		self.scan_map = [0, 0, 0] #保存激光雷达扫描的扫描结果

		# 如果提供了路径，读取并处理之前已经保存的似然场图像
		if path is not None:
			for i in range(self.levels_):
				image = cv2.imread(f"{path}_{i}.png", cv2.IMREAD_GRAYSCALE)
				image = image.astype(np.float32) / 255.0  # 将图像转换为浮点数并归一化
				self.field_.append(image)  # 将图像保存到self.field_中

		# 如果提供了机器人位置估计，设置当前位置
		if robotpose is not None:
			self.current_pose_ = robotpose

	def BuildModel(self):
		#建立似然场模型
		range_ = 25	#50*50大小。
		k=1/range_ #概率折算，最大概率除以最大范围
		for x in range(-range_, range_ + 1):
			for y in range(-range_, range_ + 1):
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

	def AlignInLevelGaussNewton(self,source,level,init_pose=None):
		iterations = 20  # 迭代次数
		cost = 0  # 当前代价
		lastCost = 0  # 上一次的代价
		self.original_pose=self.current_pose_.copy()
		if init_pose != None:
			self.current_pose_ = init_pose  # 当前姿态，x，y，theta
		
		self.source_ = source
		min_effect_pts = 200  # 最小有效点数
		image_boarder = 1  # 预留图像边界

		self.has_outside_pts_ = False  # 是否有超出边界的点
		# print(self.source_)
		for iter in range(iterations):  # 迭代次数控制
			self.scan_map[level] = np.full((self.field_[level].shape[0],self.field_[level].shape[1]), 1, dtype=np.float32)
			H = np.zeros((3, 3))  # Hessian矩阵
			b = np.zeros(3)  # 梯度
			cost = 0  # 重置代价

			effective_num = 0  # 有效点数

			for i in range(len(self.source_['ranges'])):  # 遍历所有的测量值
				r = self.source_['ranges'][i]  # 当前测量值
				# if r < self.source_['range_min'] or r > self.source_['range_max']-1:  # 如果测量值超出范围，则跳过
				# 	continue
				if r < 3 or r > 800-1:  # 如果测量值超出范围（3cm-800cm），则跳过
					continue
				angle = np.rad2deg(self.source_['angles'][i])  # 计算角度
				# 留出四个轮子的角度，放弃传感器信息。
				if( (angle>30 and angle<60) and (angle>120 and angle<150) and (angle >210 and angle <240) and (angle >300 and angle <330)):
					continue
				angle=np.deg2rad(angle)
				# theta = self.current_pose_[2]  
				theta = self.current_pose_[2] + angle # 计算当前姿态的角度
				#pf代表在地图图像上的坐标
				pf=[int(round(self.current_pose_[0]+r*np.cos(theta))*self.ratios_[level]+self.field_[level].shape[1]/3),
				int(round(self.current_pose_[1]+r*np.sin(theta))*self.ratios_[level]+self.field_[level].shape[0]/3) ]

				blober=2
				if pf[1]>self.field_[level].shape[0]/3*2+blober or pf[0]>self.field_[level].shape[1]/3*2+blober \
					or pf[1]<self.field_[level].shape[0]/3-blober or pf[0]<self.field_[level].shape[1]/3-blober:
					# print("point out of 图像")
					# continue
					cost+=20
				if pf[1]>self.field_[level] .shape[0]-1 or pf[0]>self.field_[level] .shape[1]-1 \
					or pf[1]<0 or pf[0]<0:
					# print("point out of 图像")
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
				# print("singular H")
				return -1
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
			self.current_pose_[0] = max(0, min(330, self.current_pose_[0]))
			self.current_pose_[1] = max(0, min(280, self.current_pose_[1]))
			if(abs(self.current_pose_[0]-self.original_pose[0])>self.field_[-1].shape[1]/3/5 \
	  			or abs(self.current_pose_[1]-self.original_pose[1])>self.field_[-1].shape[0]/3/5):
				# print("point move too far")
				return -1
			lastCost = cost

		return cost

	def Align(self, source_):
		# if self.pose_last is not None and self.pose_last_last is not None:
		# 	self.current_pose_[0]+=(self.pose_last[0]-self.pose_last_last[0])/2
		# 	self.current_pose_[1]+=(self.pose_last[1]-self.pose_last_last[1])/2
		# 	self.current_pose_[2]+=(self.pose_last[2]-self.pose_last_last[2])/2
		# 记录当前位置
		original_pose = self.current_pose_.copy()
		# 先粗略计算机器人位置差
		cost = self.AlignInLevelGaussNewton(source_, 0)
		temp_pose=self.current_pose_.copy()
		print("cost: ", cost)
		# 机器人位置差大于0.1，进行多角度计算
		if cost > 0.05 or cost==-1:
			# 将360度分为5个部分
			angles = np.linspace(0, 2 * np.pi, 4)
			# 对每个角度进行配准，并找到成本最小的角度
			best_angle = None
			min_cost = max(cost,100)  # 初始最小成本设为角度为0时的成本
			for angle in angles[1:]:  # 从第二个角度开始计算，因为角度为0已经计算过了
				self.current_pose_ = original_pose.copy()
				self.current_pose_[2] += angle
				cost = self.AlignInLevelGaussNewton(source_, 0)
				print("angle: ", original_pose[2] + angle, "cost: ", cost)
				cv2.imshow('scan', cv2.addWeighted(self.scan_map[1], 0.5, self.field_[1], 0.5, 0))
				# cv2.waitKey(0)
				if cost > 0:
					if cost < min_cost:
						min_cost = cost
						best_angle = angle
						temp_pose=self.current_pose_.copy()
			# 使用最佳角度在每个层级上进行配准
			# if best_angle is not None:
			print("min_cost: ", min_cost)

			self.current_pose_ = temp_pose.copy()
			# self.current_pose_[2] += best_angle
			# self.AlignInLevelGaussNewton(source_, 0)  # 直接从第二层开始配准
			self.AlignInLevelGaussNewton(source_, 1)  # 直接从第二层开始配准
			self.AlignInLevelGaussNewton(source_, 2)  # 直接从第二层开始配准
		else:
			self.current_pose_ = original_pose.copy()
			self.AlignInLevelGaussNewton(source_, 2)


		if self.pose_last is not None:
			self.pose_last_last = self.pose_last.copy()
		self.pose_last=self.current_pose_.copy()