import numpy as np
import utils
import cv2

# from ParticleFilter import *
from SingleBotLaser2D import *
from LikelihoodField import *
import time
import serial
from LIDAR_LD06_python_loder.CalcLidarData import * 
from multiprocessing import Process, Lock ,Manager
import copy
# from queue import LifoQueue




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



ser = serial.Serial(port='/dev/ttyUSB0',
                    baudrate=230400,
                    timeout=5.0,
                    bytesize=8,
                    parity='N',
                    stopbits=1)

# distances = list()
# distancesQueue=Queue()
lidar=Lidar_Datas(ser)
lock = Lock()
# 获取模拟的机器人的传感器数据





# 主函数
if __name__ == '__main__':
    # 初始化OpenCV窗口
    cv2.namedWindow('view', cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow('scan', cv2.WINDOW_AUTOSIZE)

    # 初始化2D环境
    # 定义机器人参数，包括传感器点数，起始角度，结束角度，最大距离，速度和角速度
    bot_param = [468,0, 360, 800.0, 6.0, 6.0]
    # 定义机器人的初始位置，包括x坐标，y坐标和角度
    bot_pos = np.array([40.0, 235, 0.0])
    # 创建一个2D激光机器人环境，输入参数为机器人位置，机器人参数和地图图片
    env = SingleBotLaser2Dgrid(bot_pos, bot_param, 'maps/map1.png')
    # 初始化GridMap，定义地图参数，包括占用概率，空闲概率，最大概率和最小概率
    map_param = [0.4, -0.4, 5.0, 0] 
    

    # lf = LikelihoodField()
    # lf.SetFieldImageFromMap(env.img_map)
    lf = LikelihoodField('maps/field',bot_pos)
    #  设置场图像,env.img_mapw为0-1的占用概率图像

    cv2.imshow('LikelihoodField0',lf.field_[0])
    # cv2.imshow('LikelihoodField1',lf.field_[1])
    # cv2.imshow('LikelihoodField2',lf.field_[2])

   
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
        p1 = Process(target=lidar.read_and_parse_data, args=(dist_angles,))
        p1.start()

        frame_count=0
        # 主循环
        while(1):

            if len(dist_angles['ranges'])>0 :

                    sensorDatas=copy.deepcopy(dist_angles)

            if  len(sensorDatas['ranges']) == 0:
                continue

            if len(sensorDatas['angles']) !=len(sensorDatas['ranges']):
                print("error",len(sensorDatas['angles']),len(sensorDatas['ranges']))
                continue

         # 绘制机器人在地图上的位置和传感器数据
            sensorDatas['ranges'] = [x *10 for x in sensorDatas['ranges']]
            img = Draw(env.img_map, 1, lf.current_pose_, sensorDatas, env.bot_param)
        
            cv2.circle(img,(int(lf.current_pose_[0]), int(lf.current_pose_[1])), int(3), (0,255,255), -1)
            cv2.imshow('view',img)


            start_time = time.time()
            lf.Align(sensorDatas)#测试结果20-60ms执行一次
            end_time = time.time()
            elapsed_time = end_time - start_time
            if elapsed_time>0.4:
                print("AlignGaussNewton took {:.2f} seconds to process.".format(elapsed_time))
            position_text = "Position: ({:.2f}, {:.2f}, {:.2f})".format(lf.current_pose_[0], lf.current_pose_[1], lf.current_pose_[2])
            # print(position_text)
            # cv2.imshow('scan',lf.scan_map[0])

            cv2.waitKey(10)

    cv2.destroyAllWindows()

