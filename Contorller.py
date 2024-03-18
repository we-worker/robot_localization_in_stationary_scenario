import serial
import time

ser = serial.Serial('dev/ttyUSB0', 115200)  # 请根据实际情况修改串口号和波特率

def robot_move(x_speed, y_speed, z_speed):
    instruction = f'{x_speed},{y_speed},{z_speed}\n'
    ser.write(instruction.encode())
    
def robot_stop():
    ser.write("stop\n".encode())

def arm_move(z_speed):
    instruction = f'{z_speed}\n'
    ser.write(instruction.encode())

# 示例：让机器人向x轴移动
robot_move(10, 0, 0)
time.sleep(1)
robot_stop()


ser.close()