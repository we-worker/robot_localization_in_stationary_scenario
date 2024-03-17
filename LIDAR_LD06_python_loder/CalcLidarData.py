import binascii
import math

class LidarData:
    def __init__(self,FSA,LSA,CS,Speed,TimeStamp,Confidence_i,Angle_i,Distance_i):
        self.FSA = FSA
        self.LSA = LSA
        self.CS = CS
        self.Speed = Speed
        self.TimeStamp = TimeStamp

        self.Confidence_i = Confidence_i
        self.Angle_i = Angle_i
        self.Distance_i = Distance_i



class Lidar_Datas:
    def __init__(self,ser_):
        self.ser=ser_
        self.loopFlag = True
        self.tmpString = ""
        self.flag2c = False
        self.b=""
        self.Datacount=0
        self.tempdistances=list()
        self.tempangles=list()
        # self.lock=Lock()

    def CalcLidarData(self,str):
        str = str.replace(' ','')

        Speed = int(str[2:4]+str[0:2],16)/100
        FSA = float(int(str[6:8]+str[4:6],16))/100
        LSA = float(int(str[-8:-6]+str[-10:-8],16))/100
        TimeStamp = int(str[-4:-2]+str[-6:-4],16)
        CS = int(str[-2:],16)

        Confidence_i = list()
        Angle_i = list()
        Distance_i = list()
        count = 0
        if(LSA-FSA > 0):
            angleStep = float(LSA-FSA)/(12)
        else:
            angleStep = float((LSA+360)-FSA)/(12)
        
        counter = 0
        circle = lambda deg : deg - 360 if deg >= 360 else deg
        for i in range(0,6*12,6): 
            Distance_i.append(int(str[8+i+2:8+i+4] + str[8+i:8+i+2],16)/100)
            Confidence_i.append(int(str[8+i+4:8+i+6],16))
            Angle_i.append(circle(angleStep*counter+FSA)*math.pi/180.0)
            counter += 1
        

        lidarData = LidarData(FSA,LSA,CS,Speed,TimeStamp,Confidence_i,Angle_i,Distance_i)
        return lidarData
    


    def read_and_parse_data(self,dist_angles):

        # angles = []
        # distances = []

        while True:
            if(self.Datacount % 40 == 39):
                self.Datacount=0

                dist_angles['ranges']=(self.tempdistances)
                dist_angles['angles']=(self.tempangles)
                dist_angles['is_new']=True

                self.tempdistances.clear()
                self.tempangles.clear()

            while self.loopFlag:
                self.b = self.ser.read()

                tmpInt = int.from_bytes(self.b, 'big')
                
                if (tmpInt == 0x54):
                    self.tmpString +=  self.b.hex()+" "
                    self.flag2c = True
                    continue
                
                elif(tmpInt == 0x2c and self.flag2c):
                    self.tmpString += self.b.hex()

                    if(not len(self.tmpString[0:-5].replace(' ','')) == 90 ):
                        self.tmpString = ""
                        self.loopFlag = False
                        self.flag2c = False
                        continue

                    lidarData = self.CalcLidarData(self.tmpString[0:-5])
                    self.tempangles.extend(lidarData.Angle_i)

                    self.tempdistances.extend(lidarData.Distance_i)
                        
                    self.tmpString = ""
                    self.loopFlag = False
                else:
                    self.tmpString += self.b.hex()+" "
                
                self.flag2c = False

            self.Datacount+=1
            # angles.clear()
            # distances.clear()
            self.loopFlag = True