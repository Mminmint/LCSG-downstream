# -*- coding: utf-8 -*-
# @Time    : 2024/10/14 13:55
# @Author  : Mminmint
# @File    : vehicle.py
# @Software: PyCharm

import traci
from toolFunction import trimTuple


class Vehicle:

    def __init__(self,vehId):
        # 基本信息
        self.vehId = vehId          # str: "cv_1"
        self.laneIndex = None            # int: 0
        self.LCModel = None
        self.lastSSGTime = -1


    '''
    获取/更新车辆基本信息
    车辆类型、位置、速度、加速度
    Application: allVehs
    '''
    def gainInfo(self):
        # 车辆类型
        if self.vehId[:2] == 'hv':
            self.type = 0
        elif self.vehId[:2] == 'cv':
            self.type = 1
        else:
            self.type = 2

        # 车辆相对位置、绝对位置（在2km范围内）、速度、加速度
        self.position = self.convertPos()       # float: 50.236(m)
        self.speed = traci.vehicle.getSpeed(self.vehId)                 # float: 3.28(m/s)
        self.acceleration = traci.vehicle.getAcceleration(self.vehId)   # float: 0.28(m/s2)


    def convertPos(self):
        lane = traci.vehicle.getLaneID(self.vehId)
        position = traci.vehicle.getLanePosition(self.vehId)
        self.rePosition = position

        if "Input" in lane:
            tag = lane[-3]
            if tag == "1":
                position += 500
            elif tag == "2":
                position += 1000
            elif tag == "3":
                position += 1500
        return position


    '''
    车辆刚进入车道时获取车道信息并初始化换道信息
    对于CV和CAV，初始化换道优化参数
    Application: allVehs
    '''
    def initLCInfo(self):
        self.lane = traci.vehicle.getLaneID(self.vehId)
        self.laneIndex = int(self.lane[-1])     # int: 0

        if self.type:
            # 换道建议执行信息
            self.lastLCTime = -1        # int
            self.lastSLCTime = -1       # int
            self.totalLCTimes = 0


    '''
    针对优化车辆更新车道信息以及换道时刻
    Application: optVehs
    '''
    def updateLCInfo(self,step:int):
        curLane = traci.vehicle.getLaneID(self.vehId)
        if self.type and int(curLane[-1]) != int(self.lane[-1]):
            self.lastLCTime = step      # int
            self.totalLCTimes += 1
        self.lane = curLane
        self.laneIndex = int(self.lane[-1])


    '''
    为符合条件的HV执行静态晚合流控制
    Application: HVs
    '''
    def staticLateMerge(self):
        if 1450 < self.position < 1550:
            traci.vehicle.setLaneChangeMode(self.vehId, 0b010101000101)
            self.LCModel = 0b010101000101


    '''
    判断车辆向左换道的安全可行性
    获取左侧车道前后车信息判断间隙
    Application: optVehs
    '''
    def LCSafetyLeft(self,vehs):
        # traci给出的相对距离会减去miniGap(2.5m)
        vehIdLL, distLL = trimTuple(traci.vehicle.getNeighbors(self.vehId, 0b00000010))
        vehIdLF, distLF = trimTuple(traci.vehicle.getNeighbors(self.vehId, 0b00000000))

        self.vehLL = vehs.get(vehIdLL, None) if vehIdLL is not None else None
        self.vehLF = vehs.get(vehIdLF, None) if vehIdLF is not None else None
        distLL = distLL if distLL is not None else 300
        distLF = distLF if distLF is not None else 300
        speedLF = self.vehLF.speed if self.vehLF else 0     # 获取目标车道后车信息
        speedLL = self.vehLL.speed if self.vehLL else 25    # 获取目标车道前车信息

        t_ch = 3        # 变道时间
        t_act = 1       # 车辆感知到前车减速的反应时间
        t_avg = 3       # todo:敏感度参数
        a_min = 4.5    # 最大减速度（绝对值

        # 如果不满足约束，则返0
        constrain1 = distLL + speedLL*(t_ch+t_avg) + (1/(2*a_min))*(speedLL*speedLL-self.speed*self.speed)\
                     - self.speed*(t_ch+t_act+t_avg)
        if constrain1 < 0:
            return 0

        constrain2 = distLF + self.speed*(t_ch+t_avg) + (1/(2*a_min))*(self.speed*self.speed-speedLF*speedLF) \
                     - speedLF*(t_ch+t_act+t_avg)

        return 0 if constrain2 < 0 else 1


    '''
    判断车辆向右换道的安全可行性
    获取右侧车道前后车信息判断间隙
    Application: optVehs
    '''
    def LCSafetyRight(self,vehs):
        # traci给出的相对距离会减去miniGap
        vehIdRL, distRL = trimTuple(traci.vehicle.getNeighbors(self.vehId, 0b00000011))
        vehIdRF, distRF = trimTuple(traci.vehicle.getNeighbors(self.vehId, 0b00000001))

        distRL = distRL if distRL is not None else 300
        distRF = distRF if distRF is not None else 300

        self.vehRL = vehs.get(vehIdRL,None) if vehIdRL is not None else None
        self.vehRF = vehs.get(vehIdRF,None) if vehIdRF is not None else None

        t_ch = 3        # 变道时间
        t_act = 1       # 车辆感知到前车减速的反应时间
        t_avg = 3       # todo:敏感度参数
        a_min = -4.5    # 最大减速度

        speedRF = self.vehRF.speed if self.vehRF else 0     # 获取目标车道后车信息
        speedRL = self.vehRL.speed if self.vehRL else 25    # 获取目标车道前车信息

        # 如果不满足约束，则返回0
        constrain1 = distRL + speedRL * (t_ch+t_avg) + (1 / (2 * a_min)) * (speedRL * speedRL - self.speed * self.speed) \
                     - self.speed * (t_ch + t_act + t_avg)
        if constrain1 < 0:
            return 0

        constrain2 = distRF + self.speed * (t_ch + t_avg) + (1 / (2 * a_min)) * (
                    self.speed * self.speed - speedRF * speedRF) \
                     - speedRF * (t_ch + t_act + t_avg)

        return 0 if constrain2 <0 else 1


    '''
    换道引导频率约束
    包括与上一次换道、建议换道时间的间隔，总换道次数
    Application: optVehs
    '''
    def LCFrequency(self,step:int):
        # 距离上一次换道的时间（包括自主变道）
        if step - self.lastLCTime <= 10:
            return 0
        # 距离上一次收到换道建议的时间
        if step - self.lastSLCTime <= 20:
            return 0
        # 总换道次数过多且在开放车道，禁用换道模型
        if self.totalLCTimes >= 5 and self.laneIndex:
            traci.vehicle.setLaneChangeMode(self.vehId, 256)
            self.LCModel = 256
            return 0
        # 总换道次数过多且不在开放车道，采用CV自身換道模型
        if self.totalLCTimes >= 5 and not self.laneIndex:
            traci.vehicle.setLaneChangeMode(self.vehId,0b011000001001)
            self.LCModel = 0b011000001001
            return 0

        return 1


    '''
    设置换道引导参数
    当车辆在优化后接到换道引导后，需要改变参数记录
    '''
    def setLGInfo(self, step: int):
        self.lastSLCTime = step


    '''
    设置速度引导参数
    当车辆在优化后接到速度引导后，需要改变参数记录
    '''
    def setSGInfo(self, step: int):
        self.lastSSGTime = step


    '''
    车辆无论与前后车相对速度、相对距离关系如何，均可接受速度建议
    '''
    def SGSafety(self):
        pass


    '''
    速度引导频率约束
    包括与上一次速度建议的间隔
    Application: optVehs
    '''
    def SGFrequency(self,step:int):
        if step - self.lastSSGTime <= 20:
            return -1
        return 1