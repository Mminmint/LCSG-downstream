# -*- coding: utf-8 -*-
# @Time    : 2024/10/14 15:51
# @Author  : Mminmint
# @File    : vehicles.py
# @Software: PyCharm
import copy
import random
import traci
from vehicle import Vehicle
from collections import defaultdict
from paramSetting import genLCReactTimes,genSGReactTimes
from typing import Tuple


class Vehicles:

    def __init__(self,botPos):
        self.botPos = botPos
        self.vehs = {}
        self.lastVehs = {}
        self.prepareLC = [{},{},{},{},{}]      # todo:敏感性参数
        self.prepareSG = [{},{},{},{},{}]


    '''
    【main】
    遍历车辆，初始化vehs中的车辆信息
    获得optVehs
    ps: lastVehs和vehs引用的是一个对象，属性值改了会一起变
    '''
    def initVehs(self,step:int,curVehs):
        self.step = step
        self.optVehs = {}

        for vehId in curVehs:
            # 初始化/更新车辆信息，添加至veh字典中
            if vehId not in self.lastVehs.keys():
                veh = self.addVeh(vehId)
            else:
                veh = self.lastVehs[vehId]
                veh.gainInfo()
                veh.updateLCInfo(self.step)
                self.vehs[vehId] = veh

            # 对车辆进行分类
            if veh.type:
                if "M" in veh.lane:
                    self.addOptVeh(veh)


    '''
    通过vehId新建veh类，初始化veh类属性
    将新出现车辆加入vehs中并返回当前车辆
    '''
    def addVeh(self,vehId:str) -> Vehicle:
        veh = Vehicle(vehId)
        veh.gainInfo()
        veh.initLCInfo()
        self.vehs[vehId] = veh

        return veh


    '''
    判断当前车辆是否在控制范围内
    在optVehs中加入
    '''
    def addOptVeh(self,veh:Vehicle):
        if self.botPos-700 < veh.position < self.botPos-100:
            if 'O1' in veh.vehId:
                pass
            else:
                self.optVehs[veh.vehId] = veh


    '''
    【main】
    将满足频率、安全的可优化车辆依据车道给出
    readyLC: {0:["cv_0","cav_3"],1:["cv_1","cv_2","cav_2"]}
    readySG: ["cv_0","cav_3"]
    '''
    def readyOptByLane(self):
        readyLC,readySG = defaultdict(list),list()
        readySGRef = {}
        readyLCRef = {}
        LCCount = 0

        for vehId,veh in self.optVehs.items():
            if veh.LCFrequency(self.step):
                if not veh.laneIndex:       # laneIndex为0
                    readyLCTag = veh.LCSafetyLeft(self.vehs)
                elif veh.laneIndex == 2:
                    readyLCTag = veh.LCSafetyRight(self.vehs)
                else:
                    if "M6" in veh.lane:
                        readyLCTag = veh.LCSafetyLeft(self.vehs) and veh.LCSafetyRight(self.vehs)
                    else:
                        readyLCTag = veh.LCSafetyRight(self.vehs)

                if readyLCTag:
                    readyLC[veh.laneIndex].append(vehId)
                    readyLCRef[vehId] = veh.lane
                    LCCount += 1

            if veh.SGFrequency(self.step) and veh.SGBound():
                readySG.append(vehId)
                readySGRef[vehId] = veh.speed
        SGCount = len(readySG)

        return readyLC,LCCount,readyLCRef,readySG,SGCount,readySGRef


    '''
    为当前时刻换道引导建议初始化车辆参数
    体现不确定参数：换道反应时间
    '''
    def initLCs(self,suggestLC):
        for vehID,lane in suggestLC.items():
            veh = self.vehs[vehID]
            veh.setLCInfo(self.step)
            # 若引导车辆为CV
            if veh.type == 1:
                reactTime = genLCReactTimes(1)
                index = reactTime[0]
                # 如果索引超出 prepareLC 的长度，则扩展 prepareLC
                if index >= len(self.prepareLC):
                    self.prepareLC.extend([{} for _ in range(index - len(self.prepareLC) + 1)])
                self.prepareLC[index][vehID] = lane
            else:
                self.prepareLC[0][vehID] = lane


    '''
    为先前收到换道引导指令的车辆执行建议
    '''
    def executeLCs(self):
        nowLC = self.prepareLC[0]

        for vehID,targetLane in nowLC.items():
            try:
                traci.vehicle.changeLane(vehID, int(targetLane[-1]), 3)
            except traci.TraCIException as e:
                print(f"Error traci for vehicle {vehID} while LC: {e}")
            except Exception as e:
                print(f"Unexpected error for vehicle {vehID} while LC: {e}")


        self.prepareLC.pop(0)
        self.prepareLC.append({})


    '''
    为当前时刻速度引导建议初始化车辆参数，依据反应时间放入待执行区域
    体现不确定参数：变速反应时间
    prepareSG[0] = {”cv.1”:targetSpeed}
    '''
    def initSGs(self, suggestSG):
        for vehID, targetSpeed in suggestSG.items():
            veh = self.vehs[vehID]
            veh.setSGInfo(self.step)
            # 若引导车辆为CV
            if veh.type == 1:
                reactTime = genSGReactTimes(1)
                index = reactTime[0]
                if index >= len(self.prepareSG):
                    self.prepareSG.extend([{} for _ in range(index - len(self.prepareSG) + 1)])
                self.prepareSG[index][vehID] = targetSpeed
            # 若引导车辆为CAV
            else:
                self.prepareSG[0][vehID] = targetSpeed


    '''
    为先前收到速度引导指令的车辆执行建议
    体现不确定参数：执行偏差
    '''
    def executeSGs(self, executeBias):
        nowSG = self.prepareSG[0]

        if nowSG:
            for vehID, targetSpeed in nowSG.items():
                try:
                    # CAV无延迟精准执行，直接set
                    if "cav" in vehID:
                        targetSpeed = max(0,targetSpeed)
                        traci.vehicle.slowDown(vehID,targetSpeed,5)
                    # CV要记录目标加速度，和剩余执行时间
                    else:
                        curSpeed = self.vehs[vehID].speed
                        randomRatio = random.uniform(executeBias, 2-executeBias)
                        realTargetSpeed = round(curSpeed + (targetSpeed - curSpeed)*randomRatio,3)
                        realTargetSpeed = max(0,realTargetSpeed)
                        traci.vehicle.slowDown(vehID,realTargetSpeed,5)
                        # print(vehID,targetSpeed,realTargetSpeed)
                except traci.TraCIException as e:
                    print(f"Error traci for vehicle {vehID} while SG: {e}")
                except Exception as e:
                    print(f"Unexpected error for vehicle {vehID} while SG: {e}")

        self.prepareSG.pop(0)
        self.prepareSG.append({})


    def deinit(self):
        self.lastVehs = self.vehs
        self.vehs = {}


    def organizeInfo(self):
        orgVehsInfo = []

        for vehId,veh in self.vehs.items():
            type = veh.type
            lane = veh.lane
            position = veh.position
            speed = veh.speed
            LCModel = veh.LCModel
            rePosition = veh.rePosition
            ramp = False
            if "M1_O1" in vehId:
                ramp = True
            orgVehsInfo.append((vehId,type,lane,position,speed,LCModel,rePosition,ramp))

        return orgVehsInfo

