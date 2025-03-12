# -*- coding: utf-8 -*-
# @Time    : 2024/10/14 22:04
# @Author  : Mminmint
# @File    : main.py
# @Software: PyCharm

from __future__ import absolute_import
from __future__ import print_function

import time

import traci
import sys
import math

import toolFunction
from typing import List
from vehicles import Vehicles
from optimizer import Optimizer

'''
将可换道的车辆字典转换为列表形式，并给出车道区分
readyLCDict: {0: ["cv_0", "cav_3"], 1: ["cv_1", "cv_2", "cav_2"]}
LCBound: [2,5]
readyLC: ["cv_0", "cav_3","cv_1", "cv_2", "cav_2"]
'''
def lineBound(readyLCDict):
    LCBound = list()
    LCBound.append(0)
    readyLC = []

    for i in range(3):
        if i in readyLCDict.keys():
            bound = LCBound[-1] + len(readyLCDict[i])
            LCBound.append(bound)
            readyLC.extend(readyLCDict[i])
        else:
            LCBound.append(LCBound[-1])

    LCBound.pop(0)
    return LCBound, readyLC


def setVSL(avgDensity, edges, prevSpeed, prevSpeedLimit):
    jamDensity = 100  # 最大密度 (veh/km)
    rhoCrit = jamDensity * 0.25  # 临界密度取最大密度的40%（可调整）
    vFree = 80.0  # 自由流速度 (km/h)
    tau = 0.03  # 驾驶员反应时间 (h)
    kappa = 50  # METANET敏感度参数
    T = 5.0 / 60  # 控制周期 (h)（5分钟）

    # 约束密度值在合理范围内
    avgDensity = min(avgDensity, jamDensity)  # 密度不超过最大密度

    # METANET速度更新方程（简化版）
    desiredSpeed = vFree * math.exp(-1 / kappa * (avgDensity / rhoCrit) ** 2)
    new_speed = prevSpeed + (T / tau) * (desiredSpeed - prevSpeed)

    # 约束速度值在合理范围内
    newSpeed = max(0, min(new_speed, vFree))  # 速度范围0~v_free km/h

    # 限速规则约束（避免剧烈波动）
    speedLimit = max(40, newSpeed)  # 限速范围40~80 km/h
    speedLimit = round(speedLimit/5) * 5

    # 约束限速变化幅度（不超过10 km/h）
    prevSpeedLimitKmh = prevSpeedLimit * 3.6  # 转换为km/h
    speedLimit = max(prevSpeedLimitKmh - 10, min(speedLimit, prevSpeedLimitKmh + 10))

    for edge in edges:
        # 设置限速（转换为m/s）
        traci.edge.setMaxSpeed(edge, speedLimit / 3.6)

    return speedLimit / 3.6, newSpeed


def callOpt(vehicles_:Vehicles,optimizer_:Optimizer,prevSpeedLimits:List):
    # 依据频率、安全约束给出可以接受变道/变速引导的车辆
    # readySG: ["cv_0","cav_3"]
    # readyLCRef: {"cv_0":"Input.3_0","cv_1":"Input.2_1"}
    readyLCDict, readyLCCount, readyLCRef, readySG, readySGCount, readySGRef = vehicles_.readyOptByLane()

    # 有可建议换道车辆，没有可建议变速车辆
    if readyLCCount and not readySGCount:
        # 将readyLCDict进行转化
        # readyLCDict: {0: ["cv_0", "cav_3"], 1: ["cv_1", "cv_2", "cav_2"]}
        # LCBound: [2,5]
        # readyLC: ["cv_0", "cav_3","cv_1", "cv_2", "cav_2"]
        LCBound, readyLC = lineBound(readyLCDict)
        # 整理车辆信息，便于多线程调用
        orgVehsInfo = vehicles_.organizeInfo()

        LCInfo = {"readyLC": readyLC, "readyLCRef": readyLCRef, "LCBound": LCBound}
        # 算法优化结果
        suggestLC, suggestSG = optimizer_.optimize(orgVehsInfo=orgVehsInfo, LCInfo=LCInfo)
        print("onlySuggestLC:", suggestLC)

    # 没有可建议换道车辆，有可建议变速车辆
    elif not readyLCCount and readySGCount:
        # 整理车辆信息，便于多线程调用
        orgVehsInfo = vehicles_.organizeInfo()

        SGInfo = {"readySG": readySG, "readySGRef": readySGRef, "speedLimits": prevSpeedLimits}
        # 算法优化结果
        suggestLC, suggestSG = optimizer_.optimize(orgVehsInfo=orgVehsInfo, SGInfo=SGInfo)
        print("onlySuggestSG:", suggestSG)

    # 有可建议换道车辆，有可建议变速车辆
    elif readyLCCount and readySGCount:
        LCBound, readyLC = lineBound(readyLCDict)
        # 整理车辆信息，便于多线程调用
        orgVehsInfo = vehicles_.organizeInfo()

        LCInfo = {"readyLC": readyLC, "readyLCRef": readyLCRef, "LCBound": LCBound}
        SGInfo = {"readySG": readySG, "readySGRef": readySGRef, "speedLimits": prevSpeedLimits}
        # 算法优化结果
        suggestLC, suggestSG = optimizer_.optimize(orgVehsInfo=orgVehsInfo, LCInfo=LCInfo, SGInfo=SGInfo)
        print("suggestLC:", suggestLC)
        print("suggestSG:", suggestSG)

    # 没有可优化对象
    else:
        suggestLC, suggestSG = {}, {}

    return suggestLC,suggestSG


def run():
    sumoCmd = toolFunction.startSUMO(True, "MainFile/MAGIC.sumocfg")
    traci.start(sumoCmd, label="Main")  # 打开仿真建立连接

    step = 0
    edgeList = [('M1',), ('M2','M3','M4'), ('M5',), ('M6','M7','M8')]
    edgeRatio = [425*2,245*2+200+55*2,495*2,250*3+200*2+100*3]
    botPos1,botPos2 = 670,1670       # todo:瓶颈点位置

    count = [0, 0, 0, 0]
    prevSpeeds = [80.0] * 4  # 初始化各路段速度（km/h）
    prevDensities = [0.0] * 4  # 初始化各路段密度（veh/km）
    prevSpeedLimits = [80.0 / 3.6] * 4  # 初始化各路段限速值（m/s）

    vehicles1 = Vehicles(botPos1)
    vehicles2 = Vehicles(botPos2)
    optimizer1 = Optimizer(cfgFileTag=1,originPopNum=20,popNum=6,iterTimes=10,
                           sameBestTimes=3,crossParam=0.6,mutationParam=0.1)
    optimizer2 = Optimizer(cfgFileTag=2,originPopNum=20,popNum=6,iterTimes=10,
                           sameBestTimes=3,crossParam=0.6,mutationParam=0.1)

    start = time.time()

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()

        '''进行可变信息板限速控制'''
        if (step >= 240 and step < 300) or (step >= 540 and step < 600) \
                or (step >= 840 and step < 900) or (step >= 1140 and step < 1200):
            for i in range(len(edgeList)):
                edges = edgeList[i]
                for edge in edges:
                    count[i] += traci.edge.getLastStepVehicleNumber(edge)

        if step == 300 or step == 600 or step == 900 or step == 1200:
            for i in range(len(edgeList)):
                # 计算平均密度
                avgDensity = (count[i] * 1000) / (60 * edgeRatio[i])
                # 调用METANET模型更新限速
                speedLimit, newSpeed = setVSL(avgDensity, edgeList[i], prevSpeeds[i], prevSpeedLimits[i])

                # 保存状态供下一周期使用
                prevSpeeds[i] = newSpeed
                prevDensities[i] = avgDensity
                prevSpeedLimits[i] = speedLimit

            count = [0, 0, 0, 0]  # 重置计数器

        '''模型优化'''
        if step >= 600:
            curVehs1 = traci.edge.getLastStepVehicleIDs('M1') \
                       + traci.edge.getLastStepVehicleIDs('M2') \
                       + traci.edge.getLastStepVehicleIDs('M3')\
                       + traci.edge.getLastStepVehicleIDs('M4')\
                       + traci.edge.getLastStepVehicleIDs('M5')
            curVehs2 = traci.edge.getLastStepVehicleIDs('M5') \
                       + traci.edge.getLastStepVehicleIDs('M6')\
                       + traci.edge.getLastStepVehicleIDs('M7')\
                       + traci.edge.getLastStepVehicleIDs('M8')\
                       + traci.edge.getLastStepVehicleIDs('I1')

            # 初始化/更新vehs中的车辆信息，更新optVehs
            vehicles1.initVehs(step,curVehs1)
            vehicles2.initVehs(step,curVehs2)

            speedLimits1 = prevSpeedLimits[:3]
            speedLimits2 = prevSpeedLimits[3:]

            suggestLC1,suggestSG1 = callOpt(vehicles1,optimizer1,speedLimits1)
            suggestLC2, suggestSG2 = callOpt(vehicles2, optimizer2, speedLimits2)

            # 主仿真执行之前发送的建议
            vehicles1.executeLCs()
            vehicles2.executeLCs()
            vehicles1.executeSGs(executeBias=0.8)
            vehicles2.executeSGs(executeBias=0.8)

            # 主仿真发送当前优化的建议（待执行）
            if suggestLC1:
                vehicles1.initLCs(suggestLC1)
            if suggestSG1:
                vehicles1.initSGs(suggestSG1)
            if suggestLC2:
                vehicles2.initLCs(suggestLC2)
            if suggestSG2:
                vehicles2.initSGs(suggestSG2)

            # 更新vehs到lastVehs
            vehicles1.deinit()
            vehicles2.deinit()

        '''操作结束，准备进入下一个步长'''
        step += 1

        if step == 1500:
            end = time.time()
            print("costTime:", end - start)
            break

    traci.close()  # 关闭连接，还需在gui中点击退出并关闭gui
    sys.stdout.flush()


if __name__ == "__main__":
    run()