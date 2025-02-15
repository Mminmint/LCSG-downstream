# -*- coding: utf-8 -*-
# @Time    : 2024/10/14 22:04
# @Author  : Mminmint
# @File    : main.py
# @Software: PyCharm

from __future__ import absolute_import
from __future__ import print_function

import time

import traci
import os, sys
import optparse
import copy

import toolFunction
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


def setVSL(avgDensity,edge):
    jamDensity = 1000/(5+2.5)
    ratio = avgDensity/jamDensity

    if ratio >= 0.5:
        speedLimit = 40
    elif ratio >= 0.4:
        speedLimit = 45
    elif ratio >= 0.3:
        speedLimit = 50
    elif ratio >= 0.2:
        speedLimit = 55
    else:
        speedLimit = 60

    # 设置可变限速
    traci.edge.setMaxSpeed(edge, speedLimit/3.6)

    return speedLimit/3.6


def run():
    sumoCmd = toolFunction.startSUMO(True, "MainFile/MainFile.sumocfg")
    traci.start(sumoCmd, label="Main")  # 打开仿真建立连接

    step = 0
    edgeList = ['Input', 'Input.1', 'Input.2', 'Input.3']
    count = [0, 0, 0, 0]
    speedLimits = [16.67,16.67,16.67,16.67]

    # 遗传算法参数
    originPopNum = 20
    popNum = 6
    iterTimes = 10
    sameBestTimes = 3
    crossParam = 0.6
    mutationParam = 0.1

    vehicles_ = Vehicles()
    optimizer_ = Optimizer(originPopNum,popNum,iterTimes,sameBestTimes,crossParam,mutationParam)
    start = time.time()

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()

        # 进行可变信息板限速控制
        if (step >= 540 and step < 600) or (step >= 840 and step < 900) or (step >= 1140 and step < 1200):
            for i in range(len(edgeList)):
                edge = edgeList[i]
                lastVehNum = traci.edge.getLastStepVehicleNumber(edge)
                count[i] += lastVehNum
        elif step == 600 or step == 900 or step == 1200:
            for i in range(len(count)):
                avgDensity = count[i]*2.0/60      # 平均密度veh/km
                speedLimit = setVSL(avgDensity,edgeList[i])
                speedLimits[i] = speedLimit
            count = [0,0,0,0]

        # 模型优化
        if step >= 600:
            curVehs = traci.vehicle.getIDList()
            # 初始化/更新vehs中的车辆信息，更新optVehs
            vehicles_.initVehs(step,curVehs)
            # 依据频率、安全约束给出可以接受变道/变速引导的车辆
            # readySG: ["cv_0","cav_3"]
            # readyLCRef: {"cv_0":"Input.3_0","cv_1":"Input.2_1"}
            readyLCDict, readyLCCount, readyLCRef, readySG,readySGCount, readySGRef = vehicles_.readyOptByLane()

            # 有可建议换道车辆，没有可建议变速车辆
            if readyLCCount and not readySGCount:
                # 将readyLCDict进行转化
                # readyLCDict: {0: ["cv_0", "cav_3"], 1: ["cv_1", "cv_2", "cav_2"]}
                # LCBound: [2,5]
                # readyLC: ["cv_0", "cav_3","cv_1", "cv_2", "cav_2"]
                LCBound, readyLC = lineBound(readyLCDict)
                # 整理车辆信息，便于多线程调用
                orgVehsInfo = vehicles_.organizeInfo()

                LCInfo = {"readyLC":readyLC,"readyLCRef":readyLCRef,"LCBound":LCBound}
                # 算法优化结果
                suggestLC,suggestSG = optimizer_.optimize(orgVehsInfo=orgVehsInfo, LCInfo=LCInfo)

            # 没有可建议换道车辆，有可建议变速车辆
            elif not readyLCCount and readySGCount:
                # 整理车辆信息，便于多线程调用
                orgVehsInfo = vehicles_.organizeInfo()

                SGInfo = {"readySG":readySG,"readySGRef":readySGRef,"speedLimits":speedLimits}
                # 算法优化结果
                suggestLC,suggestSG = optimizer_.optimize(orgVehsInfo=orgVehsInfo, SGInfo=SGInfo)

            # 有可建议换道车辆，有可建议变速车辆
            elif readyLCCount and readySGCount:
                LCBound, readyLC = lineBound(readyLCDict)
                # 整理车辆信息，便于多线程调用
                orgVehsInfo = vehicles_.organizeInfo()

                LCInfo = {"readyLC":readyLC,"readyLCRef":readyLCRef,"LCBound":LCBound}
                SGInfo = {"readySG":readySG,"readySGRef":readySGRef,"speedLimits":speedLimits}
                # 算法优化结果
                suggestLC,suggestSG = optimizer_.optimize(orgVehsInfo=orgVehsInfo, LCInfo=LCInfo, SGInfo=SGInfo)

            # 没有可优化对象
            else:
                suggestLC,suggestSG = {},{}

            # 主仿真执行之前发送的建议
            vehicles_.executeLCs()
            vehicles_.executeSGs(executeBias=0.8)

            if suggestLC:
                # 主仿真发送当前优化的建议（待执行）
                vehicles_.initLCs(suggestLC,avgReactTime=3,reactTimeBias=0.8)
            if suggestSG:
                vehicles_.initSGs(suggestSG, avgReactTime=3, reactTimeBias=0.8)

            # 更新vehs到lastVehs
            vehicles_.deinit()

        # 操作结束，准备进入下一个步长
        step += 1

        if step == 1500:
            end = time.time()
            print("costTime:", end - start)
            break

    traci.close()  # 关闭连接，还需在gui中点击退出并关闭gui
    sys.stdout.flush()


if __name__ == "__main__":
    run()