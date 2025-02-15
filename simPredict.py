# -*- coding: utf-8 -*-
# @Time    : 2024/10/16 19:47
# @Author  : Mminmint
# @File    : simPredict.py
# @Software: PyCharm
import copy
import random
import time

import traci
import os, sys
import optparse
from vehicle import Vehicle
from toolFunction import startSUMO
from collections import ChainMap
from typing import Dict


'''
向路网中添加车辆，构造初始状态
'''
def addSimVehs(allVehs,speedLimits):
    traci.route.add("expressway", ["Input", "Output"])
    traci.route.add("expressway1", ["Input.1", "Output"])
    traci.route.add("expressway2", ["Input.2", "Output"])
    traci.route.add("expressway3", ["Input.3", "Output"])

    typeRef = {0: "HV", 1: "CV", 2: "CAV"}
    routeRef = {"1":"expressway1","2":"expressway2","3":"expressway3","t":"expressway"}
    speedRef = {"1":1,"2":2,"3":3,"t":0}

    for vehInfo in allVehs:
        routeID = routeRef[vehInfo[2][-3]]
        departSpeed = min(vehInfo[4],speedLimits[speedRef[vehInfo[2][-3]]])
        traci.vehicle.add(vehID=vehInfo[0], routeID=routeID, typeID=typeRef[vehInfo[1]], depart='now',departLane=int(vehInfo[2][-1]), departPos=vehInfo[-1],departSpeed=departSpeed)
        if vehInfo[5] is not None:
            traci.vehicle.setLaneChangeMode(vehInfo[0], vehInfo[5])


'''
若车辆被建议换道成功，认为其接下来不会继续换道
禁用其换道模型
'''
def banLCModel(suggestLC):
    for vehID,lane in suggestLC.items():
        if traci.vehicle.getLaneID(vehID) == lane:    # laneID:Input.1_1
            if int(lane[-1]):                         # int(laneID[-1]):1
                traci.vehicle.setLaneChangeMode(vehID, 256)

'''
为符合条件的HV与驶出控制区后的optVehs执行默认换道模型
'''
def staticLateMerge():
    for vehID in traci.vehicle.getIDList():
        position = traci.vehicle.getLanePosition(vehID)
        if "hv" in vehID:
            if 1450 < position < 1550:
                traci.vehicle.setLaneChangeMode(vehID, 0b010101000101)
        else:
            if (1750 < position < 1850) and traci.vehicle.getLaneIndex(vehID):
                traci.vehicle.setLaneChangeMode(vehID, 0b011000001001)


'''
CAV执行换道建议，要筛选出剩下的CV
suggestLC: {"cv.1":"Input.2_0","cv.3":"Input.2_0"}
'''
def simCavLCExecute(suggestLC:Dict) -> Dict:
    suggestLCCopy = copy.deepcopy(suggestLC)

    for vehID,lane in suggestLC.items():
        if "cav" in vehID:
            traci.vehicle.changeLane(vehID,int(lane[-1]),3)
            del suggestLCCopy[vehID]

    return suggestLCCopy


'''
CV执行换道建议
suggestLC: {"cv.1":"Input.2_0","cv.3":"Input.2_0"}
'''
def simCvLCExecute(suggestLC:Dict) -> None:
    for vehID,lane in suggestLC.items():
        traci.vehicle.changeLane(vehID,int(lane[-1]),3)


'''
执行速度建议
suggestSG: {"cv.1":30,"cv.3":20...}
'''
def simSGExecute(suggestSG:Dict) -> None:
    for vehID,targetSpeed in suggestSG.items():
        targetSpeed = max(0,targetSpeed)
        traci.vehicle.slowDown(vehID,targetSpeed,5)


'''
所有车辆平均行驶速度
'''
def avgSpeed(allVehs,horizon,detectOut) -> float:
    allDist,count = 0,0

    for vehInfo in allVehs:
        vehID = vehInfo[0]
        originPos = vehInfo[3]

        # 若车辆还在路段内
        if vehID in traci.vehicle.getIDList():
            curLane = traci.vehicle.getLaneID(vehID)
            dist = traci.vehicle.getLanePosition(vehID)

            # 若车辆一开始在Input路段，后面驶入Output，position会突变
            if "Output" in curLane and "Input" in vehInfo[2]:
                dist += 2000
            else:
                if "Input" in curLane:
                    tag = curLane[-3]
                    if tag == "1":
                        dist += 500
                    elif tag == "2":
                        dist += 1000
                    elif tag == "3":
                        dist += 1500
            allDist += (dist-originPos)/horizon
            count += 1

        # 若车辆已经驶出路段
        else:
            if vehID not in detectOut.keys():
                continue
            if "Output" in vehInfo[2]:
                dist = 200
            else:
                dist = 2200
            allDist += (dist - originPos)/(detectOut[vehID]+1)
            count += 1

    return allDist/count


'''
子仿真多线程执行函数，最终输出车辆行驶的总距离
suggestCvLC:{'cv.1':'Input.1_2','cv.16':'Input.2_2'}
suggestCavLC:{'cav.1':'Input.1_2','cav.16':'Input.2_2'}
'''
def simExecute(allVehs,suggestLC,suggestSG,simID,queue,speedLimits):
    result = 0
    detectOut = {}
    edgeList = ['Input', 'Input.1', 'Input.2', 'Input.3']

    try:
        sumoCmd = startSUMO(False, "SubFile/SubTry.sumocfg")

        traci.start(sumoCmd,label=f'Sub_{simID}')
        traci.switch(f'Sub_{simID}')

        totalStep = 60
        avgLCReactTime = 3      # todo: 灵敏度分析参数
        avgSGReactTime = 3      # todo: 灵敏度分析参数

        for step in range(totalStep):
            traci.simulationStep()

            if traci.lanearea.getLastStepVehicleIDs("e1"):
                for vehId in traci.lanearea.getLastStepVehicleIDs("e1"):
                    detectOut[vehId] = step
                for vehId in traci.lanearea.getLastStepVehicleIDs("e2"):
                    detectOut[vehId] = step

            if step == 0:
                addSimVehs(allVehs,speedLimits)
            # 到时间执行换道引导
            elif step == 1:
                for i in range(4):
                    traci.edge.setMaxSpeed(edgeList[i], speedLimits[i])

                if suggestLC:
                    suggestCvLC = simCavLCExecute(suggestLC)

            if step == avgLCReactTime + 1:
                if suggestLC:
                    simCvLCExecute(suggestCvLC)
            # 到时间执行速度引导
            # todo: 灵敏度分析参数，要与Vehicle.setSGInfo中的SGRemainTime相同
            if step == avgSGReactTime + 1:
                if suggestSG:
                    simSGExecute(suggestSG)
            # 到时间判断若换道引导成功执行，禁用自身换道模型
            if step == avgLCReactTime + 10:
                if suggestLC:
                    banLCModel(suggestLC)
            # 为部分车辆实施静态晚合流
            staticLateMerge()

            step += 1
            if step == totalStep:
                result = avgSpeed(allVehs,totalStep,detectOut)
                break

            time.sleep(0.03)  # 在此等待以防止问题

    except Exception as e:
        print(allVehs)
        print(suggestSG)
        print(f"Error: {e}")

    finally:
        traci.close()
        # 将结果放入队列
        queue.put(result)
        # print(f"process {simId} end in {time.time()}")