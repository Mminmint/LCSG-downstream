# -*- coding: utf-8 -*-
# @Time    : 2024/10/16 19:47
# @Author  : Mminmint
# @File    : simPredict.py
# @Software: PyCharm

import copy
import time
import traci

from toolFunction import startSUMO
from paramSetting import botInfoRef
from typing import Dict


'''
向路网中添加车辆，构造初始状态
'''
def addSimVehs(cfgFileTag,allVehs,speedLimits):
    if cfgFileTag == 1:
        traci.route.add("M1-M6", ["M1", "M6"])
        traci.route.add("M2-M6", ["M2", "M6"])
        traci.route.add("M3-M6", ["M3", "M6"])
        traci.route.add("M4-M6", ["M4", "M6"])
        traci.route.add("M5-M6", ["M5", "M6"])
        routeRef = {"1": "M1-M6", "2": "M2-M6", "3": "M3-M6", "4": "M4-M6", "5": "M5-M6"}
        speedRef = {"1": 0, "2": 1, "3": 1, "4": 1, "5": 2}
    else:
        traci.route.add("M5-M9", ["M5", "M9"])
        traci.route.add("M6-M9", ["M6", "M9"])
        traci.route.add("M7-M9", ["M7", "M9"])
        traci.route.add("M8-M9", ["M8", "M9"])
        routeRef = {"5": "M5-M9", "6": "M6-M9", "7": "M7-M9", "8": "M8-M9"}
        speedRef = {"5":2,"6":3,"7":3,"8":3}

    typeRef = {0: "HV", 1: "CV", 2: "CAV"}

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
            if not int(lane[-1]):                         # int(laneID[-1]):1
                traci.vehicle.setLaneChangeMode(vehID, 256)

'''
为符合条件的HV与驶出控制区后的optVehs执行默认换道模型
'''
def staticLateMerge(botPos):
    for vehID in traci.vehicle.getIDList():
        position = traci.vehicle.getLanePosition(vehID)
        if botPos-250 < position < botPos-150:
            traci.vehicle.setLaneChangeMode(vehID, 0b010101000101)


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
def avgSpeed(allVehs,horizon,detectOut,endPos) -> float:
    allDist,count = 0,0
    curVehs = traci.vehicle.getIDList()
    posRef = {"2":425,"3":925,"4":1420,"5":1970}

    for vehInfo in allVehs:
        vehID = vehInfo[0]
        originPos = vehInfo[3]

        if vehID in detectOut.keys():
            dist = endPos
            allDist += (dist - originPos) / (detectOut[vehID] + 1)
            count += 1
        else:
            if vehID in curVehs:
                curLane = traci.vehicle.getLaneID(vehID)
                dist = traci.vehicle.getLanePosition(vehID)
                dist += posRef[curLane[-3]]
                allDist += (dist - originPos) / horizon
                count += 1

    return allDist/count


'''
子仿真多线程执行函数，最终输出车辆行驶的总距离
suggestCvLC:{'cv.1':'Input.1_2','cv.16':'Input.2_2'}
suggestCavLC:{'cav.1':'Input.1_2','cav.16':'Input.2_2'}
'''
def simExecute(cfgFileTag,allVehs,suggestLC,suggestSG,simID,queue,speedLimits,LCReactTime,SGReactTime):
    result = 0
    detectOut = {}
    edgeList, cfgFile, detectors, botPos, endPos = botInfoRef(cfgFileTag)

    try:
        sumoCmd = startSUMO(False,cfgFile)
        traci.start(sumoCmd,label=f'Sub_{simID}')
        traci.switch(f'Sub_{simID}')

        totalStep = 60

        for step in range(totalStep):
            traci.simulationStep()

            for detector in detectors:
                outVehs = traci.lanearea.getLastStepVehicleIDs(detector)
                if outVehs:
                    for vehId in outVehs:
                        detectOut[vehId] = step

            if step == 0:
                # 初始化路网车辆
                addSimVehs(cfgFileTag,allVehs,speedLimits)
            elif step == 1:
                # 设置路段限速
                for i in range(len(edgeList)):
                    for edge in edgeList[i]:
                        traci.edge.setMaxSpeed(edge, speedLimits[i])
                # 对CAV进行换道引导
                if suggestLC:
                    suggestCvLC = simCavLCExecute(suggestLC)
            # 对CV进行换道引导
            if step == LCReactTime + 1:
                if suggestCvLC:
                    simCvLCExecute(suggestCvLC)
            # 到时间执行速度引导
            if step == SGReactTime + 1:
                if suggestSG:
                    simSGExecute(suggestSG)
            # 到时间判断若换道引导成功执行，禁用自身换道模型
            if step == LCReactTime + 10:
                if suggestLC:
                    banLCModel(suggestLC)
            # 为部分车辆实施静态晚合流
            staticLateMerge(botPos)

            step += 1
            if step == totalStep:
                result = avgSpeed(allVehs,totalStep,detectOut,endPos)
                break

            time.sleep(0.03)  # 在此等待以防止问题

    except Exception as e:
        print(f"Error: {e}")

    finally:
        traci.close()
        # 将结果放入队列
        queue.put(result)