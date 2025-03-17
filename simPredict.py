# -*- coding: utf-8 -*-
# @Time    : 2024/10/16 19:47
# @Author  : Mminmint
# @File    : simPredict.py
# @Software: PyCharm

import copy
import time
import traci

from toolFunction import startSUMO
from paramSetting import botInfoRef,genLCReactTimes,genSGReactTimes
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
        traci.route.add("M1-O1", ["M1", "O1"])
        routeRef = {"1": "M1-M6", "2": "M2-M6", "3": "M3-M6", "4": "M4-M6", "5": "M5-M6"}
        speedRef = {"1": 0, "2": 1, "3": 1, "4": 1, "5": 2}
    else:
        traci.route.add("M5-M9", ["M5", "M9"])
        traci.route.add("M6-M9", ["M6", "M9"])
        traci.route.add("M7-M9", ["M7", "M9"])
        traci.route.add("M8-M9", ["M8", "M9"])
        traci.route.add("I1-M9", ["I1", "M9"])
        routeRef = {"5": "M5-M9", "6": "M6-M9", "7": "M7-M9", "8": "M8-M9", "1": "I1-M9"}
        speedRef = {"5":2,"6":3,"7":3,"8":3}

    typeRef = {0: "HV", 1: "CV", 2: "CAV"}

    for vehInfo in allVehs:
        if vehInfo[-1]:
            routeID = "M1-O1"
        else:
            routeID = routeRef[vehInfo[2][-3]]
        if 'M' in vehInfo[2]:
            departSpeed = min(vehInfo[4],speedLimits[speedRef[vehInfo[2][-3]]])
        else:
            departSpeed = vehInfo[4]
        traci.vehicle.add(vehID=vehInfo[0], routeID=routeID, typeID=typeRef[vehInfo[1]], depart='now',departLane=int(vehInfo[2][-1]), departPos=vehInfo[-2],departSpeed=departSpeed)
        if vehInfo[5] is not None:
            traci.vehicle.setLaneChangeMode(vehInfo[0], vehInfo[5])


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


def gainLCTimes(suggestLC):
    LCreactTimes = genLCReactTimes(len(suggestLC))
    vehs = list(suggestLC.keys())
    vehLCTimes = list(zip(vehs, LCreactTimes))

    maxTime = max(LCreactTimes)  # 找到最大换道时间
    prepareLC = [[] for _ in range(maxTime + 1)]  # 初始化嵌套列表

    # 将车辆ID分配到对应的时间槽
    for veh, LCTime in vehLCTimes:
        prepareLC[LCTime].append(veh)

    return prepareLC


'''
CAV执行变速建议，要筛选出剩下的CV
suggestSG: {"cv.1":30,"cv.3":20...}
'''
def simCavSGExecute(suggestSG:Dict) -> Dict:
    suggestSGCopy = copy.deepcopy(suggestSG)

    for vehID,targetSpeed in suggestSG.items():
        if "cav" in vehID:
            targetSpeed = max(0, targetSpeed)
            traci.vehicle.slowDown(vehID, targetSpeed, 5)
            del suggestSGCopy[vehID]

    return suggestSGCopy


def gainSGTimes(suggestSG):
    SGreactTimes = genSGReactTimes(len(suggestSG))
    vehs = list(suggestSG.keys())
    vehSGTimes = list(zip(vehs, SGreactTimes))

    maxTime = max(SGreactTimes)  # 找到最大换道时间
    prepareSG = [[] for _ in range(maxTime + 1)]  # 初始化嵌套列表

    # 将车辆ID分配到对应的时间槽
    for veh, SGTime in vehSGTimes:
        prepareSG[SGTime].append(veh)

    return prepareSG


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
    # 这里的1只可能是O1
    posRef = {"1":425,"2":425,"3":670,"4":870,"5":925,
              "6":1420,"7":1670,"8":1870,"9":1970}

    for vehInfo in allVehs:
        vehID = vehInfo[0]
        originPos = vehInfo[3]

        if 'I' in vehInfo[2]:
            originPos = 1320

        if vehID in detectOut.keys():
            dist = endPos
            allDist += (dist - originPos) / (detectOut[vehID] + 1)
            count += 1
        else:
            if vehID in curVehs:
                curLane = traci.vehicle.getLaneID(vehID)
                dist = traci.vehicle.getLanePosition(vehID)
                if 'I' in curLane:
                    dist += 1320
                else:
                    dist += posRef[curLane[-3]]
                allDist += (dist - originPos) / horizon
                count += 1

    return allDist/count


'''
子仿真多线程执行函数，最终输出车辆行驶的总距离
suggestCvLC:{'cv.1':'Input.1_2','cv.16':'Input.2_2'}
suggestCavLC:{'cav.1':'Input.1_2','cav.16':'Input.2_2'}
'''
def simExecute(cfgFileTag,allVehs,suggestLC,suggestSG,simID,queue,speedLimits):
    result = 0
    detectOut = {}
    edgeList, cfgFile, detectors, botPos, endPos = botInfoRef(cfgFileTag)
    prepareCvLC, prepareCvSG = [],[]

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
                    if suggestCvLC:
                        prepareCvLC = gainLCTimes(suggestCvLC)
                        if prepareCvLC[0]:
                            for vehId in prepareCvLC[0]:
                                traci.vehicle.changeLane(vehId, int(suggestCvLC[vehId][-1]), 3)
                        prepareCvLC.pop(0)

                if suggestSG:
                    suggestCvSG = simCavSGExecute(suggestSG)
                    if suggestCvSG:
                        prepareCvSG = gainSGTimes(suggestCvSG)
                        if prepareCvSG[0]:
                            for vehId in prepareCvSG[0]:
                                targetSpeed = max(0, suggestCvSG[vehId])
                                traci.vehicle.slowDown(vehId, targetSpeed, 5)
                        prepareCvSG.pop(0)
            else:
                if prepareCvLC:
                    if prepareCvLC[0]:
                        for vehId in prepareCvLC[0]:
                            traci.vehicle.changeLane(vehId, int(suggestCvLC[vehId][-1]), 3)
                    prepareCvLC.pop(0)
                if prepareCvSG:
                    if prepareCvSG[0]:
                        for vehId in prepareCvSG[0]:
                            targetSpeed = max(0, suggestCvSG[vehId])
                            traci.vehicle.slowDown(vehId, targetSpeed, 5)
                    prepareCvSG.pop(0)

            step += 1
            if step == totalStep:
                result = avgSpeed(allVehs,totalStep,detectOut,endPos)
                break

            time.sleep(0.03)  # 在此等待以防止问题

    except Exception as e:
        print(step,suggestLC,suggestSG,prepareCvLC,prepareCvSG)
        print(f"Error: {e}")

    finally:
        traci.close()
        # 将结果放入队列
        queue.put(result)