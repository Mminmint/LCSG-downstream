# -*- coding: utf-8 -*-
# @Time    : 2024/11/25 19:40
# @Author  : Mminmint
# @File    : multiProcess.py
# @Software: PyCharm

import time

from simPredict import simExecute
import copy
import multiprocessing


# 多进程
def multiProcess(processNum,cfgFileTag,vehs,suggestLCs,suggestSGs,speedLimits,LCReactTimes,SGReactTimes):
    processes = []
    queue = multiprocessing.Queue()  # 创建一个队列用于收集结果

    # 使用多进程执行仿真
    for i in range(processNum):
        suggestLC = suggestLCs[i] if suggestLCs else []
        suggestSG = suggestSGs[i] if suggestSGs else []
        # simExecute(cfgFileTag,allVehs,suggestLC,suggestSG,simID,queue,speedLimits,LCReactTime,SGReactTime)
        p = multiprocessing.Process(target=simExecute, args=(cfgFileTag, vehs, suggestLC,suggestSG,i,queue,
                                                             speedLimits,LCReactTimes[i],SGReactTimes[i]))
        processes.append(p)
        p.start()
        # print(time.time())

    # 等待所有进程完成
    for p in processes:
        p.join()

    # 收集所有仿真的返回值
    results = [queue.get() for _ in range(processNum)]
    return results


def processExecute(processNum,cfgFileTag,vehs,suggestLCs,suggestSGs,speedLimits,LCReactTimes,SGReactTimes):
    results = multiProcess(processNum,cfgFileTag,vehs,suggestLCs,suggestSGs,speedLimits,LCReactTimes,SGReactTimes)
    return results