# -*- coding: utf-8 -*-
# @Time    : 2025/3/10 14:05
# @Author  : Mminmint
# @File    : parameterSample.py
# @Software: PyCharm

import numpy as np
import random
from scipy.stats import gamma


def botInfoRef(cfgFileTag):
    if cfgFileTag == 1:
        edgeList = [('M1'), ('M2','M3','M4'),('M5'),('M6')]
        cfgFile = "SubFile/SubTry1.sumocfg"
        detectors = ["e0", "e1"]
        botPos = 200
        endPos = 1000
    else:
        edgeList = [('M5'),('M6','M7','M8')]
        cfgFile = "SubFile/SubTry2.sumocfg"
        detectors = ["e0", "e1", "e2"]
        botPos = 1500
        endPos = 20000
    return edgeList,cfgFile,detectors,botPos,endPos


def genLCReactTimes(size=1):
    probabilityDict = {0: 0.019704433497536946, 1: 0.034482758620689655,
                     2: 0.029556650246305417, 3: 0.019704433497536946,
                     4: 0.07389162561576355, 5: 0.06896551724137931,
                     6: 0.1330049261083744, 7: 0.14285714285714285,
                     8: 0.08374384236453201, 9: 0.07881773399014778,
                     10: 0.059113300492610835, 11: 0.08374384236453201,
                     12: 0.06403940886699508, 13: 0.014778325123152709,
                     14: 0.019704433497536946, 15: 0.009852216748768473,
                     16: 0.029556650246305417, 17: 0.009852216748768473,
                     18: 0.014778325123152709, 19: 0.009852216748768473}
    times = list(probabilityDict.keys())
    probs = list(probabilityDict.values())
    return np.random.choice(times, size=size, p=probs)


def genSGReactTimes(size):
    k = 7.688
    theta = 0.396
    SGReactTimes = gamma.rvs(a=k, scale=theta, size=size)
    SGReactTimes = np.round(SGReactTimes).astype(int)
    return SGReactTimes


def genTestSLCs(suggestLCs):
    TestSLCs = []
    for SLC in suggestLCs:
        TestSLCs.extend([SLC] * 3)
    return TestSLCs


def genTestSSGs(suggestSGs,readySGRef):
    TestSSGs = []

    for suggestSG in suggestSGs:
        for _ in range(3):  # 每个字典随机化三次
            TestSSG = {}
            for vehId, targetSpeed in suggestSG.items():
                curSpeed = readySGRef.get(vehId, 0)  # 如果 key 不存在，默认速度为 0
                randomFactor = random.uniform(0.8,1.2)
                # 计算最终速度
                realSpeed = (targetSpeed - curSpeed) * randomFactor + curSpeed
                # 更新随机化后的字典
                TestSSG[vehId] = realSpeed
            # 将随机化后的字典添加到结果列表
            TestSSGs.append(TestSSG)

    return TestSSGs