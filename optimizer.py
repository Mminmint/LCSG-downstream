# -*- coding: utf-8 -*-
# @Time    : 2024/10/16 12:03
# @Author  : Mminmint
# @File    : optimizer.py
# @Software: PyCharm

import random
import traci
import numpy as np
# import matplotlib.pyplot as plt

from copy import deepcopy
from multiProcess import processExecute
from toolFunction import nearestFive
from operator import itemgetter
from typing import List,Dict
# from matplotlib import rcParams
#
# config = {
#     "font.family": 'serif',
#     "mathtext.fontset": 'stix',  # matplotlib渲染数学字体时使用的字体，和Times New Roman差别不大
#     "font.serif": ['SimSun'],  # 宋体
#     'axes.unicode_minus': False  # 处理负号，即-号
# }
# rcParams.update(config)


class Optimizer:
    def __init__(self,originPopNum,popNum,iterTimes,sameBestTimes,crossParam,mutationParam):
        self.originPopNum = originPopNum
        self.popNum = popNum
        self.iterTimes = iterTimes
        self.sameBestTimes = sameBestTimes
        self.crossParam = crossParam
        self.mutationParam = mutationParam
        self.bestLC = None
        self.bestSG = None


    '''初始化种群'''
    def initPopulation(self,LCTag,SGTag):
        initPop = []

        for _ in range(self.originPopNum):
            individual = {}
            if LCTag:
                # lane0: -1,1   lane1: 0,-1,2   lane2: 1,-1
                # 生成换道随机种子，不换道用-1表示
                newLC = []
                tmp = list(np.random.randint(0,2,self.LCBound[0]))
                newLC.extend(list(map(lambda x: -1 if x == 0 else x, tmp)))
                tmp = list(np.random.randint(0, 3, self.LCBound[1]-self.LCBound[0]))
                newLC.extend(list(map(lambda x: -1 if x == 1 else x, tmp)))
                tmp = list(np.random.randint(1, 3, self.LCBound[2]-self.LCBound[1]))
                newLC.extend(list(map(lambda x: -1 if x == 2 else x, tmp)))
                individual["LC"] = newLC

            if SGTag:
                # 生成变速随机种子
                newSG,absSG = [],[]
                refSG = [2.778,1.389,1.389,-1.389,-1.389,-1.389,-1.389,-2.778,-2.778,-2.778]

                # 考虑车辆可能同时换道的可能性
                if LCTag:
                    for vehId in self.readySG:
                        if vehId in self.readyLC:
                            index = self.readyLC.index(vehId)
                            # 被要求换道
                            if individual["LC"][index] != -1:
                                absSG.append(0)
                                newSG.append(0)
                                continue
                        if np.random.randint(0,2):
                            # 给五次机会，如果一直比限速大，就赋值为0
                            for i in range(5):
                                choice = np.random.choice(refSG)
                                targetSpeed = choice + self.readySGRef[vehId]
                                if targetSpeed <= self.curMaxSpeed and targetSpeed >= 0:
                                    absSG.append(choice)
                                    newSG.append(targetSpeed)
                                    break
                            else:
                                absSG.append(0)
                                newSG.append(0)
                        else:
                            absSG.append(0)
                            newSG.append(0)
                else:
                    for vehId in self.readySG:
                        if np.random.randint(0, 2):
                            # 给五次机会，如果一直比限速大，就赋值为0
                            for i in range(5):
                                choice = np.random.choice(refSG)
                                targetSpeed = choice + self.readySGRef[vehId]
                                if targetSpeed <= self.curMaxSpeed and targetSpeed >= 0:
                                    absSG.append(choice)
                                    newSG.append(targetSpeed)
                                    break
                            else:
                                absSG.append(0)
                                newSG.append(0)
                        else:
                            absSG.append(0)
                            newSG.append(0)

                individual["SG"] = newSG
                individual["absSG"] = absSG

            individual["fit"] = -1
            initPop.append(individual)

        return initPop


    '''选择种群中的最优个体'''
    def selectBest(self,popWithFit:List[Dict]) -> Dict:
        sortPop = sorted(popWithFit,key=itemgetter("fit"),reverse=True)     # 降序
        return sortPop[0]       # 返回最大值


    '''
    将便于变换的列表形式转换为真正需要换道的字典形式
    readyLC: ["cv_0","cav_3","cv_1","cv_2","cav_2"]
    pop["LC"]: [-1,1,-1,-1,2]
    
    suggestLC: {"cv.1": "Input.3_0", "cv.3": "Input.2_1"}
    '''
    def transReadyToSuggestLC(self,individual:Dict,readyLCRef:Dict):
        suggestLC = {}
        popLC = individual['LC']

        # 将需要变道的车辆加入suggest集合中
        for i in range(len(popLC)):
            if popLC[i] != -1:
                veh = self.readyLC[i]
                suggestLC[veh] = readyLCRef[veh][:-2]+'_'+str(popLC[i])

        return suggestLC

    '''
    将便于变换的列表形式转换为真正需要换道的字典形式
    readySG: ["cv_0","cav_3","cv_1","cv_2","cav_2"]
    pop["SG"]: [0,1.389,-1.389,0,0]

    suggestSG: {"cv.1": 30, "cv.3": 20}
    '''
    def transReadyToSuggestSG(self,individual:Dict):
        suggestSG = {}
        popSG = individual['SG']

        # 将进行变速引导的车辆加入suggest集合中
        for i in range(len(popSG)):
            if popSG[i] != 0:
                vehID = self.readySG[i]
                targetSpeed = popSG[i]
                if "cv" in vehID:
                    targetSpeed = max((nearestFive(targetSpeed*3.6))/3.6,0)
                suggestSG[vehID] = targetSpeed

        return suggestSG


    '''快速计算一个种群的适应度'''
    def quickFitness(self,pop:List[Dict],count,LCTag,SGTag) -> List[Dict]:
        suggestLCs,suggestSGs = [],[]

        if LCTag:
            for i in range(count):
                # 要进行readyLC和suggestLC的转换
                suggestLC = self.transReadyToSuggestLC(pop[i],self.readyLCRef)
                suggestLCs.append(suggestLC)
        if SGTag:
            for i in range(count):
                # 要进行readySG和suggestSG的转换
                suggestSG = self.transReadyToSuggestSG(pop[i])
                suggestSGs.append(suggestSG)

        # 有需要预测的适应度时
        results = processExecute(count,self.orgVehsInfo,suggestLCs,suggestSGs,self.speedLimits)

        # 和pop中的序号对上，赋值fit
        for i in range(count):
            pop[i]['fit'] = results[i]

        return pop


    '''用轮盘赌方式按照概率从上一代选择个体直至形成新的一代'''
    def selection(self,popWithFit:List[Dict]) -> List[Dict]:
        afterSelect = []
        sortPop = sorted(popWithFit,key=itemgetter("fit"), reverse=True)        # 从大到小排列

        minFit = sortPop[-1]['fit']
        sumFit = 0
        for individual in sortPop:
            individual['fitRef'] = individual['fit'] - minFit   # 归一化
            sumFit += individual['fitRef']

        for i in range(self.popNum):
            pointer = sumFit * np.random.uniform(0, 1)  # 随机产生一个[0,sum_fit]范围的数，即轮盘赌这局的指针
            curSum = 0
            for individual in sortPop:
                # 逐次累加从大到小排列的个体的适应度函数的值，直至超过指针，即选择它
                curSum += individual['fitRef']
                if curSum >= pointer:
                    afterSelect.append(sortPop[i])
                    break

        afterSelect = sorted(afterSelect,key=itemgetter('fit'), reverse=True)       # 从大到小排列选择的个体，方便进行交叉操作

        return afterSelect


    '''实现交叉操作'''
    def crossover(self,offSpring1,offSpring2,LCTag,SGTag):
        crossOff1, crossOff2 = {},{}

        if LCTag:
            # 交换换道区间
            pos1 = random.randrange(0, len(self.readyLC))
            pos2 = random.randrange(0, len(self.readyLC))
            if pos2 < pos1:
                pos1,pos2 = pos2,pos1

            crossOff1['LC'] = offSpring1['LC'][:pos1]+offSpring2['LC'][pos1:pos2]+offSpring1['LC'][pos2:]
            crossOff2['LC'] = offSpring2['LC'][:pos1]+offSpring1['LC'][pos1:pos2]+offSpring2['LC'][pos2:]

        if SGTag:
            # 交换变速区间
            pos1 = random.randrange(0, len(self.readySG))
            pos2 = random.randrange(0, len(self.readySG))
            if pos2 < pos1:
                pos1,pos2 = pos2,pos1
            crossOff1['SG'] = offSpring1['SG'][:pos1] + offSpring2['SG'][pos1:pos2] + offSpring1['SG'][pos2:]
            crossOff2['SG'] = offSpring2['SG'][:pos1] + offSpring1['SG'][pos1:pos2] + offSpring2['SG'][pos2:]
            crossOff1['absSG'] = offSpring1['absSG'][:pos1] + offSpring2['absSG'][pos1:pos2] + offSpring1['absSG'][pos2:]
            crossOff2['absSG'] = offSpring2['absSG'][:pos1] + offSpring1['absSG'][pos1:pos2] + offSpring2['absSG'][pos2:]

            if LCTag:
                # 交换之后，如果该车辆被建议换道了，SG要赋值为0
                for i in range(len(self.readySG)):
                    vehId = self.readySG[i]
                    if vehId in self.readyLC:
                        index = self.readyLC.index(vehId)
                        # 被要求换道
                        if crossOff1['LC'][index] != -1:
                            crossOff1['SG'][i] = 0
                            crossOff1['absSG'][i] = 0
                        if crossOff2['LC'][index] != -1:
                            crossOff2['SG'][i] = 0
                            crossOff2['absSG'][i] = 0

        crossOff1['fit'] = -1
        crossOff2['fit'] = -1

        return crossOff1,crossOff2


    '''实现变异操作'''
    def mutation(self,crossOff,LCTag,SGTag):
        if LCTag:
            # 选取变道变异点，依据可选择车道变异
            pos = random.randrange(0, len(self.readyLC))
            if self.LCBound[0] <= pos < self.LCBound[1]:
                choice = [-1,0,2]
                choice.remove(crossOff['LC'][pos])
                crossOff['LC'][pos] = random.choice(choice)
            else:
                crossOff['LC'][pos] = -crossOff['LC'][pos]

            if SGTag:
                # 若变异后被建议换道，检查变速建议，置为0
                if crossOff['LC'][pos] != -1:
                    if self.readyLC[pos] in self.readySG:
                        index = self.readySG.index(self.readyLC[pos])
                        crossOff['SG'][index] = 0
                        crossOff['absSG'][index] = 0

        if SGTag:
            # 选取合适的变速变异点
            while True:
                pos = random.randrange(0, len(self.readySG))
                if LCTag:
                    # 当选到的车辆在可变道集合中且被建议变道，重新选取
                    if self.readySG[pos] in self.readyLC and crossOff['LC'][self.readyLC.index(self.readySG[pos])] != -1:
                        continue
                # 否则结束循环，此值有效
                break

            # 选取合适的变速变异值
            choice = [2.778, 1.389, 1.389, -1.389, -1.389, -1.389, -1.389, -2.778, -2.778, -2.778]
            # 若先前的车速建议不为0
            if crossOff['SG'][pos]:
                for i in range(5):
                    value = random.choice([0, 1])
                    if value:
                        value = random.choice(choice)
                    if value != crossOff['absSG'][pos]:
                        targetSpeed = value + self.readySGRef[self.readySG[pos]]
                        if targetSpeed <= self.curMaxSpeed and targetSpeed >= 0:
                            crossOff['absSG'][pos] = value
                            crossOff['SG'][pos] = targetSpeed
                            break
            # 若先前车速建议为0
            else:
                value = random.choice(choice)
                for i in range(5):
                    targetSpeed = value + self.readySGRef[self.readySG[pos]]
                    if targetSpeed <= self.curMaxSpeed and targetSpeed >= 0:
                        crossOff['absSG'][pos] = value
                        crossOff['SG'][pos] = targetSpeed
                        break

        crossOff['fit'] = -1

        return crossOff


    # '''迭代图绘制'''
    # def iterPlot(self,allFits):
    #     plt.figure(figsize=(4, 2))
    #     plt.plot(allFits)
    #     plt.xlim([1, 20])
    #     plt.xticks(range(1, self.popNum, 2))
    #     plt.xlabel('迭代次数', fontsize=12)
    #     plt.ylabel('平均行驶距离/m', fontsize=12)
    #     plt.show()


    '''关联分析，判断驾驶引导建议涉及的车辆数'''
    def correlationChoose(self,individuals:list,LCTag,SGTag):
        res = []

        # 指标系数取值进一步确定
        # 没有换道
        if not LCTag:
            for i in range(3):
                index = 0.2*len(list(filter(lambda x: x != 0, individuals[i]['SG']))) \
                        + 0.1*sum(abs(x) for x in individuals[i]['absSG'])
                res.append(index)
        # 没有变速
        elif not SGTag:
            for i in range(3):
                index = len(list(filter(lambda x: x != -1, individuals[i]['LC'])))
                res.append(index)
        # 有换道与变速
        else:
            for i in range(3):
                index = len(list(filter(lambda x: x != -1, individuals[i]['LC']))) \
                        + 0.2*len(list(filter(lambda x: x != 0, individuals[i]['SG']))) \
                        + 0.1*sum(abs(x) for x in individuals[i]['absSG'])
                res.append(index)

        minValue = min(res)
        minIndex = [index for index, value in enumerate(res) if value == minValue]

        return individuals[minIndex[-1]]


    '''优化主函数——换道及变速'''
    def optimize(self,orgVehsInfo,LCInfo=None,SGInfo=None):
        # 参数初始化
        print('-------------start-------------')
        self.orgVehsInfo = orgVehsInfo

        LCTag = 1 if LCInfo is not None else 0
        SGTag = 1 if SGInfo is not None else 0

        if LCTag:
            self.readyLC = LCInfo["readyLC"]
            self.LCBound = LCInfo["LCBound"]
            self.readyLCRef = LCInfo["readyLCRef"]
        if SGTag:
            self.readySG = SGInfo["readySG"]
            self.readySGRef = SGInfo["readySGRef"]
            self.speedLimits = SGInfo["speedLimits"]
            self.curMaxSpeed = max(self.speedLimits)

        bestTimes = 1
        allFits,bestIndividuals = [],[]
        bestLC,bestSG = {},{}

        # 种群初始化
        # initPop: [{'LC': [-1,1,-1,-1,2,-1,1], 'SG': [0,1.389,-1.389,0,0], 'fit': -1},
        #           {'LC': [-1,1,-1,-1,2,-1,1], 'SG': [0,1.389,-1.389,0,0], 'fit': -1}]
        initPop = self.initPopulation(LCTag,SGTag)
        # 为初始化的种群计算fitness
        popWithFit = self.quickFitness(initPop,self.originPopNum,LCTag,SGTag)
        popWithFit = sorted(popWithFit, key=itemgetter('fit'), reverse=True)[:self.popNum]      # 截取初始种群中前部分

        # 找到当前最优个体
        self.bestIndividual = self.selectBest(popWithFit)
        bestFit = self.bestIndividual['fit']
        allFits.append(bestFit)
        bestIndividuals.append(deepcopy(self.bestIndividual))

        for _ in range(self.iterTimes):
            # 选择、交叉及变异
            selectPop = self.selection(popWithFit)
            nextOff = []

            while len(nextOff) != self.popNum:
                offSpring1,offSpring2 = [selectPop.pop() for _ in range(2)]  # 后代间两两选择
                # 交叉
                if random.random() < self.crossParam:
                    if len(self.readyLC) or len(self.readySG) > 1:
                        crossOff1, crossOff2 = self.crossover(offSpring1,offSpring2,LCTag,SGTag)
                        # 变异
                        if random.random() < self.mutationParam:
                            mutationOff1 = self.mutation(crossOff1,LCTag,SGTag)
                            mutationOff2 = self.mutation(crossOff2,LCTag,SGTag)
                            popWithFit = self.quickFitness([mutationOff1,mutationOff2],2,LCTag,SGTag)
                            nextOff.extend(popWithFit)
                        else:
                            popWithFit = self.quickFitness([crossOff1, crossOff2],2,LCTag,SGTag)
                            nextOff.extend(popWithFit)
                    else:
                        nextOff.extend([offSpring1, offSpring2])
                else:
                    nextOff.extend([offSpring1, offSpring2])

            popWithFit = nextOff
            bestIndividual = self.selectBest(popWithFit)
            curFit = bestIndividual['fit']

            # 更新最优算子
            if curFit > bestFit:
                self.bestIndividual = bestIndividual
                bestIndividuals.append(deepcopy(self.bestIndividual))
                bestFit = curFit
                bestTimes = 1
            else:
                bestTimes += 1

            allFits.append(bestFit)

            # 判断终止条件
            if bestTimes >= self.sameBestTimes:
                break
            # todo: 加一个阈值，然后关联分析
            if len(bestIndividuals) >= 3 and (bestIndividuals[-1]['fit']-bestIndividuals[-3]['fit']) < 0.001:
                self.bestIndividual = self.correlationChoose(bestIndividuals[-3:],LCTag,SGTag)
                break

        print(allFits)
        # self.iterPlot(allFits)

        if LCTag:
            bestLC = self.transReadyToSuggestLC(self.bestIndividual,self.readyLCRef)
        if SGTag:
            bestSG = self.transReadyToSuggestSG(self.bestIndividual)
        # self.bestLC = {'cv.0':1,'cv.5':1,'cav.2':1}
        # self.bestSG = {'cav.3':15,'cav.5':13}

        return bestLC, bestSG