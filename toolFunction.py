# -*- coding: utf-8 -*-
# @Time    : 2024/10/14 15:47
# @Author  : Mminmint
# @File    : toolFunction.py
# @Software: PyCharm

from __future__ import absolute_import
from __future__ import print_function
import os, sys
import optparse


'''
去除找TFV和TLV过程中可能出现的双重元组
Input:  det_tuple：(('cv_16',201.215),)
Output: det_tuple：('cv_16',201.215)
'''

def trimTuple(det_tuple):
    empty = ()

    if det_tuple != empty:
        det_tuple = det_tuple[0]
    else:
        det_tuple = (None,None)

    return det_tuple


def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                        default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


def startSUMO(gui:bool,sumocfgFile:str):
    # 校验环境变量中是否存在SUMO_HOME
    if 'SUMO_HOME' in os.environ:
        tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
        sys.path.append(tools)
    else:
        sys.exit("please declare environment variable 'SUMO_HOME'")

    # get_options
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()

    # 构造sumoBinary与sumoCmd
    if gui:
        sumoBinary = "sumo-gui"
    else:
        sumoBinary = "sumo"
    sumoCmd = [sumoBinary, "-c", sumocfgFile]

    return sumoCmd


def nearestFive(num):
    return round(num / 5) * 5