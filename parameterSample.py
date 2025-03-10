# -*- coding: utf-8 -*-
# @Time    : 2025/3/10 14:05
# @Author  : Mminmint
# @File    : parameterSample.py
# @Software: PyCharm

import numpy as np


def LCReactTime():
    frequencyDict = {0: 0.019704433497536946, 1: 0.034482758620689655,
                     2: 0.029556650246305417, 3: 0.019704433497536946,
                     4: 0.07389162561576355, 5: 0.06896551724137931,
                     6: 0.1330049261083744, 7: 0.14285714285714285,
                     8: 0.08374384236453201, 9: 0.07881773399014778,
                     10: 0.059113300492610835, 11: 0.08374384236453201,
                     12: 0.06403940886699508, 13: 0.014778325123152709,
                     14: 0.019704433497536946, 15: 0.009852216748768473,
                     16: 0.029556650246305417, 17: 0.009852216748768473,
                     18: 0.014778325123152709, 19: 0.009852216748768473}


# 随机采样函数
def sample_time(probability_dict, n_samples=1):
    times = list(probability_dict.keys())
    probs = list(probability_dict.values())
    return np.random.choice(times, size=n_samples, p=probs)