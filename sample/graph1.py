# -*- coding: utf-8 -*-
"""
Created on Sun Dec  3 22:29:07 2023

@author: square
"""

import matplotlib.pyplot as plt
import pandas as pd

df = pd.read_csv("dump.txt")

fig, ax = plt.subplots()

x = df['step']
y = df['position']

ax.plot(x,y)

