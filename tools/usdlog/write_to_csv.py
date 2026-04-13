# -*- coding: utf-8 -*-
"""
code to write usd logged crazyflie data to csv in format used for the snn pid
"""
import cfusdlog
import matplotlib.pyplot as plt
import re
import argparse
import pandas as pd
import numpy as np
from scipy import stats

parser = argparse.ArgumentParser()
parser.add_argument("--filename", type=str, default="tools/usdlog/log03")
args = parser.parse_args()

# decode binary log data
logData = cfusdlog.decode(args.filename)

#only focus on regular logging
logData = logData['fixedFrequency']

# set window background to white
plt.rcParams['figure.facecolor'] = 'w'

# number of columns and rows for suplot
plotCols = 1
plotRows = 1

# let's see which keys exists in current data set
keys = []
for k, v in logData.items():
    keys.append(k)

# first simply store in dataframe
data = pd.DataFrame()
for key in keys:
    data[key] = logData[key]

data.to_csv(f'{args.filename}.csv', index=False)