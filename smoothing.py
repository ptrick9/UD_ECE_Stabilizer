__author__ = 'ptric'

import os

print(os.getcwd())

f = open("smoothing", 'r')

xr = []
xc = []
yr = []
yc = []
zr = []
zc = []

for line in f:
    if 'X' in line:
        xr.append(line.split(' unsmoothed: ')[1].split(' raw')[0])
        xc.append(line.split(' corrected: ')[1].split('\n')[0])
    elif 'Y' in line:
        yr.append(line.split(' unsmoothed: ')[1].split(' raw')[0])
        yc.append(line.split(' corrected: ')[1].split('\n')[0])
    elif 'Z' in line:
        zr.append(line.split(' unsmoothed: ')[1].split(' raw')[0])
        zc.append(line.split(' corrected: ')[1].split('\n')[0])

i = 0
while i < len(xr):
    print(xr[i] + "\t" + xc[i] + "\t" + yr[i] + "\t" + yc[i] + "\t" + zr[i] + "\t" + zc[i])
    i += 1