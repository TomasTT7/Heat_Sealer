# -*- coding: utf-8 -*-
import time
import math
import numpy as np
from matplotlib import pyplot as plt


# Cement-bonded particleboard, Teflon tape, Teflon tape, Film, Silicone rubber profile
# thickness [m], k [W/mK], Cp [J/KgK], ρ [kg/m3], ε []
materials = np.array([[0.003,       0.00018,    0.00018,    0.00008,    0.003],
                     [0.244,        0.148,      0.148,      0.302,      0.2],
                     [1880.0,       936.0,      936.0,      2000.0,     1255.0],
                     [1350.0,       1900.0,     1900.0,     972.0,      1160.0],
                     [0.54,         0.85,       0.85,       0.7,        0.93]])


pos = 2             # position of heating wire among materials
film_pos = 171      # position of film-film boundary among dx's

dt = 0.0005         # [s]
dx = 0.00002        # [m]

decimationX = 1     # every Xth sample
decimationT = 100   # every Xth sample

length = 11.0       # [s] simulation length

Tinf = 25           # [°C] infinity

Vin = 12.0          # [V] input voltage
Rw = 0.274          # [Ω] wire resistance
R2 = 0.220          # [Ω] additional resistance
I = Vin / (Rw + R2)
Ein = I**2 * Rw * dt

Nl = 0.368          # [m] Nichrome wire length
Nw = 0.010          # [m] Nichrome wire width
Nm = 0.00386        # [kg] Nichrome wire mass
NCp = 450           # [J/kgK] Nichrome specific heat

h = 15              # [W/m2K] heat transfer coefficient of convection to air

sigma = 0.00000005670374419 # [W/m2K4] Stefan-Boltzmann constant


start = time.time()


# Temperature Array
p, layers, wire_pos = (0,)*3
_yticks = [1 * dx]

for mt in materials[0]:
    layers += mt / dx
    _yticks.append((layers + 1) * dx)
    
    if p is pos-1:
        wire_pos = layers + 1
    
    p = p + 1

temps = np.zeros((2, int(layers) + 4))
temps.fill(Tinf)

mpr = int(length / dt)
print("Datapoints: {:d}\tRows: {:d}\tColumns: {:d}".format((int(length / dt)) * (int(layers) + 4), int(length / dt), int(layers) + 4))
print("Decimated Datapoints: {:d}\tRows: {:d}\tColumns: {:d}".format(int(length / dt / decimationT) * int((layers + 4) / decimationX), \
                                                                     int(length / dt / decimationT), int((layers + 4) / decimationX)))


# Alpha array
alphas = np.zeros((5, int(layers) + 4))

m, n = materials.shape

x = 1
p = 0

for i in range(n):
    _layers = materials[0][i] / dx
    
    for y in range(int(_layers)):
        alphas[0][x] = materials[1][i] / (materials[2][i] * materials[3][i])
        alphas[1][x] = materials[1][i]
        alphas[2][x] = materials[2][i]
        alphas[3][x] = materials[3][i]
        alphas[4][x] = materials[4][i]
        x = x + 1
        
        if i == n - 1 and y == int(_layers) - 1:
            alphas[0][x+1] = materials[1][i] / (materials[2][i] * materials[3][i])
            alphas[1][x+1] = materials[1][i]
            alphas[2][x+1] = materials[2][i]
            alphas[3][x+1] = materials[3][i]
            alphas[4][x+1] = materials[4][i]
    
    if p is pos-1:
        x = x + 1
    
    p = p + 1


# Clear Data File
mr, nr = (0,)*2
factor = 1000
width = 0
mm, nm = materials.shape

for i in range(nm):
    width = width + materials[0][i]

width = width + 4 * dx # + lower boundary, wire and two upper boundaries

f = open("data", 'w')
f.write(str(dt) + ',' + str(dx) + ',' + str(decimationT) + ',' + str(decimationX) \
        + ',' + str(int(wire_pos)) + ',' + str(int(layers)) + ',' + str(width*factor) + ',')

for i in range(len(_yticks)):
    if i < pos:
        _yticks[i] = _yticks[i] - dx * 0.5
    elif i > pos:
        _yticks[i] = _yticks[i] + dx * 0.5
    
    _yticks[i] = _yticks[i] * factor - wire_pos * dx * factor
    f.write(str(_yticks[i]))
    
    if i == len(_yticks) - 1:
        f.write('\n')
    else:
        f.write(',')

for z in range(int(layers) + 4):
    if z == 0 or z == 1 or z == int(layers) + 3 or z == int(layers) + 2 or z == int(layers) + 1 or z == wire_pos or z % decimationX == int(decimationX / 2):
        f.write(str("{:.1f}".format(temps[0][z])))
        f.write(',')
        nr = nr + 1
    
    if z == int(layers) + 3:
        f.write('\n')

mr = mr + 1
f.close()


# Heat Transfer
progress = 0

for i in range(int(length / dt)):
    if i is 0:
        continue
    
    for y in range(int(layers) + 3):
        progress = progress + 1
        
        if y == 0 or y == int(layers) + 2: # lower and upper Air
            continue
        
        if y == wire_pos: # Wire
            temps[1][y] = temps[0][y] + (Ein - (alphas[1][y-1] * (Nl * Nw) * (temps[0][y] - temps[0][y-1]) / dx * dt) \
                        - (alphas[1][y+1] * (Nl * Nw) * (temps[0][y] - temps[0][y+1]) / dx * dt)) / (Nm * NCp)
            continue
        
        if y == 1: # lower boundary - conduction, convection, radiation
            temps[1][y] = temps[0][y] + ((alphas[1][y] * (Nl * Nw) * (temps[0][y+1] - temps[0][y]) / dx * dt) \
                        - (h * (Nl * Nw) * (temps[0][y] - temps[0][y-1]) * dt) - (alphas[4][y] * (Nl * Nw) * sigma \
                        * ((temps[0][y] + 273.15)**4 - (temps[0][y-1] + 273.15)**4) * dt)) / (alphas[3][y] * Nl * Nw * dx * alphas[2][y])
            continue
        
        if y == int(layers) + 1:  # upper boundary - conduction, convection, radiation
            temps[1][y] = temps[0][y] + ((alphas[1][y-1] * (Nl * Nw) * (temps[0][y-1] - temps[0][y]) / dx * dt) \
                        - (h * (Nl * Nw) * (temps[0][y] - temps[0][y+1]) * dt) - (alphas[4][y-1] * (Nl * Nw) * sigma \
                        * ((temps[0][y] + 273.15)**4 - (temps[0][y+1] + 273.15)**4) * dt)) / (alphas[3][y-1] * Nl * Nw * dx * alphas[2][y-1])
            continue
        
        temps[1][y] = (temps[0][y-1] + temps[0][y+1] - 2 * temps[0][y]) * (alphas[0][y] * dt / dx**2) + temps[0][y] # inside points
    
    # add data row to file
    f = open("data", 'a')
    
    if i == int(length / dt) - 1 or i % decimationT == decimationT - 1:
        for z in range(int(layers) + 4):
            if z == 0 or z == 1 or z == int(layers) + 3 or z == int(layers) + 2 or z == int(layers) + 1 or z == wire_pos or z % decimationX == int(decimationX / 2):
                f.write(str("{:.1f}".format(temps[1][z])))
                f.write(',')
                nr = nr + 1
            
            if z == int(layers) + 3:
                f.write('\n')
            
        mr = mr + 1
        
    f.close()
    
    # store only last and current rows
    for z in range(int(layers) + 4):
        temps[0][z] = temps[1][z]
    
    # inform about progress
    if progress > 100000:
        print("{:.1f}%".format(float(i) / float(mpr) * 100.0))
        progress = 0

nr = nr / mr

print("{:.2f}s".format(time.time() - start))
start = time.time()

print("film: {:.1f}\xb0C position: {:d}/{:d} {:.2f}mm".format(temps[-1][int(film_pos)], int(film_pos), int(layers) + 2, \
                                                              (int(film_pos)-int(wire_pos))*dx*1000.0))
print("wire: {:.1f}\xb0C position: {:d}/{:d} 0.00mm".format(temps[-1][int(wire_pos)], int(wire_pos), int(layers) + 2))
print("opposite: {:.1f}\xb0C position: {:d}/{:d} {:.2f}mm".format(temps[-1][int(wire_pos)-(int(film_pos)-int(wire_pos))], \
                                                                  int(wire_pos)-(int(film_pos)-int(wire_pos)), int(layers) \
                                                                  + 2, -(int(film_pos)-int(wire_pos))*dx*1000.0))


# Plot
readtemps = np.zeros((mr, nr))

f = open("data", 'r')

for i in range(mr+1):
    line = f.readline()
    
    if i == 0:
        continue
    
    splitline = line.split(',')
    
    for y in range(nr):
        readtemps[i-1][y] = float(splitline[y])

f.close()

_w = width * factor + (width * factor / (nr - 1))
_st = width * factor / (nr - 1)
_mov = width * factor * ((wire_pos + 1) / int(layers + 4))
xx, yy = np.mgrid[0.0:mr*dt*decimationT:dt*decimationT, 0.0-_mov:_w-_mov:_st]

fig, ax = plt.subplots()
pcm = ax.pcolormesh(xx, yy, readtemps, cmap='jet', vmin=0, vmax=np.abs(readtemps).max())
ax.set_xlabel('[s]', fontsize=10)
ax.set_ylabel('[mm]', fontsize=10, rotation=0)
ax.xaxis.set_label_coords(1.0, -0.075)
ax.yaxis.set_label_coords(-0.1, 1.02)
cb = plt.colorbar(pcm)
cb.set_label(u'[\xb0C]', fontsize=10, rotation=0, y=1.075)
plt.grid(axis='x', linestyle='--', linewidth=0.25, color='k')
plt.grid(axis='y', linestyle='--', linewidth=0.25, color='k')
plt.yticks(_yticks)
plt.show()
