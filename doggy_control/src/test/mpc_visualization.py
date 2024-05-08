# #!/usr/bin/env python
# # -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import csv

from decimal import Decimal

MPC_PLAN_HORIZON = 10
MPC_dt = 0.05
save_dt = 0.004

prediction_z = []
prediction_twist = []
prediction_grf = []
pos_z = []
twist_z = []  
grf = [] 
contact = []

with open('data/draw_mpc.csv', newline='') as csvfile:
    reader = csv.reader(csvfile)
    
    # count row 
    data = list(reader)
    row_count = len(list(data))
    # print("row:",row_count)
    # append pos_z twist_z grf
    for i in range(row_count):
        # array= [float(x) for x in data[i]]
        array = []
        for x in data[i]:
            try:
                array.append(float(x))
            except ValueError:
                array.append(0.0)  
        # print(array)
        pos_z.append(array[MPC_PLAN_HORIZON*3+2])
        twist_z.append(array[MPC_PLAN_HORIZON*3+3])
        grf.append(array[MPC_PLAN_HORIZON*3+4])
        contact.append(array[MPC_PLAN_HORIZON*3+5])
    
    for i in range(row_count):
        array = []
        for x in data[i]:
            try:
                array.append(float(x))
            except ValueError:
                array.append(0.0)  

        prediction_z = [array[j] for j in range(MPC_PLAN_HORIZON + 1)]
        prediction_twist = [array[MPC_PLAN_HORIZON + 1 + j] for j in range(MPC_PLAN_HORIZON + 1)]
        prediction_grf = [array[(MPC_PLAN_HORIZON + 1) * 2 + j] for j in range(MPC_PLAN_HORIZON)]

        if(i >= (row_count - MPC_dt / save_dt * MPC_PLAN_HORIZON)):
            pos_z_num = row_count - i
        else:
            pos_z_num = (int)(MPC_dt / save_dt * MPC_PLAN_HORIZON + 1)

        fig, (ax1, ax2, ax3) = plt.subplots(ncols=3, figsize=(18, 6))
        ax1.set_ylim([-0.1, 0.4])
        ax2.set_ylim([-2.5, 2.5])
        ax3.set_ylim([0, 800])

        t1 = [save_dt * i + MPC_dt * j for j in range(MPC_PLAN_HORIZON + 1)] # MPC_PLAN_HORIZON + 1
        t2 = [save_dt * (i+j) for j in range(pos_z_num)]  # pos_z_num
        t3 = [save_dt * i + MPC_dt * j for j in range(MPC_PLAN_HORIZON)] # MPC_PLAN_HORIZON

        
        ax1.step(t1, prediction_z, where='post')
        ax1.plot(t2 ,pos_z[i:i+pos_z_num], color = 'darkorange')
        ax1.plot(t2, [ (data) * 0.1 + 0.1 for data in contact[i:i+pos_z_num]] , color = 'green')
        ax1.axhline(y=0.26, color='gray', linestyle='--')
        ax1.grid(color='b', linestyle='--', linewidth=0.8)

        ax1.set_xlabel('time')
        ax1.set_ylabel('pos z')


        ax2.step(t1, prediction_twist, where='post')
        ax2.set_xlabel('time')
        ax2.plot(t2, twist_z[i:i+pos_z_num], color = 'darkorange')
        ax2.plot(t2, [ (1 - 2 * data) * 0.5 for data in contact[i:i+pos_z_num]] , color = 'green')
        ax2.set_ylabel('twist z')    
        ax2.grid(color='b', linestyle='--', linewidth=0.8)   
        # ax4 = ax2.twinx()
        # ax4.plot(t2, twist_z[i:i+pos_z_num], color = 'darkorange')
        # ax4.set_ylabel('twist z')
        # ax4.tick_params(axis='twist z', colors='orange')

        ax3.step(t3, prediction_grf, where='post')
        ax3.set_xlabel('time')
        ax3.plot(t2, grf[i:i+pos_z_num], color = 'darkorange')
        ax3.plot(t2, [data * 100 for data in contact[i:i+pos_z_num]] , color = 'green')
        ax3.set_ylabel('grf')
        ax3.grid(color='b', linestyle='--', linewidth=0.8)
        # ax5 = ax3.twinx()
        # ax5.plot(t2, grf[i:i+pos_z_num], color = 'darkorange')
        # ax5.set_ylabel('grf')

        fig.savefig(f'pic/pic{i}:{Decimal(t1[0]).quantize(Decimal("0.000"))}s.png', dpi=100)
        print(f'{i + 1}/{row_count + 1}:{Decimal(t1[0]).quantize(Decimal("0.000"))}s.png saved.')
        plt.close()
        # print("array %d: %f", i, array)
        #print(prediction_z)
    # print(array1)

    