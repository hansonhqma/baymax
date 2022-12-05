#!/usr/bin/python3 -i
import numpy as np

import fk_ik_jq as kin
import naive_fk
from pathgen import squared_velocity_path
import std_msgs.msg

import matplotlib.pyplot as plt

msg = std_msgs.msg.Float32MultiArray()
msg.data = [0.15, 0.13, 0.1]

starting_config = [0, 0, 0, 0, 0]

lamda_max = 100
# get current xyz
initial_xyz = naive_fk.tf_base_to_tool(*(starting_config))
# calculate delta xyz
delta_xyz = [msg.data[i] - initial_xyz[i] for i in range(3)]

# generate xyz motion profiles
xyz_paths = [[squared_velocity_path(0, lamda_max, delta_xyz[i]) for i in range(3)]]
for i in range(1, lamda_max):
    xyz_paths.append([xyz_paths[-1][dim]+squared_velocity_path(i, lamda_max, delta_xyz[dim]) for dim in range(3)])
for i in range(lamda_max):
    for dim in range(3):
        xyz_paths[i][dim] += initial_xyz[dim]

xyz_paths = np.array(xyz_paths).reshape((100,3))

plt.plot(xyz_paths[:,0])
plt.plot(xyz_paths[:,1])
plt.plot(xyz_paths[:,2])
plt.show()

# ik on motion profiles to get joint motion profiles
joint_paths = []
recent_config = [x for x in starting_config]
for i in range(len(xyz_paths)):
    angles = kin.DOFBOT.ik(xyz_paths[i], np.array(recent_config))
    recent_config = [x for x in angles]
    joint_paths.append(list(angles) + [0])



jp = np.array(joint_paths)