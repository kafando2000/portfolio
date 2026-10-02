#!/usr/bin/env python3

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt

# Path to your bag
bag_path = "Joints"

# List of mimic joints to ignore
mimic_joints = {
    "ee_unactuated_tooth_joint",
    "left_rear_wheel_joint",
    "right_rear_wheel_joint"
}

storage_options = rosbag2_py.StorageOptions(
    uri=bag_path,
    storage_id='mcap'  
)

converter_options = rosbag2_py.ConverterOptions('', '')

reader = rosbag2_py.SequentialReader()
reader.open(storage_options, converter_options)

joint_efforts = {}
joint_velocities = {}
joint_power = {}
joint_times = {}
total_power = []
process_times = []

t0 = None

while reader.has_next():
    topic, data, timestamp = reader.read_next()

    if topic != "/joint_states":
        continue

    msg = deserialize_message(data, JointState)

    if t0 is None:
        t0 = timestamp

    t = (timestamp - t0) * 1e-9  # ns → s
    process_times.append(t)
    for name, effort in zip(msg.name, msg.effort):

        if name in mimic_joints:
            continue

        if name not in joint_efforts:
            joint_efforts[name] = []
            joint_times[name] = []

        joint_efforts[name].append(effort)
        joint_times[name].append(t)

    for name, vel in zip(msg.name, msg.velocity):
        if name in mimic_joints:
            continue
        if name not in joint_velocities:
            joint_velocities[name] = []
        joint_velocities[name].append(vel)

import numpy as np


for joint in joint_velocities:
    joint_power[joint] = (np.array(joint_efforts[joint])*np.array(joint_velocities[joint]))

i = 0

total = 0.0
dt = process_times[1] - process_times[0]
e_total = 0.0

for  ti in process_times :
    ti_consumed_e = 0.0
    for joint in joint_power:
        ti_consumed_e = ti_consumed_e + max(0.0,(joint_power[joint])[i])
    e_total = e_total + ti_consumed_e*dt
    total_power.append(ti_consumed_e)
    i = i + 1
print(e_total)
for joint in joint_efforts:
    plt.figure()
    plt.plot(joint_times[joint], joint_efforts[joint])
    plt.xlabel("Time [s]")
    plt.ylabel("couple [Nm]")
    plt.title(joint)
    plt.grid()
for joint in joint_power :
    plt.figure()
    plt.plot(joint_times[joint], joint_power[joint])
    plt.xlabel("Time [s]")
    plt.ylabel("Puissance [Watts]")
    plt.title(joint)
    plt.grid()

plt.figure()
plt.plot(process_times, total_power)
plt.xlabel("Time [s]")
plt.ylabel("Puissance [Watts]")
plt.title("Puissance totale consommée")
plt.grid()

plt.show()
