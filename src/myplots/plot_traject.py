#!/usr/bin/env python3

import matplotlib.pyplot as plt
import rosbag2_py
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message



# ======================================================================
# Configuration
# ======================================================================

BAG_PATH = "plans4"      # Relative path to bag folder
PLAN_TOPIC = "/plan"
ODOM_TOPIC = "/amcl_pose"

# ======================================================================
def knn_numpy(global_path, robot_traj):
    global_path = np.array(global_path)
    robot_traj = np.array(robot_traj)

    errors = []
    matched = []

    for p in robot_traj:
        dists = np.linalg.norm(global_path - p, axis=1)
        idx = np.argmin(dists)

        matched.append(global_path[idx])
        errors.append(dists[idx])

    return np.array(matched), np.array(errors)

def main():

    storage_options = rosbag2_py.StorageOptions(
        uri=BAG_PATH,
        storage_id='mcap'
    )

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # Get topic types
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}

    # Check topics exist
    if PLAN_TOPIC not in type_map:
        raise RuntimeError(f"{PLAN_TOPIC} not found in bag")

    if ODOM_TOPIC not in type_map:
        raise RuntimeError(f"{ODOM_TOPIC} not found in bag")

    # Get message classes
    path_msg_type = get_message(type_map[PLAN_TOPIC])
    odom_msg_type = get_message(type_map[ODOM_TOPIC])

    # Store last global plan
    global_x = None
    global_y = None

    # Store executed trajectory
    robot_x = []
    robot_y = []
    i = 0
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic == PLAN_TOPIC:
            if i == 1 :
                    msg = deserialize_message(data, path_msg_type)

                    global_x = [
                        pose.pose.position.x
                        for pose in msg.poses
                    ]

                    global_y = [
                        pose.pose.position.y
                        for pose in msg.poses
                    ]
            i = i + 1

        # Robot trajectory
        elif topic == ODOM_TOPIC:

            msg = deserialize_message(data, odom_msg_type)

            robot_x.append(msg.pose.pose.position.x)
            robot_y.append(msg.pose.pose.position.y)

    # Plot results
    # make same length (simple truncation)
    kmeans,errors = knn_numpy(np.vstack([global_x,global_y]).T,np.vstack([robot_x,robot_y]).T)
    rmse = np.sqrt(np.mean(errors**2))

    fig, (ax1, ax2) = plt.subplots(
    2, 1,
    figsize=(8, 10),
    gridspec_kw={'height_ratios': [3, 1]}
    )

    if global_x is not None:
        ax1.plot(global_x, global_y,
                linewidth=2,
                label="Trajectoire planifiée")
    if(BAG_PATH == "plans1" or BAG_PATH == "plans2"):
        if robot_x:
            ax1.plot(robot_x, robot_y,
                    linewidth=2,
                    label="Trajectoire exécutée par MPPI ")
    else :
        if robot_x:
            ax1.plot(robot_x, robot_y,
                    linewidth=2,
                    label="Trajectoire exécutée par DWB")

    # Annotate start and goal
    if global_x is not None and len(global_x) > 0:

        ax1.scatter(global_x[0], global_y[0], marker='o')
        ax1.annotate(
            'Pose Initial',
            (global_x[0], global_y[0]),
            xytext=(10, 10),
            textcoords='offset points',
            arrowprops=dict(arrowstyle='->')
        )

        ax1.scatter(global_x[-1], global_y[-1], marker='x')
        ax1.annotate(
            'Pose cible',
            (global_x[-1], global_y[-1]),
            xytext=(10, 10),
            textcoords='offset points',
            arrowprops=dict(arrowstyle='->')
        )

    ax1.set_xlabel("X [m]")
    ax1.set_ylabel("Y [m]")
    if(BAG_PATH == "plans1" or BAG_PATH == "plans2"):
        ax1.set_title("Trajectoire exécutée par MPPI vs Trajectoire planifiée")
    else :
        ax1.set_title("Trajectoire exécutée par DWB vs Trajectoire planifiée")
    ax1.axis("equal")
    ax1.grid(True)
    
    ax1.legend()
    ax2.plot(errors, label="Erreur")
    ax2.axhline(rmse, linestyle='--', label=f"RMSE = {rmse:.3f}")

    ax2.set_xlabel("temps")
    ax2.set_ylabel("Error [m]")
    ax2.set_title("Erreur de Positon au cours de la navigation")
    ax2.grid(True)
    ax2.legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()