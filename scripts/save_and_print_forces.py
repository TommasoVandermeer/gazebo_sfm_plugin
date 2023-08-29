#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_sfm_plugin.msg import Forces
import matplotlib.pyplot as plt
import math
import numpy as np

class SaveForcesNode(Node):
    # Constructor
    def __init__(self):
        super().__init__("saveforces_node")

        # Lists to store the forces
        # self.desired_force = []
        # self.obstacle_force = []

        # Lists to store magnitude and phase of each force
        self.des_force_magnitude = []
        self.des_force_phase = []
        self.obs_force_magnitude = []
        self.obs_force_phase = []

        # Data counter
        self.count = 0

        # Forces subscriber
        self.forces_sub = self.create_subscription(Forces, '/forces/actor1', self.forces_callback, 1)

    # Forces callback
    def forces_callback(self, msg: Forces):
        # Store forces
        # self.desired_force.append([msg.desired_force.x,msg.desired_force.y])
        # self.obstacle_force.append([msg.obstacle_force.x,msg.obstacle_force.y])

        # Compute magnitude and phase of each force
        self.des_force_magnitude.append(math.sqrt(msg.desired_force.x**2 + msg.desired_force.y**2))
        self.des_force_phase.append(math.degrees(math.atan2(msg.desired_force.y, msg.desired_force.x)))
        self.obs_force_magnitude.append(math.sqrt(msg.obstacle_force.x**2 + msg.obstacle_force.y**2))
        self.obs_force_phase.append(math.degrees(math.atan2(msg.obstacle_force.y, msg.obstacle_force.x)))

        # Increment data counter
        self.count += 1

def main(args=None):

    # Initialize the training node to get the desired parameters
    rclpy.init()
    node = SaveForcesNode()
    node.get_logger().info("Save forces node has been created")

    # Spin the node
    while(node.count <= 7000):
        rclpy.spin_once(node)

    # Time vector
    t = np.arange(0,7.001,0.001)

    # Print results
    fig, axs = plt.subplots(2, 2)
    fig.suptitle('SFM2 forces analysis (HSFM without heading)')
    axs[0, 0].plot(t, node.des_force_magnitude, color='tab:red')
    axs[0, 0].set_title('Desired Force Magnitude')
    axs[0, 0].grid()
    axs[1, 0].plot(t, node.des_force_phase, color='tab:orange')
    axs[1, 0].set_title('Desired Force Phase')
    axs[1, 0].grid()
    axs[0, 1].plot(t, node.obs_force_magnitude, color='tab:blue')
    axs[0, 1].set_title('Obstacle Force Magnitude')
    axs[0, 1].grid()
    axs[1, 1].plot(t, node.obs_force_phase, color='tab:green')
    axs[1, 1].set_title('Obstacle Force Phase')
    axs[1, 1].grid()
    plt.show()

if __name__ == "__main__":
    main()