# Event-Based Multi-Robot Consensus Control with EKF and Cost Evaluation

This repository contains a ROS-based Python implementation for controlling multiple e-puck2 robots in a circular motion using **event-based state updates** and **Extended Kalman Filter (EKF)** for localization. The system computes linear and angular velocities for four robots while continuously evaluating a **cost** and logging robot states to an Excel file.

---

## Features

- **Multi-robot coordination:** Controls four e-puck2 robots simultaneously.
- **Event-based control:** Updates robot motion based on significant changes in position and orientation.
- **State estimation:** Uses EKF data combined with odometry for accurate localization.
- **cost computation:** Continuously evaluates a cost function to quantify system performance.
- **Data logging:** Saves robot positions, velocities, and costs to an Excel file for analysis.
- **ROS integration:** Fully compatible with ROS topics for odometry, EKF poses, and velocity commands.

---

## Installation

1. Make sure you have **ROS** installed (tested with ROS Kinetic/Melodic/Noetic).
2. Clone this repository:

```bash
git clone https://github.com/your-username/event_based_multi_robot_control.git
cd event_based_multi_robot_control
