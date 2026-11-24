# Autonomous Robotics Challenges (`SJK001-7`)
**Course ID / Project:** SJK001-7  
**Platform:** [JdeRobot Robotics Academy](https://jderobot.github.io/RoboticsAcademy/) / [Unibotics](https://unibotics.org)

This repository contains the complete solutions, experimental benchmarks, and technical documentation for two autonomous robotics challenges developed in **ROS 2**, **OpenCV**, and **Gazebo**:

1. **[Autonomous Racing Challenge: Visual Follow Line](./SJK001-7_Autonomous_Racing_Challenge-main/)** — High-speed vision-based circuit navigation comparing **P**, **PD**, and **PID** control architectures on CPU and GPU backends.
2. **[Autonomous Drone Challenge: Rescue People](./SJK001-7_Autonomous_Drone_Challenge-main/)** — Autonomous maritime search-and-rescue mission implemented both with JdeRobot's **HAL** abstraction and directly on **native ROS 2 (`rclpy`) + Aerostack2**.

---

## Repository Structure

```text
.
├── SJK001-7_Autonomous_Racing_Challenge-main/   # Project 1: Visual Follow Line documentation, metrics & media
├── code_Controllers/                            # Project 1: P, PD, and PID controller Python implementations
├── SJK001-7_Autonomous_Drone_Challenge-main/    # Project 2: Rescue People (HAL & direct ROS 2 / Aerostack2)
└── README.md                                    # Repository overview & environment setup
