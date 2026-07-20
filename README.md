# ROS2 Inverted Pendulum Simulation and LQR Control

> A complete ROS2 Humble project demonstrating dynamic modeling, state-space representation, Gazebo simulation, and optimal LQR control of an inverted pendulum system.

---

## 🚧 Project Status

> **This project is currently under active development.**  
> New features, documentation, simulation results, and hardware implementation will be added progressively.

![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-22314E?logo=ros&logoColor=white)
![Gazebo](https://img.shields.io/badge/Simulation-Gazebo-blue)
![Python](https://img.shields.io/badge/Python-3.x-yellow?logo=python)
![Control](https://img.shields.io/badge/Control-LQR-success)
![License](https://img.shields.io/badge/License-MIT-green)

---

# Overview

This repository presents the complete development process of an inverted pendulum control system using the ROS2 ecosystem.

Instead of focusing solely on the final controller, this project documents the complete engineering workflow—from mechanical modeling and dynamic equation derivation to simulation, controller implementation, and future hardware deployment.

The project includes:

- Dynamic modeling using Newton-Euler mechanics
- State-space representation
- System linearization around the upright equilibrium
- Optimal LQR controller design
- ROS2 node implementation in Python
- Gazebo simulation environment
- Modular ROS2 package architecture
- Future extensions including swing-up control and real hardware implementation

This repository is intended for robotics students, control engineers, and developers interested in learning how modern robotic control systems are designed from first principles.

---

# Table of Contents

- [Overview](#overview)
- [Project Motivation](#project-motivation)
- [Project Highlights](#project-highlights)
- [Engineering Workflow](#engineering-workflow)
- [System Model](#system-model)
- [Dynamic Modeling](#dynamic-modeling)
- [State-Space Representation](#state-space-representation)
- [System Linearization](#system-linearization)
- [LQR Controller Design](#lqr-controller-design)
- [ROS2 Software Architecture](#ros2-software-architecture)
- [Repository Structure](#repository-structure)
- [Installation](#installation)
- [Running the Simulation](#running-the-simulation)
- [Simulation Results](#simulation-results)
- [Future Work](#future-work)
- [License](#license)