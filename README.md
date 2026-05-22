# Deep Reinforcement Learning for Effective Indoor Mobile Robot Navigation

![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue?style=flat-square&logo=ros)
![Python](https://img.shields.io/badge/Python-3.10-blue?style=flat-square&logo=python)
![PyTorch](https://img.shields.io/badge/PyTorch-2.6.0-orange?style=flat-square&logo=pytorch)
![Gazebo](https://img.shields.io/badge/Gazebo-Classic-green?style=flat-square)
![License](https://img.shields.io/badge/License-Apache%202.0-yellow?style=flat-square)

**B.Sc. Thesis — University of Khartoum, Faculty of Engineering, July 2026**

> *"Deep Reinforcement Learning Approach for Effective Indoor Mobile Robot Navigation"*

**Authors:**
- Mohammed Faiz Mohammed Noor Ahmed (184089) — [mo7ammedfaiz@gmail.com](mailto:mo7ammedfaiz@gmail.com)
- Al-hassen Mohammed Ahmed Sabeeh (184025) 

**Supervisor:** Prof. Sharief F. Babikir  
**Department:** Electrical and Electronic Engineering

---

## Overview

This repository implements a **Dueling Double DQN (D3QN)** navigation policy for the TurtleBot3 Burger robot in a Gazebo simulation. The agent learns mapless navigation directly from LiDAR observations and goal-relative information, without requiring a pre-built map.

The project benchmarks D3QN against a reference DQN baseline through five targeted optimisations:

| Optimisation | Baseline DQN | Proposed D3QN |
|:---|:---|:---|
| Architecture | DQN (512 units) | Dueling Double DQN (1024 units) |
| Action space | 5 actions (forward-only) | 12 actions (includes zero-linear rotation) |
| LiDAR resolution | 40 rays @ 9° intervals | 60 rays @ 6° intervals |
| State dimension | 44 | 64 |
| Target update | Hard every 1,000 episodes | Soft every 10 + hard every 500 iterations |

---

## Demo

### Baseline DQN

<video src="https://github.com/user-attachments/assets/e426a348-9764-44d6-bff5-1229bef91d9c" controls width="100%"></video>

### Proposed D3QN

<video src="https://github.com/user-attachments/assets/e50cb310-4c1d-40ce-a821-400da9a95a4c" controls width="100%"></video>

---

## Results

### Training Phase

| Metric | Baseline DQN | Proposed D3QN |
|:---|:---|:---|
| Training episodes | 10,500 | 7,200 |
| Training duration | 40 h | 31.3 h |
| Cumulative success rate | 26.7% | **63.9%** |
| Loss stabilisation | Episode ~1,000 | Episode ~800 |

### Testing Phase (200 episodes)

| Metric | Baseline DQN | Proposed D3QN |
|:---|:---|:---|
| **Success rate** | 55% | **84%** |
| Collision — static obstacles | 10% | 7% |
| Collision — dynamic obstacles | 13% | 9% |
| **Timeout** | 22% | **0%** |
| Avg. distance / episode | 1.21 m | 1.87 m |
| Avg. episode duration | 7.8 s | 11.1 s |

D3QN achieves a **2.39× higher cumulative training success rate** in **31.4% fewer episodes**, and completely eliminates navigation timeouts by enabling zero-linear-velocity in-place rotation.

---

## Architecture

### State Vector (64-dimensional)

```plain
S_t = [ L_1 ... L_60,  d_goal/d_max,  θ_goal/π,  v_lin_prev,  v_ang_prev ]
       └────────────┘  └─────────────────────────┘ └─────────────────────┘
       60 LiDAR rays      goal-relative info          previous action
```

### Reward Function

```plain
R_t = R_yaw + R_progress + R_obstacle + R_step + R_terminal

  R_yaw       = −0.3 × |θ_goal|
  R_progress  = 200 × (d_prev − d_curr)
  R_obstacle  = −25  if min_lidar < 0.25 m
  R_step      = −1   per step
  R_terminal  = +3500 (goal) / −3500 (collision)
```

### Dueling Network

```plain
              ┌────────────────────────┐
              │  Shared MLP            │  (input → 512 → 1024 → 1024)
              └───────────┬────────────┘
          ┌───────────────┴──────────────┐
   Value stream V(s)         Advantage stream A(s, a)
          └───────────────┬──────────────┘
     Q(s,a) = V(s) + ( A(s,a) − mean_a′ A(s,a′) )
```

---

## Repository Structure

```plain
ROS2-DRL/
├── src/
│   └── turtlebot3_Navigation_D3QN/
│       ├── agent/               # D3QN network, training loop, replay buffer
│       ├── environment/         # ROS 2 environment node (LiDAR, odometry, step service)
│       ├── test/                # Evaluation scripts
│       ├── checkpoints & Logs/  # Saved model weights and training logs
│       └── setup.py
├── src/baseline.mp4             # Baseline DQN test demo
├── src/D3QN.mp4                 # Proposed D3QN test demo
└── .gitignore
```

---

## Setup

### Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill
- Python 3.10
- PyTorch 2.6.0 + CUDA 12.4
- TurtleBot3 packages + Gazebo Classic

### Install & Build

```bash
git clone https://github.com/Mo7ammed-faiz/ROS2-DRL.git
cd ROS2-DRL/src
colcon build --packages-select turtlebot3_Navigation_D3QN
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

---

## Hardware

| Parameter | Value |
|:---|:---|
| Robot | TurtleBot3 Burger |
| Max linear velocity | 0.22 m/s |
| Max angular velocity | 2.84 rad/s |
| Training GPU | NVIDIA GeForce RTX 4060 Laptop (8 GB GDDR6) |
| Simulation arena | Gazebo Classic, 4.2 m × 4.2 m |

---

## Acknowledgements

This work builds upon the TurtleBot3 DRL navigation framework by [tomasvr](https://github.com/tomasvr/turtlebot3_drlnav) and the original TurtleBot3 packages by [ROBOTIS](https://github.com/ROBOTIS-GIT/turtlebot3), both licensed under Apache License 2.0.

---

## License

The D3QN modifications and original contributions in this repository are copyright © 2026 Mohammed Faiz Mohammed Noor Ahmed & Al-hassen Mohammed Ahmed Sabeeh.

Derived components retain their original Apache License 2.0 from ROBOTIS CO., LTD.
