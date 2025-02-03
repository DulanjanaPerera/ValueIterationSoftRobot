# Tetrahedral Soft Robot Navigation

This repository contains a Python implementation of an autonomous navigation system for a tetrahedral soft robot using value iteration in the PyBullet simulation environment. The project explores motion planning and control techniques to enable the robot to navigate a discrete honeycomb-like environment.

## Table of Contents

- [Introduction](#introduction)
- [Features](#features)
- [System Requirements](#system-requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [Simulation Scenarios](#simulation-scenarios)
- [Results](#results)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgments](#acknowledgments)

## Introduction

Soft robotics is an emerging field with applications in areas such as medical robotics, exploration, and industrial automation. However, due to the stochastic nature of soft robots, navigation remains a challenging task. This project implements a value iteration-based path planning and control algorithm for a tetrahedral soft robot, enabling it to move autonomously in a constrained environment.

## Features

- **Physics-based Simulation:** Uses PyBullet to model soft robot dynamics.
- **Finite Horizon Value Iteration:** Implements an optimal control algorithm for navigation.
- **Discrete State Space:** The robot moves in a honeycomb-like grid structure with predefined actions.
- **Obstacle Avoidance:** Ensures safe navigation in cluttered environments.
- **Customizable Parameters:** Users can modify environment settings and robot properties.

## System Requirements

- Python 3.8 or later
- PyBullet
- NumPy
- Matplotlib

## Installation

1. **Clone the Repository:**

   ```bash
   git clone https://github.com/DulanjanaPerera/ValueIterationSoftRobot.git
   cd ValueIterationSoftRobot
   ```

2. **Install Dependencies:**

   ```bash
   pip install -r requirements.txt
   ```

## Usage

1. **Run the Simulation:**

   ```bash
   python main_VI.py
   ```

2. **Run Learned Policy for Visualization:**

   ```bash
   python run_policy.py
   ```

## Project Structure

- `main_VI.py`: Entry point for running the value iteration simulation.
- `run_policy.py`: Runs the learned policy to visualize the results.
- `Tetra_pybullet.py`: Creates the PyBullet-based simulation environment.
- `Rolling_gait.py`: Defines the locomotion pattern for the robot.
- `value_iteration.py`: Implements the value iteration algorithm.
- `README.md`: Project documentation.

## Simulation Scenarios

The project includes different navigation scenarios:

1. **Basic Navigation:** The robot moves from an initial position to a goal location.
2. **Multi-Step Planning:** The robot adapts its trajectory based on the planning horizon.

## Results

The value iteration algorithm effectively guides the soft robot to its target with defined steps. The results demonstrate the feasibility of using learning-based optimum planning techniques in soft robotic navigation. However, without a surrogate model, learning required significant amount of time.

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request with your proposed changes.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Acknowledgments

This project was developed as part of the MEEN 689 – Planning and Controlling of Autonomous Vehicles course at Texas A&M University. Special thanks to the instructors and peers for their guidance and support.

---

For a demonstration of the system, watch the [project video](https://youtu.be/kcocsoEcRiE).

