# Puzzlebot
This repository contains the source code for a robotic navigation system using ultrasonic sensors (HC-SR04) and implementing the A* search algorithm for pathfinding. The system is designed to autonomously navigate in an environment with obstacles, calculating and adjusting paths dynamically.

## Features

- **Ultrasonic Distance Measurement**: Utilizes HC-SR04 ultrasonic sensors for distance measurements in four directions (front, back, left, right).
- **Dynamic Pathfinding**: Implements the A* search algorithm to find the shortest path from start to goal while avoiding obstacles.
- **Real-time Obstacle Detection**: Detects and reacts to obstacles dynamically using sensor data.
- **Direction Handling**: Supports complex maneuvers by maintaining awareness of the robot's current direction and making directional decisions based on pathfinding results.

## Hardware Requirements

- Any compatible microcontroller (e.g., Arduino, STM32)
- HC-SR04 ultrasonic sensors (4x)
- Basic wiring components (jumpers)
- Power supply for the microcontroller and sensors

## Software Dependencies

- `mbed.h` - Mbed OS API for hardware abstraction
- `hcsr04.h` - Custom library for handling HC-SR04 sensors
- Standard C libraries: `stdio.h`, `math.h`, `string.h`

## Setup and Configuration

1. **Hardware Setup**:
   - Connect each HC-SR04 sensor to the microcontroller according to the defined pins in the code (`PC_10`, `PC_11`, `D12`, `D11`, `A2`, `D13`, `PB_0`, `PB_1`).
   - Ensure that the `InterruptIn` button and `RawSerial` objects are correctly configured for your microcontroller.

2. **Software Setup**:
   - Load the code onto your microcontroller using the Mbed CLI or your preferred IDE that supports Mbed OS.
   - Adjust `MAX_ROWS`, `MAX_COLS`, and sensor pin assignments as necessary to match your hardware setup.

## Usage

- Power on the system.
- The system will initialize and print the starting conditions.
- Obstacles can be predefined in the code or dynamically added based on real-time sensor data.
- The system continuously checks for the optimal path, recalculating as needed when new obstacles are detected or when manually triggered via an interrupt button.

## Functions Description

- **`initObstacles()`**: Initialize predefined obstacles.
- **`expandObstacles()`**: Adjust obstacle boundaries considering the robot's dimensions.
- **`locateCurrentPosition()`**: Estimate the robot's current position based on sensor data and known map.
- **`tangentBugAvoidance()`**: Avoidance algorithm when an immediate obstacle is detected in the path.
- **`aStarSearch()`**: Implementation of the A* search algorithm for pathfinding.

## Contributing

Contributions to this project are welcome. Please ensure that any pull requests or issues are clear and detailed to facilitate effective collaboration.
