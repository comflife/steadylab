# SteadyLab Repository

> **WARNING:** This code is under the Apache License 2.0. Please ensure to check and comply with the license before using or distributing the code. Unauthorized copying or redistribution is strictly prohibited.

## Overview

This repository contains Python scripts related to serial communication and control of a vehicle. These scripts are designed to be used with the Robot Operating System (ROS2), and they utilize custom messages from the `steadylab` package.

### 1. Serial Node

The `SerialNode` class is responsible for managing serial communication with the connected hardware. It reads and writes data to the specified serial port, translating messages between the system and the device.
```
sudo apt-get install python3-serial
```

### 2. Serial Control

The `Serial` class provides a user interface to control speed and steering of the vehicle. The user can input commands via the terminal, and the class translates these commands into appropriate messages for the hardware.

## Requirements

- ROS 2
- Python 3
- Numpy
- PySerial
- Custom messages (`ErpRead`, `ErpWrite`) from the `steadylab` package

## Installation

1. Clone the repository.
2. Install the necessary dependencies.
3. Build the `steadylab` package.
4. Source the workspace and run the nodes.

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## Contributing

Please read [CONTRIBUTING.md](CONTRIBUTING.md) for details on our code of conduct, and the process for submitting pull requests.

## Contact

For any inquiries, please contact [okharry1@dgu.ac.kr].
