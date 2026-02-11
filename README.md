<div align="center">

# ROS 2 Pick-and-Place with React Web App

[![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![UR3e](https://img.shields.io/badge/Platform-UR3e-orange)](https://www.universal-robots.com/)
[![MoveIt 2](https://img.shields.io/badge/MoveIt-2-green)](https://moveit.ros.org/)
[![Python](https://img.shields.io/badge/API-Python-yellow)](https://www.python.org/)
[![React](https://img.shields.io/badge/Frontend-React-61DAFB)](https://react.dev/)
[![Vite](https://img.shields.io/badge/Build-Vite-646CFF)](https://vitejs.dev/)
[![License: MIT](https://img.shields.io/badge/License-MIT-red.svg)](https://opensource.org/licenses/MIT)



**NOVA RICS Open Lab**  
NOVA School of Science and Technology (FCT NOVA)  
Lisbon, Portugal

<table>
  <tr>
    <td style="vertical-align: top;">
      This repository presents a complete robotic pick-and-place system based on
      <strong>ROS 2</strong>, integrating a <strong>web-based React interface</strong>
      for intuitive human-robot interaction. It also implements a way to control the robotic cell from a base AP router wirellessly connected to the cell.
      <br><br>
      The system combines a ROS 2 backend (motion planning, execution, and robot control)
      with an API python server and a modern frontend that allows users to command and monitor the robot
      through a browser web app.
    </td>
  </tr>
</table>

</div>

---

# <div align="center">Architecture</div>

## System Overview

The system follows a modular layered architecture separating robot control, API communication, and user interaction.

It is composed of three main layers:

- **ROS 2 Backend (C++)**
- **Python API Server**
- **React + Vite Web Interface**

This separation ensures safety, scalability, and clean responsibility boundaries.

---

## ROS 2 Backend

The backend is responsible for all robot-level operations.

It integrates:

- `ur_robot_driver`
- `MoveIt 2`
- A custom ROS 2 action server (`move_piece`)
- IO control for the suction gripper

The `move_piece` action handles the full pick-and-place sequence:

1. Approach source  
2. Descend and pick  
3. Lift  
4. Transit  
5. Place  
6. Retreat  

Motion is executed using Cartesian planning (`computeCartesianPath`) with velocity scaling and collision checking enabled.

A dedicated calibration node initializes the robot to a known joint configuration before operation.



## Python API Server

The Python server acts as a bridge between the web interface and ROS 2.

It:

- Exposes HTTP endpoints  
- Sends goals to the ROS 2 action server through a spinning client node
- Receives feedback and results  
- Maintains logical board state validation  

This layer isolates web interaction from direct robot control.



## React + Vite Frontend

The frontend provides a browser-based interface for:

- Selecting source and destination slots with a drag and drop style
- Setting execution speed  
- Triggering pick-and-place operations  
- Monitoring execution progress  

It communicates with the backend via HTTP.



## Communication Flow
```
Browser React webApp
      |
      |  HTTP
      ↓
Python API Server
      |
      | ROS 2 Action
      ↓ 
C++ Action Server
      |
      | MoveIt 2
      ↓ 
UR Robot + Gripper
```
The system supports wireless operation through a dedicated access point (AP) connecting the operator device to the robotic cell network through Wifi.

---

# <div align="center">Prerequisites</div>

To run this system, the following environment and hardware are required.


## System Requirements

- **Ubuntu 22.04 LTS**
- **ROS 2 Humble**
- `colcon` build tools
- `rosdep`
- Python ≥ 3.10

Internet access is recommended for installing missing dependencies.

## ROS 2 Dependencies

- `MoveIt 2`
- `ur_robot_driver`
- `ur_calibration`
- `ur_moveit_config`
- `ur_controllers`
- `ur_description`
- `External Control URCaps` installed on the UR robot

Make sure all ROS 2 packages are properly sourced before running the system.


## Web Application Requirements

- Node.js v22
- npm
- Modern web browser (Chrome, Firefox, Edge)


## Hardware Requirements

- Universal Robots **UR3e** (tested platform)
- Compatible gripper (suction-based in this implementation)
- At least one router or access point (AP) for local wireless communication
- Operator device (PC or laptop connected to the same local network)


## Optional

- Simulation environment (Gazebo / RViz) for testing without hardware

*Simulating the robot will not be taken into account here but you can refer to the UR teams ROS 2 documentation to do it yourself.*

---

## <div align="center">Setup Guide</div>
text

---

## <div align="center">Running the System</div>
text

---

## <div align="center">Usage</div>
text

---

## <div align="center">License</div>
This repository is released under the MIT License. Please see the [LICENSE](LICENSE) file for more details.
