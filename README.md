<div align="center">

# ROS 2 Pick-and-Place with React Web App

[![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Frontend: React](https://img.shields.io/badge/Frontend-React-blue)](https://react.dev/)
[![Build: Vite](https://img.shields.io/badge/Build-Vite-purple)](https://vitejs.dev/)
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
      through a browser web app, enabling rapid prototyping of industrial and educational
      automation cells.
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

To build this sytem you should have **at least** the following:

- Ubuntu 22.04
- ROS 2 Humble
- Node.js ≥ 22
- A supported robot or simulation environment (this was only tested with a real ur3e)

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
