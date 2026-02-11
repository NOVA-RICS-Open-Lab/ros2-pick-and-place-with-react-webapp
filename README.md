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

## <div align="center">Architecture</div>

# General

The architecture is divided into three main layers:

- **ROS 2 Backend**
  - Robot drivers and controllers
  - Pick-and-place logic
  - ROS 2 action server
  - MoveIt2 motion planning and execution
- **API Python Server Layer**
  - HTTP interface bridging the web frontend
  - ROS 2 action client bridging ROS and the API sever
  - Hard coded board state truth checking and logic
- **React + Vite Frontend**
  - Browser-based UI for commanding pick-and-place operations
  - Visualization of task state and system feedback

This separation ensures modularity, scalability, and ease of deployment.

---

## <div align="center">Dependencies</div>

### Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- Node.js ≥ 20
- `colcon`, `rosdep`
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
