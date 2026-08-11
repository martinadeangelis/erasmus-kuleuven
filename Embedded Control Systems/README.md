# 🤖 Autonomous Docking in a Robot Recharge Room

This project implements an autonomous navigation and docking system for a service robot. The robot is designed to navigate along a wall, detect a docking bay, park forward into it, and reverse back out, simulating the behavior required for periodic, unassisted recharging.

*This repository documents both the initial mathematical design and the final implementation, highlighting how theoretical control strategies were adapted to overcome real-world sensor limitations.*

## 🎬 Video Demonstration
Watch the robot successfully executing the autonomous docking maneuver on LinkedIn:
**👉 [video link]**

## 📝 Specifications and Design Requirements
*   **Navigation:** Maximum wall-following speed of $v_{max}=0.5\ \text{m/s}$.
*   **Clearance:** Maintains a $30\,\text{cm}$ safety margin during wall-following inside the bay.
*   **Bay Dimensions:** Designed for a docking bay $40\ \text{cm}$ wide and $50\ \text{cm}$ deep.
*   **Sensors:** Utilizes a $10\ \text{Hz}$ LiDAR to process environmental data and detect geometric anchors.

## 🧠 LiDAR Processing Pipeline
To ensure robust detection without false positives, LiDAR data is processed through a rigorous pipeline:
1.  **Clustering:** Sequential LiDAR points are grouped based on a distance threshold ($d_{split} = 4\ \text{cm}$), then merged into "superclusters" representing solid obstacles like walls.
2.  **Anchor Detection:** The system looks for a "Near Anchor" (sharp wall termination) and a "Far Anchor" (L-shaped corner), verifying that the gap matches the $40\ \text{cm}$ bay width.
3.  **Validation:** Anchors are tracked and validated.

## ⚙️ Final Implementation: From Theory to Reality
The initial theoretical design (documented in the report) was significantly updated during physical testing to handle unexpected hardware and environmental constraints. The final executed FSM and control strategies are:

### Updated Finite State Machine (FSM)
1.  **Searching:** Follows the wall and scans for the bay.
2.  **Decelerating:** Stops at a fixed threshold (near-anchor $x=0.40\text{m}$) instead of using the initially planned PID+LQR braking, relying on L-shape re-validation.
3.  **Turning:** Executes an open-loop entry arc based on odometry yaw, because the LiDAR loses sight of the anchors during the turn.
4.  **Docking:** Centers itself to the back wall using purely Proportional (P-centering) control, stopping when the front LiDAR cone ($\pm5^\circ$) detects the wall at $15\text{cm}$.
5.  **Docked Pause:** Holds position for 2 seconds to simulate recharging.
6.  **Undocking & Reversing:** Performs a timed reverse out of the bay (for the exact duration of the approach) followed by an open-loop mirror arc to rejoin the corridor.
7.  **Emergency:** A global state that immediately stops the robot upon collision detection.

### Key Lessons Learned
*   **Sensor Blindspots:** Feedback control during the entry arc failed because the bay anchors left the LiDAR field of view; falling back to open-loop odometry for the turn was necessary.
*   **Control Simplification:** Complex LQR and full PID controllers were overkill at low speeds and added tuning instability; switching to simple threshold stops and Proportional-only centering yielded a much more robust system.
*   **Environmental Constraints:** A known failure case occurs if the bay is too short, leaving the robot without enough room to physically realign during the entry arc.

## 📂 Repository Contents
*   📝 **[Project Report](./Report_3.md):** The original mathematical formulation of the control laws and LiDAR pipeline.
*   📊 **[Results Presentation](./ECS_Discussion2.pdf):** The slide deck detailing the final implemented FSM and the adaptations made during physical testing.
*   📊 **[Reference Frames Diagram](./Reference_Frames.svg):** Visual representation of the coordinate systems.
*   📊 **[Initial FSM Diagram](./States.svg):** State machine flowchart from the initial design phase.
  
*(Note: Source code is omitted from this public repository due to academic privacy requirements)*.
