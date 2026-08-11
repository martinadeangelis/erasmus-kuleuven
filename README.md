# 🌍 KU Leuven - Erasmus+ Projects

This space contains the assignments, reports, and code I developed during my **Erasmus+ study exchange at KU Leuven** (Belgium). My coursework heavily focused on **Robotics, Mechatronics and Control Systems**, allowing me to bridge mathematical modeling with practical algorithm implementation and hardware design.

Below you can find an index of the main projects included in this repository. Each project has its own dedicated folder containing the source code, the full documentation, and a specific README with further technical details.

---

## 📂 Projects Index

### 1. [🤖 Autonomous Navigation for Mobile Robots](./Robotics%20-%20Mobile%20Robot)
*Robotics*
*   Developed a complete autonomous navigation stack for both differential-drive and bicycle (car-like) robots in a planar, obstacle-cluttered arena.
*   Implemented a three-tier motion planner (CasADi/IPOPT local planner, analytical collision checking, and PRM global planner with A*).
*   Designed an Extended Kalman Filter (EKF) to fuse noisy GPS data with known control inputs for state estimation.

### 2. [🔌 Autonomous Docking in a Robot Recharge Room](./Embedded%20Control%20Systems)
*Embedded Control Systems*
*   Designed an autonomous wall-following and docking system for a service robot requiring periodic, unassisted recharging.
*   Developed a rigorous LiDAR processing pipeline (clustering, superclusters, and geometric anchor validation).
*   Adapted theoretical LQR and PID control laws into a robust, real-world Finite State Machine (FSM) utilizing threshold stops and open-loop odometry to overcome sensor blindspots.

### 3. [🌡️ TCLab Temperature Control: PID vs MPC](./Control%20Systems%20and%20Applications/Heater)
*Control Systems and Applications*
*   Characterized the thermal dynamics of a TCLab heater, identifying a First-Order Plus Dead Time (FOPDT) model.
*   Designed and tuned a baseline PID controller using the Skogestad Internal Model Control (SIMC) method.
*   Developed a custom Model Predictive Control (MPC) algorithm from scratch (using `scipy.optimize`) with a bias updater for robust disturbance rejection.

### 4. [🚢 Ship Motion Analysis](./Control%20Systems%20and%20Applications/Ship%20Motion)
*Control Systems and Applications*
*   Modeled the planar motion of a ship in 3 DoF using the non-linear Fossen maneuvering model.
*   Simulated the system's ODEs, linearized the model for state-space analysis, and added time-variant environmental disturbances.

### 5. [💉 MEMS Microneedles for Diabetes Management](./Micro-Electromechanical%20Systems%20-%20Diabetes)
*Micro-Electromechanical Systems*
*   Conducted a deep technical analysis of the evolution of the "artificial pancreas", focusing on microneedle arrays for continuous glucose monitoring (CGM) and insulin delivery.
*   Compared clean-room subtractive micromachining (photolithography, Bosch DRIE) with additive electrochemical deposition processes for soft, skin-conformal biomedical devices.

---

*(Note: Click on the project titles to open their respective folders and read the full documentation.)*
