# 🚗 Autonomous Navigation for Differential-Drive and Bicycle Robots

This project implements a complete autonomous navigation stack for mobile robots operating in a planar, obstacle-cluttered arena. The work focuses on solving three core robotic tasks: motion planning, trajectory tracking, and state estimation, culminating in the successful navigation of both differential-drive and bicycle (car-like) models.

The main objectives and achievements of this work include:
*   **Motion Planning:** Developing a three-tier planner consisting of an optimization-based local planner (CasADi/IPOPT), an analytic collision-checking routine using a capsule abstraction for obstacles, and a Probabilistic Roadmap (PRM) global planner using A* search.
*   **Trajectory Tracking (Unicycle):** Designing a nonlinear feedback controller with a pure-pursuit-style lookahead to track the planned reference trajectory, including a dedicated "endgame" phase to settle the non-holonomic robot on an exact final pose.
*   **State Estimation:** Implementing an Extended Kalman Filter (EKF) to fuse noisy GPS position measurements with known control inputs, effectively estimating the robot's full pose (including the unmeasured orientation).
*   **Bicycle Model Adaptation:** Modifying the control architecture for a car-like kinematics, decoupling the steering angle from the velocity and reformulating the lookahead logic to make reference selection independent of the planning time step.

## 📂 Repository Contents

Here you can find all the materials related to this project:

*   📝 **[Project Description](./project_description.pdf):** The original requirements and goals of the assignment.
*   📊 **[Project Report](./Project3_MDADG.pdf):** The full documentation detailing the design decisions, mathematical models, and validation of the navigation stack.
*   💻 **[Differential-Drive Code](./mobile_robot_1-3_MDADG.ipynb):** The Jupyter Notebook containing the implementation of the planner, controller, and EKF for the unicycle model.
*   🚲 **[Bicycle Model Code](./bicycle_MDADG.ipynb):** The Jupyter Notebook adapting the trajectory-tracking controller for the car-like robot.
*   🛠️ **[DiffDrive URDF](./diffdrive.urdf):** The Unified Robot Description Format file for the differential-drive robot simulation.
*   🛠️ **[Bicycle URDF](./diffdrive_bicycle.urdf):** The URDF file for the bicycle robot simulation.
