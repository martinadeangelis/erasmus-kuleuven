# 🌡️ TCLab Temperature Control: PID vs MPC

This project focuses on the system identification and temperature control of a thermal process (TCLab heater). It features a comprehensive comparison between a baseline Proportional-Integral-Derivative (PID) controller and an advanced Custom Model Predictive Control (MPC) architecture.

The main objectives and achievements of this work include:
*   Characterizing the system and identifying a First-Order Plus Dead Time (FOPDT) model to represent the thermal dynamics.
*   Designing and tuning a baseline PID controller using the Skogestad Internal Model Control (SIMC) method for an optimal balance of speed and robustness.
*   Developing a Custom MPC algorithm from scratch to handle physical constraints, asymmetric dynamics, and transport delays.
*   Implementing an output disturbance estimator (bias updater) within the MPC to ensure zero steady-state error and robust disturbance rejection.
*   Evaluating and comparing both controllers using performance metrics like IAE (Tracking Error), ISE, ITAE, and Control TV (Actuator Wear).

## 📂 Repository Contents

Here you can find all the materials related to this project:

*   📝 **[Assignment Description](./CSA%20M2%202026.pdf):** The original requirements and goals of the milestone.
*   📊 **[Project Report](./ReportG15.pdf):** The full documentation detailing the system analysis, tuning procedures, and performance comparisons.
*   💻 **[PID Controller Code](./PID_heater.py):** The Python script implementing the SIMC-tuned PID controller.
*   🚀 **[MPC Controller Code](./MPC_heater.py):** The Python script for the custom Model Predictive Control algorithm.
