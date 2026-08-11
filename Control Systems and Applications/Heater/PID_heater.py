import numpy as np
import matplotlib.pyplot as plt
import tclab
import time

# =============================================================================
#  PLANT MODEL PARAMETERS
# =============================================================================
K_sys     = 0.6596   
TAU_SYS   = 125.1874 
THETA_SYS = 12.71

# =============================================================================
#  SIMC TUNING PARAMETERS (From Skogestad Paper)
# =============================================================================
# Closed-loop time constant parameter (tau_c = theta for tight control)
tau_c = 1 * THETA_SYS  

# Analytical SIMC tuning rules for FOPDT model
Kc   = (1.0 / K_sys) * (TAU_SYS / (tau_c + THETA_SYS))
tauI = min(TAU_SYS, 4.0 * (tau_c + THETA_SYS))
tauD = THETA_SYS / 2.0

print(f"SIMC PID Parameters calculated -> Kc: {Kc:.3f}, tauI: {tauI:.3f}, tauD: {tauD:.3f}")

# =============================================================================
#  EXPERIMENT SETTINGS
# =============================================================================
n = 900  # Total time steps (15 minutes)
SAMPLE_TIME = 1.0  # Sampling time in seconds

# Define setpoint schedule profile
SP1 = np.ones(n) * 45.0
SP1[180:360] = 35.0
SP1[360:600] = 50.0
SP1[600:]    = 25.0

# =============================================================================
#  HELPER PID CONTROLLER FUNCTION
# =============================================================================
def pid(sp, pv, pv_last, ierr, dt):
    # Parameters in terms of PID coefficients
    KP = Kc
    KI = Kc / tauI
    KD = Kc * tauD
    
    # ubias for controller (initial heater)
    op0 = 0
    
    # upper and lower bounds on heater level
    ophi = 100
    oplo = 0
    
    # calculate the error
    error = sp - pv
    
    # calculate the integral error
    ierr = ierr + KI * error * dt
    
    # calculate the measurement derivative (prevents derivative kick on SP changes)
    dpv = (pv - pv_last) / dt
    
    # calculate the PID output terms
    P = KP * error
    I = ierr
    D = -KD * dpv
    op = op0 + P + I + D
    
    # implement anti-reset windup conditional clamping
    if op < oplo or op > ophi:
        I = I - KI * error * dt
        # clip output to operational actuator bounds
        op = max(oplo, min(ophi, op))
        
    # return the controller output and PID terms
    return [op, P, I, D]

# =============================================================================
#  LIVE PLOT SETUP
# =============================================================================
def init_plots(total_time: float):
    plt.ion()
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    (line_sp,) = ax1.plot([], [], 'k-', linewidth=2, label='Setpoint')
    (line_t1,) = ax1.plot([], [], 'r-', linewidth=2, label='T1 (PID)')
    ax1.set_ylabel('Temperature [°C]')
    ax1.set_title('Setpoint Tracking Comparison')
    ax1.set_xlim(0, total_time)
    ax1.set_ylim(20, 65)
    ax1.legend(loc='upper left')
    ax1.grid(True, linestyle='--', alpha=0.7)

    (line_q1,) = ax2.plot([], [], 'r-', linewidth=2, label='Q1 (PID Effort)')
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Heater [%]')
    ax2.set_title('Control Effort')
    ax2.set_xlim(0, total_time)
    ax2.set_ylim(-5, 105)
    ax2.legend(loc='upper left')
    ax2.grid(True, linestyle='--', alpha=0.7)

    plt.tight_layout()
    return fig, line_sp, line_t1, line_q1

def update_plots(fig, line_sp, line_t1, line_q1, tm, SP_data, T1_data, Q1_data):
    line_sp.set_data(tm, SP_data)
    line_t1.set_data(tm, T1_data)
    line_q1.set_data(tm, Q1_data)
    fig.canvas.draw()
    fig.canvas.flush_events()

# =============================================================================
#  MAIN CONTROL LOOP
# =============================================================================
T1_data = []
Q1_data = []
SP_data = []
tm_data = []

fig, line_sp, line_t1, line_q1 = init_plots(n - 1)

# Initialize historical state variables
ierr = 0.0
T1_last = None

try:
    lab = tclab.TCLab()
    print("TCLab connected. Starting PID execution loop...")
    lab.LED(100)

    for i in range(n):
        loop_start = time.time()

        T1 = lab.T1  # Read current temperature measurement
        
        # Initialize T1_last on the very first execution step
        if T1_last is None:
            T1_last = T1

        # Execute custom PID controller routine
        op, P, I, D = pid(SP1[i], T1, T1_last, ierr, SAMPLE_TIME)
        
        # Update loop tracking integral error based on anti-windup filter output
        ierr = I
        Q1 = op

        # Apply calculated power value to physical heater
        lab.Q1(Q1)
        lab.Q2(0)

        # Log system data arrays
        tm_data.append(float(i))
        T1_data.append(T1)
        Q1_data.append(Q1)
        SP_data.append(SP1[i])

        # Render live visualization updates
        update_plots(fig, line_sp, line_t1, line_q1, tm_data, SP_data, T1_data, Q1_data)

        if i % 20 == 0:
            print(' Heater(%),  Temp(°C),  Setpoint(°C)')
        print(f'{Q1:10.2f},{T1:10.2f},{SP1[i]:13.2f}')

        # Save current state for next iteration derivative term
        T1_last = T1
        
        # Keep sampling frequency synchronized perfectly to 1.0Hz
        sleep_time = SAMPLE_TIME - (time.time() - loop_start)
        if sleep_time > 0:
            time.sleep(sleep_time)

except KeyboardInterrupt:
    print("\nStopped by user session interrupt.")

finally:
    if 'lab' in locals():
        # Ensure hardware safety shutdown protocols are met
        lab.Q1(0)
        lab.Q2(0)
        lab.LED(0)
        lab.close()

        # Save experimental data output to standardized CSV format
        if len(tm_data) > 0:
            data = np.column_stack((tm_data, SP_data, T1_data, Q1_data))
            np.savetxt("PID_results.csv", data, delimiter=",", header="Time,SP,T1,Q1", comments="")

        # Finalize and export clean static chart for report reference
        plt.ioff()
        plt.savefig('PID_results.png', dpi=150)
        print("Data traces successfully saved.")
        plt.show()
    else:
        print("No active hardware instance detected to save.")