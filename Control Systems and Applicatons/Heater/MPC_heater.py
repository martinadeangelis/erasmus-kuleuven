import tclab
import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize


# =============================================================================
#  PLANT MODEL PARAMETERS
# =============================================================================
K_sys     = 0.6596   
TAU_SYS   = 125.1874 
THETA_SYS = 12.71

# =============================================================================
#  MPC TUNING PARAMETERS
# =============================================================================
PREDICTION_HORIZON = 120 #80/120  # Np: number of future steps the controller looks ahead.
                          # Increase for sluggish/slow systems, decrease to speed up computation.

CONTROL_HORIZON    = 25 #5/25  # Nc: number of free moves the optimizer calculates.
                          # Increase for more aggressive control, decrease for smoother, more conservative control.

LAMBDA             = 0.2 #0.5/0.2  # Move-suppression weight (penalises ΔQ between steps).
                          # Increase if heater chatters or overshoots, decrease if response is too slow.

INTEGRAL           = 20.0  # Integral tracking error weight.

# =============================================================================
#  EXPERIMENT SETTINGS
# =============================================================================
RUN_MINUTES = 15          # Total experiment duration [minutes].

SAMPLE_TIME = 1.0         # Control loop sample period [s].

SETPOINT_SCHEDULE = [
    (  0, 26.25),
    (225, 70),
    (450, 26.25),
    (675, 58.75),
]

# =============================================================================
#  OPTIMIZER SETTINGS
# =============================================================================
OPTIMIZER_METHOD  = 'SLSQP'  # SciPy method. SLSQP handles bounds well.
OPTIMIZER_FTOL    = 1e-3     # Function-value tolerance. Relax (1e-1) to solve faster at the cost of slight sub-optimality.
Q_BOUNDS          = (0, 100) # Physical heater limits [%].

# =============================================================================
#  HELPER FUNCTIONS
# =============================================================================

def get_setpoint(t: float) -> float: # Return the desired temperature [°C] at time t [s].
    sp = SETPOINT_SCHEDULE[0][1]
    for t_break, sp_val in SETPOINT_SCHEDULE:
        if t >= t_break:
            sp = sp_val
    return sp


def compute_future_setpoints(t: float, n_steps: int) -> list: # Return a list of setpoints for the next n_steps seconds.
    FORESIGHT_SECONDS = 15  # looks at 15 seconds in the future
    
    sp_list = []
    for i in range(n_steps):
        if i <= FORESIGHT_SECONDS:
            # real valie
            sp_list.append(get_setpoint(t + i))
        else:
            # believes that it will stay constant
            sp_list.append(get_setpoint(t + FORESIGHT_SECONDS))
            
    return sp_list

def mpc_cost(Q_future, T_model_curr, T_ambient, SP_future, Q_past_buffer, Q_prev, bias): # Objective function minimised by SciPy at every control step.
    
    # Extend control moves: hold last calculated move for the rest of Np
    Q_ext = np.concatenate((Q_future, [Q_future[-1]] * (PREDICTION_HORIZON - CONTROL_HORIZON)))

    # Full heater sequence seen by the model: past (in delay pipe) + future moves
    Q_full = np.concatenate((Q_past_buffer, Q_ext))

    # Discrete-time FOPDT pole
    alpha = np.exp(-SAMPLE_TIME / TAU_SYS)

    cost_y = 0.0
    T_pred = T_model_curr
    delay = len(Q_past_buffer)

    # tracking error
    for i in range(PREDICTION_HORIZON):
        T_pred = alpha * T_pred + (1 - alpha) * (T_ambient + K_sys * Q_full[i])
        T_corrected = T_pred + bias

        # penalizing only after dead time
        if i >= delay:
            cost_y += INTEGRAL* (T_corrected - SP_future[i]) ** 2

    # penalty on rapid changes
    cost_u = 0.0
    for j in range(CONTROL_HORIZON):
        if j == 0:
            delta_Q = Q_future[0] - Q_prev
        else:
            delta_Q = Q_future[j] - Q_future[j - 1]
        cost_u += LAMBDA * (delta_Q ** 2)

    return cost_y + cost_u

# =============================================================================
#  LIVE PLOT SETUP
# =============================================================================
def init_plots(total_time: float):
    plt.ion()
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    (line_sp,) = ax1.plot([], [], 'k-',  linewidth=2, label='Setpoint')
    (line_t1,) = ax1.plot([], [], 'r-',  linewidth=2, label='T1 (MPC)')
    ax1.set_ylabel('Temperature [°C]')
    ax1.set_title('Setpoint Tracking Comparison')
    ax1.set_xlim(0, total_time)
    ax1.set_ylim(20, 65)
    ax1.legend(loc='upper left')
    ax1.grid(True, linestyle='--', alpha=0.7)

    (line_q1,) = ax2.plot([], [], 'r-', linewidth=2, label='Q1 (MPC Effort)')
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

def run_mpc():
    total_time  = RUN_MINUTES * 60.0
    delay_steps = int(max(1, round(THETA_SYS / SAMPLE_TIME)))
    bounds      = [Q_BOUNDS] * CONTROL_HORIZON

    fig, line_sp, line_t1, line_q1 = init_plots(total_time)

    tm, T1_data, Q1_data, SP_data = [], [], [], []

    try:
        lab = tclab.TCLab()
        print("TCLab connected.  Starting MPC ...")
        lab.LED(100)

        # ---- Initialise state ----
        T_ambient    = lab.T1                     # Use first reading as ambient
        Q_past       = [0.0] * delay_steps        # Dead-time buffer (past heater values)
        Q_prev       = 0.0                         # Last applied heater output
        Q_guess      = [0.0] * CONTROL_HORIZON    # Warm-start for the optimizer
        T_model   = T_ambient

        start_time = time.time()

        while True:
            loop_start = time.time()
            t          = loop_start - start_time

            if t > total_time:
                break

            # --- Measurement ---
            T1 = lab.T1

            # --- Bias ---
            bias = T1 - T_model

            # --- Prediction ---
            SP_future = compute_future_setpoints(t, PREDICTION_HORIZON)
            
            # --- Optimisation ---
            result = minimize(
                mpc_cost,
                Q_guess,
                args=(T_model, T_ambient, SP_future, Q_past, Q_prev, bias),
                bounds=bounds,
                method=OPTIMIZER_METHOD,
                options={'ftol': OPTIMIZER_FTOL},
            )

            # --- Apply only the first optimal move (receding-horizon principle) ---
            Q1 = float(np.clip(result.x[0], Q_BOUNDS[0], Q_BOUNDS[1]))
            lab.Q1(Q1)
            lab.Q2(0)

            alpha = np.exp(-SAMPLE_TIME / TAU_SYS)
            T_model = alpha * T_model + (1 - alpha) * (T_ambient + K_sys * Q_past[0])

            # --- Update dead-time buffer ---
            Q_past.append(Q1)
            Q_past.pop(0)   # Keep buffer length == delay_steps

            # --- Warm-start next iteration (shift solution by one step) ---
            Q_guess = np.append(result.x[1:], result.x[-1])
            Q_prev  = Q1

            # --- Log data ---
            tm.append(t)
            T1_data.append(T1)
            Q1_data.append(Q1)
            SP_data.append(SP_future[0])

            # --- Live plot ---
            update_plots(fig, line_sp, line_t1, line_q1, tm, SP_data, T1_data, Q1_data)

            if int(t) % 20 == 0:
                print('   Time |     Q1 |     T1 |     SP |   Bias')
            print(f"{t:7.1f} | {Q1:6.2f} | {T1:6.2f} | {SP_future[0]:6.2f} | {bias:6.2f}")

            sleep_time = SAMPLE_TIME - (time.time() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        if 'lab' in locals():
            lab.Q1(0)
            lab.Q2(0)
            lab.LED(0)
            lab.close()

            # Save CSV
            if len(tm) > 0:
                data = np.column_stack((tm, SP_data, T1_data, Q1_data))
                np.savetxt("MPC_results.csv", data, delimiter=",", header="Time,SP,T1,Q1", comments="")

            # Save plot
            plt.ioff()
            plt.savefig("MPC_results.png", dpi=150)
            print("Data saved.")
            plt.show()
        else:
            print("Nothing to save.")


# =============================================================================
#  ENTRY POINT
# =============================================================================
if __name__ == "__main__":
    run_mpc()