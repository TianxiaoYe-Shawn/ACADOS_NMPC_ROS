from acados_template import AcadosOcpSolver, AcadosSimSolver
from create_ocp import create_ocp_solver_description
import numpy as np
import scipy.linalg
import matplotlib.pyplot as plt
from utils import plot_robot

X0 = np.array([-1.0, 5.0, 0.00629628])  # Intital state
v_max = 10  
w_max = 3 # Define the max input allowed
T_horizon = 2.0  # Define the prediction horizon

def generate_eight_trajectory(Nsim, a, b, A, B, delta, dt):
    traj = np.zeros((Nsim, 5))  # [x, y, theta, v, w]

    # Time vector
    t = np.linspace(0, 2 * np.pi, Nsim)

    # 8-shaped curve (Lissajous curve)
    x_values = A * np.sin(a * t)
    y_values = B * np.sin(b * t + delta)

    # Calculate theta (orientation) between consecutive points
    for i in range(Nsim):
        dx = x_values[(i+1)%Nsim] - x_values[i]
        dy = y_values[(i+1)%Nsim] - y_values[i]
        traj[i, 2] = np.arctan2(dy, dx)  # theta

        # Assign x, y, and calculate velocities v and w
        traj[i, 0] = x_values[i]
        traj[i, 1] = y_values[i]
        traj[i, 3] = np.sqrt(dx**2 + dy**2) / dt  # v
        traj[i, 4] = (traj[(i+1)%Nsim, 2] - traj[i, 2]) / dt  # w
	
    print('x0 ref is:',traj[0])
    return traj


def closed_loop_simulation():

    # create solvers
    ocp = create_ocp_solver_description()
    model = ocp.model
    acados_ocp_solver = AcadosOcpSolver(ocp)
    acados_integrator = AcadosSimSolver(ocp)

    N_horizon = acados_ocp_solver.N

    # prepare simulation
    Nsim = 300
    nx = ocp.model.x.rows()
    nu = ocp.model.u.rows()

    simX = np.zeros((Nsim + 1, nx))
    simU = np.zeros((Nsim, nu))

    xcurrent = X0
    simX[0, :] = xcurrent

    # initialize solver
    for stage in range(N_horizon + 1):
        acados_ocp_solver.set(stage, "x", 0.0 * np.ones(xcurrent.shape))
    for stage in range(N_horizon):
        acados_ocp_solver.set(stage, "u", np.zeros((nu,)))

    # get trajectory for setting yref
    a = 2  # Frequency for x-component
    b = 1  # Frequency for y-component
    A = 2  # Amplitude for x-component
    B = 4  # Amplitude for y-component
    delta = np.pi / 2  # Phase difference
    dt = T_horizon / N_horizon  # Time step

    traj = generate_eight_trajectory(Nsim, a, b, A, B, delta, dt)
    
    # closed loop
    for i in range(Nsim):
        # update yref
        for j in range(N_horizon):
            acados_ocp_solver.set(j, "yref", traj[(i+j)%(Nsim)])
        acados_ocp_solver.set(N_horizon, "yref", traj[(i+j)%(Nsim),:3])

        # solve ocp
        simU[i, :] = acados_ocp_solver.solve_for_x0(xcurrent)
        status = acados_ocp_solver.get_status()

        if status not in [0]:
            acados_ocp_solver.print_statistics()
            plot_robot(
                np.linspace(0, T_horizon / N_horizon * i, i + 1),
                [v_max, w_max],
                simU[:i, :],
                simX[: i + 1, :],
            )
            raise Exception(
                f"acados acados_ocp_solver returned status {status} in closed loop instance {i} with {xcurrent}"
            )

        # simulate system
        xcurrent = acados_integrator.simulate(xcurrent, simU[i, :])
        simX[i + 1, :] = xcurrent

    # plot results
    plot_robot(
        np.linspace(0, T_horizon / N_horizon * Nsim, Nsim + 1), [v_max, w_max], simU, simX,
        x_labels=model.x_labels, u_labels=model.u_labels, time_label=model.t_label
    )
    
    # plot traj
    plt.figure()
    plt.plot(traj[:, 0], traj[:, 1], 'r--', label='Reference Trajectory')
    plt.plot(simX[:, 0], simX[:, 1], 'b-', label='Actual Trajectory')
    points = plt.scatter(simX[:, 0], simX[:, 1], c=range(len(simX[:, 0])), cmap='viridis', label='Actual Trajectory Points')
    plt.colorbar(points)
    plt.legend()
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('Comparison of Desired and Actual Trajectories')
    plt.grid(True)
    plt.axis('equal')  # Ensure equal scaling of x and y axes
    plt.show()
    
if __name__ == "__main__":
    closed_loop_simulation()
