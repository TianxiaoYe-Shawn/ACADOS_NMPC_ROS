from acados_template import AcadosOcpSolver
from create_ocp_2 import create_ocp
import numpy as np
import scipy.linalg
import matplotlib.pyplot as plt

def solver():
    # initialize solver
    ocp = create_ocp()
    N = ocp.dims.N
    solver = AcadosOcpSolver(ocp)
    
    # initialize ocp
    x = np.zeros((N + 1, 3))
    u = np.zeros((N + 1, 2))
    yref = np.zeros((N + 1, 5))
    for i in range(N):
        solver.cost_set(i, "yref", yref[i])
    solver.cost_set(N, "yref", yref[N][:3])
    for i in range(N + 1):
        solver.set(i, 'x', np.zeros(3))
    x0 = np.zeros((3,))
    solver.constraints_set(0, "lbx", x0)
    solver.constraints_set(0, "ubx", x0)
    solver.solve()
    
    # set cost weight
    W1 = 2*np.diag([1e3, 1e3, 1e-3, 5e-1, 5e-2])
    W2 = 2*np.diag([1e3, 1e3, 1e-3])
    for i in range(N):
        solver.cost_set(i, 'W', W1)
    solver.cost_set(N, 'W', W2)
    
    # set ref
    '''#half ctrcle
    angles = np.linspace(0, np.pi, N + 1)
    radius = 0.3
    x_pts = radius * np.cos(angles) 
    y_pts = radius * np.sin(angles)
    '''
    
    '''#zero
    x_pts = np.zeros(N + 1)
    y_pts = np.zeros(N + 1)
    theta_pts = np.zeros(N + 1)
    '''
    
    #line
    x_pts = np.linspace(0, 0.6, N + 1)
    y_pts = np.linspace(0, 0.6, N + 1)
    
    
    yref[:, 0] = x_pts
    yref[:, 1] = y_pts
    #yref[:, 2] = theta_pts
    for i in range(N):
        solver.cost_set(i, "yref", yref[i])
    solver.cost_set(N, "yref", yref[N][:3])
    
    # set x0
    x0 = np.array([yref[0, 0], yref[0, 1], np.arctan2(yref[1, 1]-yref[0, 1], yref[1, 0]-yref[0, 0])])
    solver.constraints_set(0, "lbx", x0)
    solver.constraints_set(0, "ubx", x0)
    
    # solve
    solution_status = solver.solve()
    
    # print results
    for i in range(N):
        print(solver.get(i, 'u'))
    
        # Extract trajectory
    actual_x = []
    actual_y = []
    for i in range(N + 1):
        state = solver.get(i, 'x')
        actual_x.append(state[0])
        actual_y.append(state[1])
    
    # Plotting the reference and actual trajectory
    plt.figure(figsize=(8, 6))
    plt.plot(x_pts, y_pts, 'ro')
    plt.plot(x_pts, y_pts, 'r--', label='Reference Trajectory')
    plt.plot(actual_x, actual_y, 'bo')
    plt.plot(actual_x, actual_y, 'b-', label='Actual Trajectory')
    plt.title('Trajectory Comparison')
    plt.xlabel('X position')
    plt.ylabel('Y position')
    plt.legend()
    plt.grid(True)
    plt.show()
    
    return solution_status

print(solver())
    
