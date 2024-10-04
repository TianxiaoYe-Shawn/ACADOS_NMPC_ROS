from acados_template import AcadosOcp, AcadosOcpSolver
from robot_model import export_robot_model
from casadi import vertcat, sin, cos
import numpy as np
import scipy.linalg
import time
import math

X0 = np.zeros((3,))  # Intital state
v_max = 5.0 
w_max = 3.0 # Define the max input allowed
T_horizon = 2.0  # Define the prediction horizon

def Solver() -> AcadosOcpSolver:
    N_horizon = 20  # Define the number of discretization steps

    # create ocp object to formulate the OCP
    ocp = AcadosOcp()

    model = export_robot_model()
    ocp.model = model
    nx = model.x.rows()
    nu = model.u.rows()

    # set dimensions
    ocp.dims.N = N_horizon

    # set cost
    Q_mat = np.diag([1e2, 1e2, 5e-1])  # [x,y,th]
    R_mat = np.diag([1e-1, 1e-2])

    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"

    ny = nx + nu
    ny_e = nx

    ocp.cost.W_e = Q_mat
    ocp.cost.W = scipy.linalg.block_diag(Q_mat, R_mat)

    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)

    Vu = np.zeros((ny, nu))
    Vu[nx : nx + nu, 0:nu] = np.eye(nu)
    ocp.cost.Vu = Vu

    ocp.cost.Vx_e = np.eye(nx)

    ocp.cost.yref = np.zeros((ny,))
    ocp.cost.yref_e = np.zeros((ny_e,))

    ## set variables 
    x, y, theta  = ocp.model.x[0], ocp.model.x[1], ocp.model.x[2]
    v, w = ocp.model.u[0], ocp.model.u[1]
    ## cost presentation
    ocp.model.cost_y_expr = vertcat(
                                    x,
                                    y,
                                    theta,
                                    v,
                                    w
                                    )
    ocp.model.cost_y_expr_e = vertcat(
                                     x,
                                     y,
                                     theta
                                     )
                                     
    # set constraints
    ocp.constraints.constr_type = 'BGH'
    ocp.constraints.idxbu = np.array([0,1])
    ocp.constraints.lbu = np.array([0,-w_max])
    ocp.constraints.ubu = np.array([+v_max,+w_max])


    ocp.constraints.x0 = X0
    
    # set prediction horizon
    ocp.solver_options.tf = T_horizon
    
    # set options
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"  # FULL_CONDENSING_QPOASES
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP"
    
    ocp.solver_options.qp_solver_cond_N = 10
    ocp.solver_options.qp_solver_iter_max = 100
    ocp.solver_options.print_level = 0
    
    acados_solver = AcadosOcpSolver(ocp, json_file="acados_solver.json")

    return acados_solver

Solver()
print("Acados solver for MPC problem is ready")
