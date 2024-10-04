from acados_template import AcadosOcp
from robot_model import export_robot_model
import numpy as np
import scipy.linalg

X0 = np.array([-2.0, 5.0, -0.00629628])  # Intital state
v_max = 10  
w_max = 3 # Define the max input allowed
T_horizon = 2.0  # Define the prediction horizon

def create_ocp_solver_description() -> AcadosOcp:
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

    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"

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

    # set constraints
    ocp.constraints.lbu = np.array([0,-w_max])
    ocp.constraints.ubu = np.array([+v_max,+w_max])
    ocp.constraints.idxbu = np.array([0,1])

    ocp.constraints.x0 = X0
    
    # set prediction horizon
    ocp.solver_options.tf = T_horizon
    
    # set options
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"  # FULL_CONDENSING_QPOASES
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP"
    
    ocp.solver_options.qp_solver_cond_N = 10
    ocp.solver_options.levenberg_marquardt = 1e-3
    ocp.solver_options.nlp_solver_max_iter = 10
    ocp.solver_options.qp_solver_iter_max = 100
    ocp.solver_options.nlp_solver_tol_stat = 1e-1
    ocp.solver_options.nlp_solver_tol_eq = 1e-1
    ocp.solver_options.print_level = 0
    
    return ocp
