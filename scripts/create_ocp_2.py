from acados_template import AcadosOcp
from robot_model_2 import robot_model
from casadi import vertcat, sin, cos
import numpy as np
import time
import math


def create_ocp():

    # initialize 
    ocp = AcadosOcp()
    ocp.dims.N = 20
    ocp.solver_options.tf = 2
    
    # system model
    ocp.model = robot_model()
    #ocp.parameter_values = ...
    
    # solver setting
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.nlp_solver_type = 'SQP'
    ocp.solver_options.qp_solver_iter_max = 100
    ocp.solver_options.qp_solver_cond_N = 10
    ocp.solver_options.print_level = 0
    
    # cost
	## cost type
    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'
    ## set variables 
    x, y, theta  = ocp.model.x[0], ocp.model.x[1], ocp.model.x[2]
    v, w = ocp.model.u[0], ocp.model.u[1]
    ## cost weight matrix (initialize)
    QR = np.diag(np.zeros(5))
    Q = np.diag(np.zeros(3))
    ocp.cost.W = QR
    ocp.cost.W_e = Q
    ## cost ref (initialize)
    ocp.cost.yref = np.zeros((5, ))
    ocp.cost.yref_e = np.zeros((3, ))
    ## cost predict
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
    
    # constraint
    ocp.constraints.constr_type = 'BGH'#if non-linear constraint -> BGP
    #ocp.constraints.idxbx = np.array([2])
    #ocp.constraints.lbx = np.array([-np.radians(90)])
    #ocp.constraints.ubx = np.array([np.radians(90)])
    ocp.constraints.idxbu = np.array([0, 1])
    v_max = 1.0
    w_max = 0.78
    ocp.constraints.lbu = np.array([0, -w_max])
    ocp.constraints.ubu = np.array([v_max, w_max])
    x0 = np.zeros((3,))
    ocp.constraints.x0 = x0
    
    # return ocp
    return ocp
