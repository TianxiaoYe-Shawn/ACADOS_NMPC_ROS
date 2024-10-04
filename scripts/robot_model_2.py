from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos

def robot_model():

    ##casadi definition##
    # x
    x = SX.sym('x')
    y = SX.sym('y')
    theta = SX.sym('theta')
    # u
    v = SX.sym('v')
    w = SX.sym('w')
    # x_dot
    x_dot = SX.sym('x_dot')
    y_dot = SX.sym('y_dot')
    theta_dot = SX.sym('theta_dot')
    # f
    f_expl = vertcat(v * cos(theta),
                     v * sin(theta),
                     w)
    
    ##acados definition##
    model = AcadosModel()
    model.name = 'jackal'
    model.x = vertcat(x, y, theta)
    model.u = vertcat(v, w)
    model.p = []
    model.xdot = vertcat(x_dot, 
                         y_dot, 
                         theta_dot)
    model.f_expl_expr = f_expl
    model.f_impl_expr = model.xdot - f_expl
    
    return model
