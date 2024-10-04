from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos

def export_robot_model() -> AcadosModel:
    model_name = "jackal"

    # set up states & controls
    x = SX.sym("x")
    y = SX.sym("y")
    theta = SX.sym("theta")

    x = vertcat(x, y, theta)

    v = SX.sym("v")
    w = SX.sym("w")
    u = vertcat(v, w)

    # xdot
    x_dot = SX.sym("x_dot")
    y_dot = SX.sym("y_dot")
    theta_dot = SX.sym("theta_dot")

    xdot = vertcat(x_dot, y_dot, theta_dot)

    # dynamics
    f_expl = vertcat(v * cos(theta), v * sin(theta), w)

    f_impl = xdot - f_expl

    # algebraic variables
    z = []
    
    # parameters
    p = []
    
    # dynamics
    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    model.z = z
    model.p = p
    model.name = model_name

    model.t_label = "$t$ [s]"
    model.x_labels = ["$x$", "$y$", "$\\theta$"]
    model.u_labels = ["$v$", "$w$"]

    return model
