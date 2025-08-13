from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos, fabs
import numpy as np
from scipy.linalg import block_diag

def export_wamv_model() -> AcadosModel:
    model_name = 'wamv'

    # States
    x = SX.sym('x')
    y = SX.sym('y')
    psi = SX.sym('psi')
    u = SX.sym('u')
    v = SX.sym('v')
    r = SX.sym('r')
    sym_x = vertcat(x, y, psi, u, v, r)

    # Controls
    Tp = SX.sym('Tp')
    Ts = SX.sym('Ts')
    sym_u = vertcat(Tp, Ts)

    # State derivatives
    x_dot = SX.sym('x_dot')
    y_dot = SX.sym('y_dot')
    psi_dot = SX.sym('psi_dot')
    u_dot = SX.sym('u_dot')
    v_dot = SX.sym('v_dot')
    r_dot = SX.sym('r_dot')
    sym_xdot = vertcat(x_dot, y_dot, psi_dot, u_dot, v_dot, r_dot)  # Size 6

    # parameters (new added)
    Tp_pre = SX.sym('Tp_pre')
    Ts_pre = SX.sym('Ts_pre')
    sym_p = vertcat(Tp_pre,Ts_pre)

    # # Slack variables for Δu
    # Tp_z = SX.sym('Tp_z')
    # Ts_z = SX.sym('Ts_z')
    # delta_p_z = SX.sym('delta_p_z')
    # delta_s_z = SX.sym('delta_s_z')
    # sym_z = vertcat(Tp_z,Ts_z,delta_p_z,delta_s_z)


    # System parameters
    m = 180
    Izz = 446
    # LCG = 2.373776
    B = 2.05427
    added_mass = np.array([0, 0, 0, 0, 0, 0])
    M = np.diag([m + added_mass[0], m + added_mass[1], Izz + added_mass[5]])
    M_inv = np.linalg.inv(M)
    xu, xuu = -100, -150
    yv, yvv = -100, -100
    nr, nrr = -800, -800

    # Thrust allocation
    Tx = Tp + Ts
    Ty = 0
    Mz = - B/2 * Tp + B/2 * Ts

    # Dynamics
    du = M_inv[0, 0] * (Tx + m * v * r + xu * u + xuu * fabs(u) * u)
    dv = M_inv[1, 1] * (Ty - m * u * r + yv * v + yvv * fabs(v) * v)
    dr = M_inv[2, 2] * (Mz + nr * r + nrr * fabs(r) * r)
    dx = cos(psi) * u - sin(psi) * v
    dy = sin(psi) * u + cos(psi) * v
    dpsi = r

    f_expl = vertcat(dx, dy, dpsi, du, dv, dr)  # Size 6
    f_impl = sym_xdot - f_expl  # Size 6
    # f_impl_z = sym_z - (sym_u - sym_p)  # Size 4, algebraic z = 0 (enforced via cost)
    # f_impl = vertcat(f_impl_x, f_impl_z)  # Size 10

    # constraints
    h_expr = sym_u

    # Cost
    cost_y_expr = vertcat(sym_x, sym_u)  

    model = AcadosModel()
    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = sym_x
    model.xdot = sym_xdot  # Size 6
    model.u = sym_u
    # model.z = sym_z
    model.p = sym_p
    model.cost_y_expr = cost_y_expr
    model.cost_y_expr_e = sym_x
    model.name = model_name

    return model