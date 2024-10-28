from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos, exp, fmax
import numpy as np
from scipy.linalg import block_diag
import math

def export_wamv_model() -> AcadosModel:

    model_name = 'wamv'

    # states
    x = SX.sym('x')                 # inertial position x
    y = SX.sym('y')                 # inertial position y
    # z = SX.sym('z')                 # inertial position z
    # phi = SX.sym('phi')             # roll angle
    # theta = SX.sym('theta')         # pitch angle
    psi = SX.sym('psi')             # yaw angle
    u = SX.sym('u')                 # body velocity u
    v = SX.sym('v')                 # body velocity v
    # w = SX.sym('w')                 # body velocity w
    # p = SX.sym('p')                 # roll velocity
    # q = SX.sym('q')                 # pitch velocity
    r = SX.sym('r')                 # yaw velocity
    sym_x = vertcat(x,y,psi,u,v,r)

    # controls inputs
    Tp = SX.sym('Tp')           # port thrust
    Ts = SX.sym('Ts')           # starboard thrust
    delta_p = SX.sym('delta_p')          # port angle
    delta_s = SX.sym('delta_s')          # starboard angle
    sym_u = vertcat(Tp,Ts,delta_p,delta_s)

    # # parameters
    # glf_A = SX.sym('glf_A')
    # glf_K = SX.sym('glf_K')
    # glf_B = SX.sym('glf_B')
    # glf_v = SX.sym('glf_v')
    # glf_C = SX.sym('glf_C')
    # glf_M = SX.sym('glf_M')
    # sym_p = vertcat(glf_A,glf_K,glf_B,glf_v,glf_C,glf_M)

    # xdot for f_impl
    x_dot = SX.sym('x_dot')
    y_dot = SX.sym('y_dot')
    # z_dot = SX.sym('z_dot')
    # phi_dot = SX.sym('phi_dot')
    # theta_dot = SX.sym('theta_dot')
    psi_dot = SX.sym('psi_dot')
    u_dot = SX.sym('u_dot')
    v_dot = SX.sym('v_dot')
    # w_dot = SX.sym('w_dot')
    # p_dot = SX.sym('p_dot')
    # q_dot = SX.sym('q_dot')
    r_dot = SX.sym('r_dot')
    sym_xdot = vertcat(x_dot,y_dot,psi_dot,u_dot,v_dot,r_dot)

    # system parameters
    m = 250.19                                       # mass
    Ixx = 225.17                                        # inertial
    Ixy = 0.12
    Ixz = 0.99
    Iyy = 454.14
    Iyz = 0.29
    Izz = 499.75
    xg = -0.003514
    yg = -0.000965
    zg = 0.255206
    LCG = 1.3
    B = 1.83
    added_mass = np.array([0,0,0,0,0,0])
    M = np.diag([m+added_mass[0], m+added_mass[1], m+added_mass[2], Izz+added_mass[5]]) # M_RB + M_A
    M_inv = np.linalg.inv(M)
    
    # # glf function: cmd to thrust
    # glf_v = fmax(glf_v, 1e-3)
    # Tp = glf_A + (glf_K-glf_A)/(glf_C + exp(-glf_B * (Tp_cmd - glf_M)))**(1/glf_v)
    # Ts = glf_A + (glf_K-glf_A)/(glf_C + exp(-glf_B * (Ts_cmd - glf_M)))**(1/glf_v)

    # thrust allocation
    Tx = Tp*cos(delta_p) + Ts*cos(delta_s)
    Ty = Tp*sin(delta_p) + Ts*sin(delta_s)
    Mz = -LCG*Tp*cos(delta_p) - B/2*Tp*sin(delta_p) - LCG*Ts*cos(delta_s) + B/2*Ts*sin(delta_s)

    # dynamics
    du = M_inv[0,0]*(Tx + m*v*r + m*xg*r*r)
    dv = M_inv[1,1]*(Ty - m*u*r + m*yg*r*r)
    dr = M_inv[2,2]*(Mz - m*xg*r*u - m*yg*r*v)
    
    dx = cos(psi)*u - sin(psi)*v
    dy = sin(psi)*u + cos(psi)*v
    dpsi = r

    f_expl = vertcat(dx,dy,dpsi,du,dv,dr)
    f_impl = sym_xdot - f_expl
  
    # constraints
    h_expr = sym_u

    # nonlinear least sqares
    cost_y_expr = vertcat(sym_x, sym_u)
    
    model = AcadosModel()
    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = sym_x
    model.xdot = sym_xdot
    model.u = sym_u
    # model.p = sym_p
    model.cost_y_expr = cost_y_expr
    model.cost_y_expr_e = sym_x
    #model.con_h_expr = h_expr
    model.name = model_name
    #model.cost_expr_ext_cost = expr_ext_cost
    #model.cost_expr_ext_cost_e = expr_ext_cost_e 

    return model