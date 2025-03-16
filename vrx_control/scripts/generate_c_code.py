from acados_template import AcadosOcp, AcadosOcpSolver
from wamv import export_wamv_model
import numpy as np
from scipy.linalg import block_diag

def main():
    ocp = AcadosOcp()
    model = export_wamv_model()
    ocp.model = model

    Tf = 1.0
    nx = model.x.size()[0]  # 6
    nu = model.u.size()[0]  # 4
    nz = model.z.size()[0]  # 4
    ny = nx + nu + nz       # 14
    nparam = model.p.size()[0]
    N = 60

    ocp.solver_options.N_horizon = N
    ocp.dims.nz = nz
    ocp.parameter_values = np.zeros((nparam, ))
    ocp.dims.np = nparam

    # Cost
    W_x = np.diag([1000, 200, 200, 200, 50, 50])     # x,y,psi,u,v,r
    W_u = np.diag([0.5, 0.5, 800, 800])             #Tp,Ts,delta_p,delta_s
    W_du = np.diag([1, 1, 10, 10])        
    W = block_diag(W_x, W_u, W_du)
    ocp.cost.W = W
    ocp.cost.W_e = W_x

    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'
    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vu = np.zeros((ny, nu))
    ocp.cost.Vu[nx:nx+nu, :] = np.eye(nu)
    ocp.cost.Vz = np.zeros((ny, nz))
    ocp.cost.Vz[nx+nu:, :] = np.eye(nz)
    ocp.cost.Vx_e = np.eye(nx)

    # Constraints
    u_min = np.array([0, 0, -2, -2])
    u_max = np.array([2353, 2353, 2, 2])
    ocp.constraints.lbu = u_min
    ocp.constraints.ubu = u_max
    ocp.constraints.idxbu = np.array([0, 1, 2, 3])
    ocp.constraints.x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    # Slack bounds
    ocp.constraints.lbz = np.array([-100,-100,-0.1,-0.1])  
    ocp.constraints.ubz = np.array([100,100,0.1,0.1])
    # ocp.constraints.lbz = np.ones(nz)
    # ocp.constraints.ubz = np.ones(nz)
    ocp.constraints.idxbz = np.array([0, 1, 2, 3])

    # Reference
    x_ref = np.zeros(nx)
    u_ref = np.zeros(nu)
    z_ref = np.zeros(nz)
    ocp.cost.yref = np.concatenate((x_ref, u_ref, z_ref))
    ocp.cost.yref_e = x_ref

    # Solver options
    ocp.solver_options.qp_solver = 'FULL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'IRK'  # For DAEs
    ocp.solver_options.print_level = 0
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    ocp.solver_options.tf = Tf

    ocp_solver = AcadosOcpSolver(ocp, json_file='acados_ocp.json')

    simX = np.ndarray((N+1, nx))
    simU = np.ndarray((N, nu))

    status = ocp_solver.solve()
    ocp_solver.print_statistics()

    if status != 0:
        raise Exception(f'acados returned status {status}.')

    for i in range(N):
        simX[i, :] = ocp_solver.get(i, "x")
        simU[i, :] = ocp_solver.get(i, "u")
    simX[N, :] = ocp_solver.get(N, "x")

if __name__ == '__main__':
    main()