import pdb
import numpy as np
import casadi as ca
import os
import subprocess
import time as time

from typing import List
from dataclasses import dataclass, field

from barc3d.pytypes import PythonMsg, VehicleState
from barc3d.control.base_controller import BaseController
from barc3d.dynamics.dynamics_3d import DynamicsModel

@dataclass
class NonplanarMPCDynaConfig(PythonMsg):
    N: int = field(default=20)
    Q_6: np.array = field(default=100 * np.diag([0, 1, 1, 1, 1, 1]))
    R: np.array = field(default=0.001 * np.eye(2))
    dR: np.array = field(default=0.1 * np.eye(2))
    P_6: np.array = field(default=100 * np.diag([0, 1, 1, 1, 1, 1]))

    dt: float = field(default=0.05)  # simulation time step and time between successive calls of MPC
    dtp: float = field(default=0.05)  # time step for MPC prediction, not the frequency of the controller
    M: int = field(default=1)  # subintervals for RK4 integration of dynamics ODE

    ua_min: float = field(default=-10)
    ua_max: float = field(default=10)
    dua_min: float = field(default=-np.inf)
    dua_max: float = field(default=np.inf)
    uy_min: float = field(default=-0.5)
    uy_max: float = field(default=0.5)
    duy_min: float = field(default=-np.inf)
    duy_max: float = field(default=np.inf)
    s_min: float = field(default=-np.inf)
    s_max: float = field(default=np.inf)
    y_min: float = field(default=-4)
    y_max: float = field(default=4)
    V_min: float = field(default=-20)
    V_max: float = field(default=20)
    ths_min: float = field(default=-2)
    ths_max: float = field(default=2)

    yref: float = field(default=0)  # lateral offset from centerline
    vref: float = field(default=10)  # target speed

    use_planar: bool = field(default=False)

    Nmin: float = field(default=8000)  # target minimum normal force
    Nmax: float = field(default=40000)  # target minimum normal force
    planner_ds: float = field(default=0.5)  # spacing of planner points
    planner_N: int = field(default=60)  # number of planner points

    # generate and compile code using CasADi, if false will only recompile if binaries not found.
    # since costs, reference speeds, and the surface are all currently intrinsic to the solver this has to be done frequently.
    recompile: bool = field(default=False)


class NonplanarMPCDyna(BaseController):
    def __init__(self, model: DynamicsModel, config: NonplanarMPCDynaConfig = NonplanarMPCDynaConfig()):
        self.model = model
        self.config = config
        self.build_mpc()
        if not self.config.use_planar:
            self.build_mpc_planner()
            self.vref = self.config.vref
        return

    def build_mpc(self):
        self.build_mpc_dynamic()

    def step(self, state: VehicleState):
        z0, u0 = self.model.state2zu(state)  
        if z0 == [0, 0, 0, 0, 0, 0]:
            z0 = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]  

        if u0 == [0, 0]:
            u0 = [10.0, 0.0]
        print("Debug - z0:", z0)
        print("Debug - u0:", u0) 
 
        """  if u0 == [0, 0]:
            u0 = [0.1, 0.1] """

        zref = [0, self.config.yref, 0, self.config.vref, 1, 1]
        duref = [0, 0]

        t0 = time.time()
        if not self.config.use_planar:
            plan = self.planner(x0=self.planner_x0,
                                lbx=self.planner_lbx,
                                ubx=self.planner_ubx,
                                lbg=self.planner_lbg,
                                ubg=self.planner_ubg,
                                #p = np.concatenate((z0[:4], u0, [self.vref])))
                                p = np.concatenate((z0, [self.vref])))
            self.planner_x0 = plan['x']
            zref[3] = float(plan['x'][0])
            self.vref = zref[3]

        p = np.concatenate((z0, u0, zref, duref))

        # Print parameter vector p for debugging
        print("Parameter vector p:", p)
        print("Parameter vector dimension:", p.shape)

        
   
        print("Solver initial guess (solver_x0):", self.solver_x0)
        sol = self.solver(x0=self.solver_x0,
                        lbx=self.solver_lbx,
                        ubx=self.solver_ubx,
                        lbg=self.solver_lbg,
                        ubg=self.solver_ubg,
                        p=p)
        
        stats = self.solver.stats()
        print("Solver Status:", stats['return_status'])  # Solution status
        print("Number of Iterations:", stats['iter_count'])  # Number of iterations


        
        # Print solver input dimensions for debugging
        #print("Solver initial guess (solver_x0):", self.solver_x0)

        """ print("Solver lower bounds (solver_lbx):", self.solver_lbx)

        print("Solver upper bounds (solver_ubx):", self.solver_ubx)

        print("Solver lower bounds (solver_lbg):", self.solver_lbg)
    
        print("Solver upper bounds (solver_ubg):", self.solver_ubg) """

        tf = time.time()
        state.t_sol = tf - t0

        # Print solver output for debugging
        print("Solver OUTPUT:", sol['x']) # ISSUE HERE ALWAYS = 0

        state.u.a = sol['x'][2].__float__()
        state.u.y = sol['x'][3].__float__()
        print("Debug - Updated control inputs: a = {}, y = {}".format(state.u.a, state.u.y))

        if self.config.use_planar:
            state.u.a += state.q.e1()[2].__float__() * 9.81

        self.unpack_prediction(sol['x'])
        self.solver_x0 = sol['x']

        return

    def unpack_prediction(self, solx):
        nu = self.F.size1_in(1)
        nx = self.F.size1_in(0)
        idx = 0
        du = []
        u = []
        x = []
        s = []
        for k in range(self.config.N):
            du.append(np.array(solx[idx: idx + nu]))
            u.append(np.array(solx[idx + nu: idx + 2 * nu]))
            x.append(np.array(solx[idx + 2 * nu: idx + 2 * nu + nx]))

            idx += 2 * nu + nx

        self.predicted_du = np.array(du).squeeze()
        self.predicted_u = np.array(u).squeeze()
        self.predicted_x = np.array(x).squeeze()
        return

    def build_mpc_dynamic(self):
        self.F = self.model.get_RK4_dynamics(dt=self.config.dtp, steps=self.config.M)
        self.build_mpc_delta(6)

    def build_mpc_delta(self, state_size):
        z_size = self.F.size_in(0)[0]
        u_size = self.F.size_in(1)[0]

        p = []  # parameters (initial state, cost terms, terminal terms)
        w = []  # states (vehicle state and inputs)
        w0 = []
        lbw = []
        ubw = []
        J = 0  # cost
        g = []  # nonlinear constraint functions
        lbg = []
        ubg = []

        if state_size == 6:
            Q = self.config.Q_6
            P = self.config.P_6
            zl = [self.config.s_min, self.config.y_min, self.config.ths_min, self.config.V_min, -5000, -5000]  # add normal force and lateral force
            zu = [self.config.s_max, self.config.y_max, self.config.ths_max, self.config.V_max, 50000, 5000]  # add normal force and lateral force
            """
            vehicle_mass = 1500  # kg
            gravity = 9.81  # m/s^2
            normal_force_static = vehicle_mass * gravity / 4  # Static normal force on each wheel

            # Typical coefficient of friction for tires
            mu = 1.0  # Dry asphalt

            # Dynamic load transfer (example value, this depends on vehicle dynamics)
            dynamic_load_transfer = 0.2 * normal_force_static

            # Set bounds for normal force
            N_min = normal_force_static - dynamic_load_transfer
            N_max = normal_force_static + dynamic_load_transfer

            # Set bounds for lateral force based on friction limit
            lateral_force_max = mu * N_max
            """

        zg = [0] * z_size

        ul = [self.config.ua_min, self.config.uy_min]
        uu = [self.config.ua_max, self.config.uy_max]
        ug = [0] * u_size

        dul = [self.config.dua_min, self.config.duy_min]
        duu = [self.config.dua_max, self.config.duy_max]
        dug = [0] * u_size

        z = ca.MX.sym('z0', z_size)
        u = ca.MX.sym('up', u_size)
        zref = ca.MX.sym('zref', z_size)
        duref = ca.MX.sym('duref', u_size)

        p += [z]
        p += [u]
        p += [zref]
        p += [duref]

        for k in range(self.config.N):
            du = ca.MX.sym('du' + str(k), u_size)
            w += [du]
            lbw += dul
            ubw += duu
            w0 += dug
            if k == 0:
                unew = u + du * self.config.dt
            else:
                unew = u + du * self.config.dtp

            u = ca.MX.sym('u' + str(k), u_size)
            w += [u]
            lbw += ul
            ubw += uu
            w0 += ug

            g += [unew - u]
            ubg += [0] * u_size
            lbg += [0] * u_size

            znew = self.F(z, u)

            if k == self.config.N - 1:
                J += ca.bilin(P, znew - zref, znew - zref)
            else:
                J += ca.bilin(Q, znew - zref, znew - zref)

            J += ca.bilin(self.config.R, u, u)
            J += ca.bilin(self.config.dR, du - duref, du - duref)

            z = ca.MX.sym('z' + str(k + 1), z_size)
            w += [z]
            lbw += zl
            ubw += zu
            w0 += zg

            g += [z - znew]
            ubg += [0] * z_size
            lbg += [0] * z_size
            

        prob = {'f': J, 'x': ca.vertcat(*w), 'g': ca.vertcat(*g), 'p': ca.vertcat(*p)}
        opts = {'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': 0}

        solver = self.get_compiled_solver(prob, opts, 'dynamic_mpc')

        self.solver = solver
        """ self.solver_x0 = np.zeros(len(w)).tolist()  # Initial guess for decision variables
        self.solver_lbx = np.array(lbw).flatten().tolist()  # Lower bounds for decision variables
        self.solver_ubx = np.array(ubw).flatten().tolist()  # Upper bounds for decision variables
        self.solver_lbg = np.array(lbg).flatten().tolist()  # Lower bounds for constraints
        self.solver_ubg = np.array(ubg).flatten().tolist()  # Upper bounds for constraints """
        print("Debug - w0:", w0)
        self.solver_x0 = w0
        self.solver_lbx = lbw
        self.solver_ubx = ubw
        self.solver_lbg = lbg
        self.solver_ubg = ubg
        return

    def build_mpc_planner(self):
        '''
        sets up a speed planner that will look ahead of the vehicle (fixed intervals of path length)
        and search for a speed that is close to vref but also satisfies normal force requirements
        '''
        fN = self.model.f_N

        z_size = fN.size_in(0)
        u_size = fN.size_in(1)

        p = []  # parameters (initial state, cost terms, terminal terms)
        w = []  # states (vehicle state and inputs)
        w0 = []
        lbw = []
        ubw = []
        J = 0  # cost
        g = []  # nonlinear constraint functions
        lbg = []
        ubg = []
        load = 0
        lateral_slip = 0

        z0 = ca.MX.sym('z0', z_size)
        u0 = ca.MX.sym('u0', u_size)
        print("z0:", z0.shape)
        print("u0:", u0.shape)

        p += [z0]
        p += [u0]

        V = ca.MX.sym('V')
        w += [V]
        ubw += [self.config.V_max]
        lbw += [self.config.V_min]
        w0 += [self.config.vref]

        Vprev = ca.MX.sym('Vprev')  # parameter for rate of change cost on plan
        p += [Vprev]

        J += (V - self.config.vref) ** 2

        # penalize sharp changes in reference velocity
        J += (V - Vprev) ** 2 * 2000

        for k in range(self.config.planner_N):
            # adjusted state variable with replanned speed and incremented path length
            z = ca.vertcat(z0[0] + self.config.planner_ds * k, 0, 0, V, load, lateral_slip)
            N = fN(z, [0, 0])

            # increment s spacing
            z[0] += self.config.planner_ds

            s = ca.MX.sym('s')
            w += [s]
            ubw += [np.inf]
            lbw += [-np.inf]
            w0 += [0]

            g += [N + s]
            ubg += [self.config.Nmax]
            lbg += [self.config.Nmin]

            J += s ** 2 * 100

        prob = {'f': J, 'x': ca.vertcat(*w), 'g': ca.vertcat(*g), 'p': ca.vertcat(*p)}
        opts = {'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': 0}

        planner = self.get_compiled_solver(prob, opts, 'planner')

        self.planner = planner
        self.planner_x0  = w0
        self.planner_lbx = lbw
        self.planner_ubx = ubw
        self.planner_lbg = lbg
        self.planner_ubg = ubg

    def get_compiled_solver(self, prob, opts, name):
        sourcename = name + '.c'
        compiledname = name + '.so'

        if not os.path.exists(compiledname) or self.config.recompile:
            solver = ca.nlpsol('solver', 'ipopt', prob)
            solver.generate_dependencies(sourcename)
            subp = subprocess.Popen('gcc -fPIC -shared -O3 %s -o %s' % (sourcename, compiledname), shell=True)

            print('Compiling %s...' % sourcename)
            subp.wait()

        load_name = './' + compiledname
        solver = ca.nlpsol('solver', 'ipopt', load_name, opts)
        return solver

