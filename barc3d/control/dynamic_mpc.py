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
class NonplanarMPCConfig(PythonMsg):
    N:  int      = field(default = 20)
    Q_4:  np.array = field(default = 100*np.diag([0,1,1,1]))
    R:  np.array = field(default = 0.0 * np.eye(2))
    dR: np.array = field(default = .1* np.eye(2))
    P_4:  np.array = field(default = 100*np.diag([0,1,1,1]))

    Q_6:  np.array = field(default = 100*np.diag([0,1,1,1,1,1]))
    P_6:  np.array = field(default = 100*np.diag([0,1,1,1,1,1]))
    
    dt:  float = field(default = 0.05)  # simulation time step and time between sucessive calls of MPC
    dtp: float = field(default = 0.05)  # time step for MPC prediction, not the frequency of the controller
    M:  int   = field(default = 1)    # subintervals for RK4 integration of dynamics ODE
    
    ua_min:   float = field(default = -10)
    ua_max:   float = field(default =  10)
    dua_min:  float = field(default = -np.inf) 
    dua_max:  float = field(default =  np.inf) 
    uy_min:   float = field(default = -0.5)
    uy_max:   float = field(default =  0.5)
    duy_min:  float = field(default = -np.inf)  
    duy_max:  float = field(default =  np.inf)
    s_min:   float = field(default = -np.inf)
    s_max:   float = field(default =  np.inf)
    y_min:   float = field(default = -4)
    y_max:   float = field(default =  4)
    V_min:   float = field(default = -20)
    V_max:   float = field(default =  20)
    ths_min: float = field(default = -2)
    ths_max: float = field(default =  2)
    
    yref: float   = field(default = 0) # lateral offset from centerline
    vref: float   = field(default = 10) # target speed
    
    use_planar: bool = field(default = False)
    
    Nmin: float   = field(default = 8000) # target minimum normal force
    Nmax: float   = field(default = 40000) # target minimum normal force
    planner_ds: float = field(default = .5)    # spacing of planner points
    planner_N:  int   = field(default = 60)   # number  of planner points
    
    #generate and compile code using CasADi, if false will only recompile if binaries not found.
    # since costs, reference speeds, and the surface are all currently intrinsic to the solver this has to be done frequently. 
    recompile:  bool = field(default = False) 

class NonplanarMPC(BaseController):
    def __init__(self, model: DynamicsModel, config: NonplanarMPCConfig = NonplanarMPCConfig()):
        self.model = model
        self.config = config
        self.build_mpc()
        if not self.config.use_planar:
            self.build_mpc_planner()
            self.vref = self.config.vref
        return
    
    def build_mpc(self):
        if self.model.f_zdot.size_in(0)[0] == 4:
            self.build_mpc_kinematic()
        elif self.model.f_zdot.size_in(0)[0] == 6:
            self.build_mpc_dynamic()
        else:
            raise NotImplementedError('Model not supported, state space error %d' % self.model.f_zdot.size_in(0)[0])
    
    
    def step(self, state:VehicleState):
        
        z0, u0 = self.model.state2zu(state)
        
        if self.model.f_zdot.size_in(0)[0] == 4:
            zref =  [0, self.config.yref, 0, self.config.vref]
        elif self.model.f_zdot.size_in(0)[0] == 6:
            zref =  [0, self.config.yref, 0, self.config.vref, 0, 0]
        
        duref = [0,0]
        
        t0 = time.time()
        if not self.config.use_planar:
            plan = self.planner(x0  = self.planner_x0,
                                lbx = self.planner_lbx, 
                                ubx = self.planner_ubx, 
                                lbg = self.planner_lbg, 
                                ubg = self.planner_ubg, 
                                p = z0+u0 + [self.vref])  
            self.planner_x0 = plan['x']
            zref[3] = float(plan['x'][0])
            self.vref = zref[3] 
        
        
        sol = self.solver(x0  = self.solver_x0, 
                          lbx = self.solver_lbx, 
                          ubx = self.solver_ubx, 
                          lbg = self.solver_lbg, 
                          ubg = self.solver_ubg, 
                          p = z0+u0+zref+duref)  
        
                                
            
        tf = time.time()
        
        state.t_sol = tf - t0
                                    
        state.u.a = sol['x'][2].__float__()
        state.u.y = sol['x'][3].__float__()
        
        
        if self.config.use_planar: state.u.a += state.q.e1()[2].__float__() * 9.81 
        
        self.unpack_prediction(sol['x'])
        self.solver_x0 = sol['x']
        
        return
        
    def unpack_prediction(self, solx):
        nu = self.F.size1_in(1)
        nx = self.F.size1_in(0)
        idx = 0
        du = []
        u  = []
        x  = []
        s = []
        for k in range(self.config.N):
            du.append(np.array(solx[idx       : idx + nu]))
            u.append( np.array(solx[idx + nu  : idx + 2*nu]))
            x.append( np.array(solx[idx + 2*nu: idx + 2*nu + nx]))
            
            idx += 2*nu + nx
        
        self.predicted_du = np.array(du).squeeze()
        self.predicted_u = np.array(u).squeeze()
        self.predicted_x = np.array(x).squeeze()
        return
    
    def build_mpc_kinematic(self):

        self.F = self.model.get_RK4_dynamics(dt = self.config.dtp, steps = self.config.M) #diff between kinematic and dynamic ?
        self.build_mpc_delta(4)  #diff between kinematic and dynamic ?

    def build_mpc_dynamic(self):
        self.F = self.model.get_RK4_dynamics(dt = self.config.dtp, steps = self.config.M)  #diff between kinematic and dynamic ?
        self.build_mpc_delta(6)  #diff between kinematic and dynamic ?
    
    def build_mpc_delta(self, state_size):
        '''
        sets up an mpc problem but with delta formulation in the input
        this helps with going up steep hills, where an affine input is necessary
        and in general where we'd rather penalize input rate over input magnitude
        in effect this tries to "automatically" attain an equillibrium input.
        
        input magnitude can still be penalized by config.R, and delta_input is penalized with config.dR
        it is recommended that R << dR
        '''
        
        # self.F = self.model.get_RK4_dynamics(dt = self.config.dtp, steps = self.config.M)
        
        z_size = self.F.size_in(0)
        u_size = self.F.size_in(1)
        
        p = []       # parameters (initial state, cost terms, terminal terms)
        w = []       # states     (vehicle state and inputs)
        w0 = []      
        lbw = []
        ubw = []
        J = 0        # cost
        g = []       # nonlinear constraint functions
        lbg = []
        ubg = []

        if state_size == 4:
            Q = self.config.Q_4
            P = self.config.P_4
            zl = [self.config.s_min, self.config.y_min, self.config.ths_min, self.config.V_min]
            zu = [self.config.s_max, self.config.y_max, self.config.ths_max, self.config.V_max]
        elif state_size == 6:
            Q = self.config.Q_6
            P = self.config.P_6
            zl = [self.config.s_min, self.config.y_min, self.config.ths_min, self.config.V_min, -np.inf, -np.inf] # add normal force and lateral force
            zu = [self.config.s_max, self.config.y_max, self.config.ths_max, self.config.V_max, np.inf, np.inf] # add normal force and lateral force
        
        
        
        zg  = [0] * self.F.size1_in(0) 
        
        ul = [self.config.ua_min, self.config.uy_min]
        uu = [self.config.ua_max, self.config.uy_max]
        ug  = [0] * self.F.size1_in(1)
        
        dul = [self.config.dua_min, self.config.duy_min]
        duu = [self.config.dua_max, self.config.duy_max]
        dug = [0] * self.F.size1_in(1)
        
        z = ca.MX.sym('z0',z_size)
        u = ca.MX.sym('up',u_size)
        zref  = ca.MX.sym('zref',z_size)
        duref = ca.MX.sym('duref',u_size)
        
        
        
        p += [z]
        p += [u]
        p += [zref]
        p += [duref]
        
        
        for k in range(self.config.N):
            # add delta state
            du = ca.MX.sym('du' + str(k), u_size)
            w += [du]
            lbw += dul
            ubw += duu
            w0  += dug
            if k == 0:  # separate dt for change from previous input compared to future prediction spacing
                unew = u + du * self.config.dt
            else:
                unew = u + du * self.config.dtp
            
            # add input state
            u  = ca.MX.sym('u'  + str(k), u_size)
            w += [u]
            lbw += ul
            ubw += uu
            w0 += ug
            
            # constrain input and delta
            g += [unew - u]
            ubg += [0] * u_size[0]
            lbg += [0] * u_size[0]
                
            # get new state
            znew = self.F(z,u)
            
            # penalize input, input delta, and new state
            if k == self.config.N - 1:
                J += ca.bilin(self.config.P,  znew-zref, znew-zref)
            else:
                J += ca.bilin(self.config.Q,  znew-zref, znew-zref)
            
                
            J += ca.bilin(self.config.R,  u,         u)
            J += ca.bilin(self.config.dR, du-duref,  du-duref)
            
            
            # add variable for next state
            z = ca.MX.sym('z' + str(k+1), z_size)
            w += [z]
            lbw += zl
            ubw += zu
            w0 += zg
            
            # add dynamics constraint to new state
            g += [z - znew]
            ubg += [0] * z_size[0]
            lbg += [0] * z_size[0]
            
        
        prob = {'f':J,'x':ca.vertcat(*w), 'g':ca.vertcat(*g), 'p':ca.vertcat(*p)}
        opts = {'ipopt.print_level': 0, 'ipopt.sb':'yes','print_time':0}
        
        if self.config.use_planar:
            solver = self.get_compiled_solver(prob, opts, 'planar_mpc')
        else:
            solver = self.get_compiled_solver(prob, opts, 'mpc')
        
        
        self.solver = solver
        self.solver_x0  = w0
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
        
        p = []       # parameters (initial state, cost terms, terminal terms)
        w = []       # states     (vehicle state and inputs)
        w0 = []      
        lbw = []
        ubw = []
        J = 0        # cost
        g = []       # nonlinear constraint functions
        lbg = []
        ubg = []
        
        z0 = ca.MX.sym('z0',z_size)
        u0 = ca.MX.sym('u0',u_size)
        
        p += [z0]
        p += [u0]
        
        V = ca.MX.sym('V')
        w += [V]
        ubw += [self.config.V_max]
        lbw += [self.config.V_min]
        w0  += [self.config.vref]
        
        Vprev = ca.MX.sym('Vprev')  # parameter for rate of change cost on plan
        p += [Vprev]
        
        J += (V - self.config.vref) ** 2
        
        # penalize sharp changes in reference velocity
        J += (V - Vprev) **2 * 2000
        
        
        for k in range(self.config.planner_N):
            # adjusted state variable with replanned speed and incremented path length
            z = ca.vertcat(z0[0] + self.config.planner_ds*k, 0, 0, V)
            
            N = fN(z,[0,0])
            
            # increment s spacing
            z[0] += self.config.planner_ds
            
            s = ca.MX.sym('s')
            w += [s]
            ubw += [np.inf]
            lbw += [-np.inf]
            w0  += [0]
            
            
            g += [N + s]
            ubg += [self.config.Nmax]
            lbg += [self.config.Nmin]
            
            J += s**2 * 100
            
        prob = {'f':J,'x':ca.vertcat(*w), 'g':ca.vertcat(*g), 'p':ca.vertcat(*p)}
        opts = {'ipopt.print_level': 0, 'ipopt.sb':'yes','print_time':0}
        
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
            solver = ca.nlpsol('solver','ipopt',prob)
            solver.generate_dependencies(sourcename)
            subp = subprocess.Popen('gcc -fPIC -shared -O3 %s -o %s'%(sourcename, compiledname), shell=True)
            
            print('Compiling %s...'%sourcename)
            subp.wait()
        
        load_name = './' + compiledname
        solver = ca.nlpsol('solver','ipopt',load_name,opts)
        return solver

