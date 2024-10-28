import sys
import pdb
import numpy as np
import time as time
from numpy import pi as pi
import pickle as pkl

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib

use_glumpy_fig = False  # legacy figure

from barc3d.pytypes import VehicleState, VehicleConfig, VehicleConfig1_10, plot_pacjeka_tire_forces, plot_vehicle_state
from barc3d.surfaces import load_surface, get_available_surfaces
from barc3d.surfaces.tait_bryan_surface import TaitBryanAngleSurface
from barc3d.dynamics.dynamics_3d import KinematicBicycle3D, KinematicBicyclePlanar, DynamicBicycle3D, DynamicTwoTrack3D, DynamicTwoTrackSlipInput3D
from barc3d.control.pid import PIDController, PIDConfig
from barc3d.control.simple_stanley_pid import SimpleStanleyPIDController, StanleyConfig
from barc3d.control.mpc import NonplanarMPC, NonplanarMPCConfig
from barc3d.control.mpc_dyna import NonplanarMPCDyna, NonplanarMPCDynaConfig

if use_glumpy_fig: # importing both together can cause errors
    from barc3d.visualization.glumpy_fig import GlumpyFig
else:
    from barc3d.visualization.opengl_fig import OpenGLFig
from barc3d.utils.run_lap import run_solo_lap

vref = 9
yref = -5
dt = 0.05

track_list = get_available_surfaces()
print(track_list)
track_test = ['tube_turn_100%']
for track in track_test:
    surface_name = track.split('.')[0]
 
    speed_plot = f'{vref}'

    surf = load_surface(surface_name)

    vehicle_config = VehicleConfig(dt = dt, road_surface = False)
    #vehicle_config = VehicleConfig1_10(dt = dt, road_surface = False)

    # simulator = KinematicBicycle3D(vehicle_config = vehicle_config, surf = surf)

    # Models
    """ 
    KinematicBiczcle3D: nonplanar kinematic bicycle model. Includes state evolution based on velocity components and accounts for gravity drag forces.

    KinematicBicyclePlanar: planar kinematic bicycle model. 

    DynamicBicycle3D: A nonplanar dynamic bicycle model with static weight distribution and simplified longitudinal dynamics. It includes quasistatic loading and tire forces.

    DynamicTwoTrack3D: A nonplanar dynamic two-track model with dynamic weight distribution and tire angular velocity states. 

    DynamicTwoTrackSlipInput3D: A nonplanar dynamic two-track model where the tire slip ratio is treated as an input. This model is always set up as a DAE (Differential Algebraic Equation).
    """
    # nonplanar_model = simulator
    # planar_model = KinematicBicyclePlanar(vehicle_config = vehicle_config, surf = surf)
    # dynamic_model = DynamicBicycle3D(vehicle_config = vehicle_config, surf = surf)
    # dyna_two_track = DynamicTwoTrack3D(vehicle_config = vehicle_config, surf = surf)
    # dyna_two_track_slip = DynamicTwoTrackSlipInput3D(vehicle_config = vehicle_config, surf = surf)

    # Controllers
    state = VehicleState()

    # pid_kinematic = PIDController(PIDConfig(dt=simulator.dt, vref=vref, yref=yref))
    # pid_dynamic = PIDController(PIDConfig(dt=dynamic_model.dt, vref=vref, yref=yref))
    # stanley_kinematic = SimpleStanleyPIDController(StanleyConfig(dt=simulator.dt, vref=vref, yref=yref))
    # stanley_dynamic = SimpleStanleyPIDController(StanleyConfig(dt=dynamic_model.dt, vref=vref, yref=yref))
    # mpc_planar = NonplanarMPC(model=planar_model, config=NonplanarMPCConfig(dt=simulator.dt, use_planar=True, vref=vref, yref=yref))
    # mpc_nonplanar_kinematic = NonplanarMPC(model=nonplanar_model, config=NonplanarMPCConfig(dt=simulator.dt, use_planar=False, vref=vref, yref=yref))
    # mpc_nonplanar_dynamic = NonplanarMPCDyna(model=dynamic_model, config=NonplanarMPCDynaConfig(dt=dynamic_model.dt, use_planar=True, vref=vref, yref=yref))
    

    # controller          = PIDController(                                 PIDConfig(dt = simulator.dt,     vref = vref,yref = yref))
    # stanley_controller  = SimpleStanleyPIDController(                    StanleyConfig(dt = simulator.dt, vref = vref,yref = yref))
    # mpc                 = NonplanarMPC(model = nonplanar_model, config = NonplanarMPCConfig(dt = simulator.dt, use_planar = False, vref = vref,yref = yref))
    # pmpc                = NonplanarMPCDyna(model = dynamic_model,    config = NonplanarMPCDynaConfig(dt = simulator.dt, use_planar = True,  vref = vref,yref = yref))

    # # Dynamic Controller
    # pid = PIDController(PIDConfig(dt = dynamic_model.dt, vref = vref, yref = yref))
    # stanley = SimpleStanleyPIDController(StanleyConfig(dt = simulator.dt, vref = vref, yref = yref))
    # pmpc_kine = NonplanarMPC(model = planar_model, config = NonplanarMPCConfig(dt = simulator.dt, use_planar = False, vref = vref, yref = yref))
    # pmpc_dyna = NonplanarMPCDyna(model = dynamic_model, config = NonplanarMPCDynaConfig(dt = dynamic_model.dt, use_planar = True, vref = vref, yref = yref))

    #----------------------------------------------------------------------------------------------------------
    # TESTING
    # #----------------------------------------------------------------------------------------------------------
    vehicle_model = KinematicBicycle3D(vehicle_config = vehicle_config, surf = surf)
    pid_controller = PIDController(PIDConfig(dt = vehicle_model.dt, vref = vref, yref = yref))
    stanley_controller  = SimpleStanleyPIDController(StanleyConfig(dt = vehicle_model.dt, vref = vref,yref = yref))
    pmpc                = NonplanarMPC(model=vehicle_model, config=NonplanarMPCConfig(dt=vehicle_model.dt, use_planar=True, vref=vref, yref=yref))
    mpc                 = NonplanarMPC(model=vehicle_model, config=NonplanarMPCConfig(dt=vehicle_model.dt, use_planar=False, vref=vref, yref=yref))
        

    if use_glumpy_fig:
        if GlumpyFig.available():
            print('Using glumpy figure')
            figure = GlumpyFig(surf, vehicle_config)
        else:
            figure = None
            print('No figure available, will not plot in real time')
    else:
        print('opengl figure')
        figure = OpenGLFig(surf, vehicle_config) #, simulator.vehicle_config)

    if figure is not None:
        while not figure.ready():  # wait for the figure to initialize textures
            pass
            
    # pid_traj     = run_solo_lap(controller,         simulator, surf, figure = figure, plot = True, lap = 'pid')
    # stanley_traj = run_solo_lap(stanley_controller, simulator, surf, figure = figure, plot = True, lap = 'stanley')
    # pmpc_traj    = run_solo_lap(pmpc,               simulator, surf, figure = figure, plot = True, lap = 'planar mpc')
    # mpc_traj     = run_solo_lap(mpc,                simulator, surf, figure = figure, plot = True, lap = 'nonplanar mpc')

    # pid_dyna_traj     = run_solo_lap(pid, dynamic_model, surf, figure = figure, plot = True, lap = 'pid_dyna')
    # stanley_dyna_traj = run_solo_lap(stanley, dynamic_model, surf, figure = figure, plot = True, lap = 'stanley_dyna')
    # mpc_nonplanar_kinematic = run_solo_lap(mpc_nonplanar_kinematic, nonplanar_model, surf, figure = figure, plot = True, lap = 'nonplanar mpc_kinematic')
    # pmpc_dyna_traj    = run_solo_lap(pmpc_dyna, dynamic_model, surf, figure = figure, plot = True, lap = 'planar mpc_dyna')

    #----------------------------------------------------------------------------------------------------------
    # TESTING
    #----------------------------------------------------------------------------------------------------------
    pid_traj     = run_solo_lap(pid_controller, vehicle_model, surf, figure = figure, plot = True, lap = 'pid')
    stanley_traj = run_solo_lap(stanley_controller, vehicle_model, surf, figure = figure, plot = True, lap = 'stanley')
    pmpc_traj    = run_solo_lap(pmpc, vehicle_model, surf, figure = figure, plot = True, lap = 'planar mpc')
    mpc_traj     = run_solo_lap(mpc, vehicle_model, surf, figure = figure, plot = True, lap = 'nonplanar mpc')

    # save trajectories:
    # pkl.dump(pid_traj,     open(f'results/trajectories/pid_{surface_name}_{speed_plot}.pkl', 'wb'))
    # pkl.dump(stanley_traj, open(f'results/trajectories/stanley_{surface_name}_{speed_plot}.pkl', 'wb'))
    # pkl.dump(pmpc_traj,    open(f'results/trajectories/pmpc_{surface_name}_{speed_plot}.pkl', 'wb'))
    # pkl.dump(mpc_traj,     open(f'results/trajectories/mpc_{surface_name}_{speed_plot}.pkl', 'wb'))


    # ts = np.mean(np.array([s.t_sol*1000 for s in pmpc_traj]))
    # print('Avg planar mpc solve time: %0.3f'%ts)
    # ts = np.mean(np.array([s.t_sol*1000 for s in mpc_traj]))
    # print('Avg nonplanar mpc solve time: %0.3f'%ts)


    matplotlib.rcParams['mathtext.fontset'] = 'cm'
    matplotlib.rcParams.update({'font.size': 18})
    #matplotlib.rcParams['text.latex.preamble']=[r"\usepackage{amsmath}"]

        
    def plot_solve_time(traj, ax = None):
        ts = [s.t_sol*1000 for s in traj]
        s  = [s.p.s for s in traj]
        plt.plot(s,ts)
        plt.xlabel(r'$s$')
        plt.ylabel('Solve Time (ms)')
        
    def plot_timeseries_results(trajs, styles, labels, filename = None):
        fig = plt.figure(constrained_layout=True, figsize=(10, 10))
        #plt.figure()     
        gs1 = gridspec.GridSpec(7, 1, figure=fig)
        gs1.update(wspace=0.025, hspace=0.1) # set the spacing between axes. 
        
        ax1 = plt.subplot(gs1[0])
        ax2 = plt.subplot(gs1[1])
        ax3 = plt.subplot(gs1[2])
        ax4 = plt.subplot(gs1[3])
        ax5 = plt.subplot(gs1[4])
        ax6 = plt.subplot(gs1[5])
        ax7 = plt.subplot(gs1[6])
        ax1.set_xticklabels([])
        ax2.set_xticklabels([])
        ax3.set_xticklabels([])
        ax4.set_xticklabels([])
        ax5.set_xticklabels([])
        ax6.set_xticklabels([])
        
        for (traj,style, label) in zip(trajs, styles, labels):
            s = [s.p.s   for s in traj]
            y = [s.p.y   for s in traj]
            th = [s.p.ths for s in traj]
            tha = [np.arctan(vehicle_model.lr / (vehicle_model.lf + vehicle_model.lr) * np.tan(s.u.y)) + s.p.ths for s in traj]
            v = [np.sqrt(s.v.v1**2 + s.v.v2**2)   for s in traj]
            ua = [s.u.a.__float__()   for s in traj]
            uy = [s.u.y.__float__()   for s in traj]
            N = [s.fb.f3.__float__() for s in traj]
        
            ax1.plot(s,y, style, label=label)
            ax2.plot(s,th, style)   
            ax3.plot(s,tha,style)
            ax4.plot(s,v,  style)
            ax5.plot(s,N,  style)
            ax6.plot(s,ua,  style)
            ax7.plot(s,uy,  style)
        
        # ax5.fill_between([min(s), max(s)], [mpc.config.Nmax, mpc.config.Nmax], [mpc.config.Nmin, mpc.config.Nmin], color = 'green',alpha = 0.3)
        ax5.plot([min(s), max(s)], [0, 0], 'k')
        
        ax1.set_ylabel(r'$y$')
        ax2.set_ylabel(r'$\theta^s$')
        ax3.set_ylabel(r'$\theta^s + \beta$')
        ax4.set_ylabel(r'$v$')
        ax5.set_ylabel(r'$F_N^b$')
        ax6.set_ylabel(r'$a_t$')
        ax7.set_ylabel(r'$\gamma$')
        ax7.set_xlabel(r'$s$')
        
        ax1.yaxis.tick_right()
        ax2.yaxis.tick_right()
        ax3.yaxis.tick_right()
        ax4.yaxis.tick_right()
        ax5.yaxis.tick_right()
        ax6.yaxis.tick_right()
        ax7.yaxis.tick_right()

        for ax in [ax1, ax2, ax3, ax4, ax5, ax6, ax7]:
            ax.grid(True)

        handles, labels = ax1.get_legend_handles_labels()
        fig.legend(handles, labels, loc = 'upper left', fontsize = 12, ncol = len(labels), bbox_to_anchor=(0, 1), bbox_transform=ax1.transAxes)

        if filename is not None:
            plt.savefig(filename)
        #plt.show()

        #plt.tight_layout()
        
    # Plot with or without PID: 
    #plot_timeseries_results([stanley_traj, pmpc_traj, mpc_traj], [':r', '--g', 'b'], filename = 'barc3d/results/test2.png')
    plot_timeseries_results([pid_traj, stanley_traj, pmpc_traj, mpc_traj], [':y', ':r', '--g', 'b'], ['PID', 'Stanley', 'Planar MPC', 'Nonplanar MPC'],filename = f'results/traj_{surface_name}_v{speed_plot}.png')
    # plot_timeseries_results([pid_traj, stanley_traj, mpc_traj], [':y', ':r', 'b'], ['PID', 'Stanley', 'Nonplanar MPC'],filename = f'results/traj_{surface_name}_v{speed_plot}.png')
    # fig = plt.figure()

    # plot_solve_time(pmpc_traj)
    # plot_solve_time(mpc_traj)
    # plt.legend(('Planar','Nonplanar'))
    # plt.tight_layout()

    # # save plots in results folder
    # plt.savefig(f'results/solving_time_{surface_name}_v{speed_plot}.png')

    #plt.show()


    # figure.close()  # makes sure the second process stops





