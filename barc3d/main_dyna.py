import sys
import pdb
import numpy as np
import time as time
from numpy import pi as pi
import pickle as pkl

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib
from matplotlib import cm
from matplotlib.colors import Normalize

use_glumpy_fig = False  # legacy figure

from barc3d.pytypes import VehicleState, VehicleConfig, VehicleConfig1_10, plot_pacjeka_tire_forces, plot_vehicle_state, ParametricPose
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

vref = 10
yref = 0
dt = 0.05

track_list = get_available_surfaces()
track_test = ['single_bump']
for track in track_test:
    surface_name = track.split('.')[0]

    speed_plot = f'{vref}'

    surf = load_surface(surface_name)

    

    vehicle_config = VehicleConfig(dt = dt, road_surface = False)
    #vehicle_config = VehicleConfig1_10(dt = dt, road_surface = False)

    simulator = KinematicBicycle3D(vehicle_config = vehicle_config, surf = surf)

    nonplanar_model = simulator
    planar_model = KinematicBicyclePlanar(vehicle_config = vehicle_config, surf = surf)
    dynamic_model = DynamicBicycle3D(vehicle_config = vehicle_config, surf = surf)
    
    

    # Controllers
    state = VehicleState()


    mpc_nonplanar_kinematic = NonplanarMPC(model=nonplanar_model, config=NonplanarMPCConfig(dt=simulator.dt, use_planar=False, vref=vref, yref=yref))
    dynamic_mpc = NonplanarMPCDyna(model=dynamic_model, config=NonplanarMPCDynaConfig(dt=dynamic_model.dt, use_planar=False, vref=vref, yref=yref))


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
   
    input("Press any key to continue...")

    kine_mpc_traj = run_solo_lap(mpc_nonplanar_kinematic, nonplanar_model, surf, figure = figure, plot = True, lap = 'nonplanar mpc_kinematic')
    dyna_mpc_traj = run_solo_lap(dynamic_mpc, dynamic_model, surf, figure = figure, plot = True, lap = 'planar mpc_dyna')

    
    ts = np.mean(np.array([s.t_sol*1000 for s in kine_mpc_traj]))
    print('Avg planar mpc solve time: %0.3f'%ts)
    ts = np.mean(np.array([s.t_sol*1000 for s in dyna_mpc_traj]))
    print('Avg nonplanar mpc solve time: %0.3f'%ts)

    # solving time after initial input
    ts_1 = np.mean(np.array([s.t_sol*1000 for s in kine_mpc_traj if s.p.s > 1]))
    print('Avg planar mpc solve time (after 1s): %0.3f' % ts_1)

    # Calculate and print average solve time for nonplanar MPC
    ts_1 = np.mean(np.array([s.t_sol*1000 for s in dyna_mpc_traj if s.p.s > 1]))
    print('Avg nonplanar mpc solve time (after 1s): %0.3f' % ts_1)


    matplotlib.rcParams['mathtext.fontset'] = 'cm'
    matplotlib.rcParams.update({'font.size': 18})
    #matplotlib.rcParams['text.latex.preamble']=[r"\usepackage{amsmath}"]



    def plot_solve_time(traj, ax = None):
        ts = [s.t_sol*1000 for s in traj]
        s  = [s.p.s for s in traj]
        plt.plot(s,ts)
        plt.xlabel(r'$s$')
        plt.ylabel('Solve Time (ms)')

    def plot_solve_time_1(traj, ax=None):
        ts = [s.t_sol*1000 for s in traj if s.p.s > 1]
        s  = [s.p.s for s in traj if s.p.s > 1]
        plt.plot(s, ts)
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
            tha = [np.arctan(simulator.lr / (simulator.lf + simulator.lr) * np.tan(s.u.y)) + s.p.ths for s in traj]
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
        
        ax5.fill_between([min(s), max(s)], [mpc_nonplanar_kinematic.config.Nmax, mpc_nonplanar_kinematic.config.Nmax], [mpc_nonplanar_kinematic.config.Nmin, mpc_nonplanar_kinematic.config.Nmin], color = 'green',alpha = 0.3)
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
    plot_timeseries_results([kine_mpc_traj, dyna_mpc_traj], [':r', '--g'], ['Kinetamic', 'Dynamic'],filename = f'barc3d/results/traj_kin_dyna{surface_name}_v{speed_plot}.png')
    fig = plt.figure()

    """ plot_solve_time(kine_mpc_traj)
    plot_solve_time(dyna_mpc_traj)
    plt.legend(('Planar','Nonplanar'))
    plt.tight_layout()
    plt.savefig(f'barc3d/results/solving_time_test_{surface_name}_v{speed_plot}.png') """

    plot_solve_time_1(kine_mpc_traj)
    plot_solve_time_1(dyna_mpc_traj)
    plt.legend(('Kinematic','Dynamic'))
    plt.tight_layout()

        # save plots in results folder
    plt.savefig(f'barc3d/results/solving_time_test1_{surface_name}_v{speed_plot}.png')

    print('Saved plots in results folder')

    def plot_control_inputs(trajs, styles, labels, filename=None):
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
        for traj, style, label in zip(trajs, styles, labels):
            s = [s.p.s for s in traj]
            ua = [s.u.a.__float__() for s in traj]
            uy = [s.u.y.__float__() for s in traj]
            ax1.plot(s, ua, style, label=label)
            ax2.plot(s, uy, style, label=label)
        ax1.set_ylabel('Acceleration')
        ax2.set_ylabel('Steering Angle')
        ax2.set_xlabel('Distance (s)')
        ax1.grid(True)
        ax2.grid(True)
        ax1.legend()
        if filename:
            plt.savefig(filename)

    def plot_tracking_error(trajs, styles, labels, filename=None):
        fig, ax = plt.subplots(figsize=(10, 6))
        for traj, style, label in zip(trajs, styles, labels):
            s = [s.p.s for s in traj]
            error = [s.p.y - yref for s in traj]
            ax.plot(s, error, style, label=label)
        ax.set_ylabel('Tracking Error (m)')
        ax.set_xlabel('Distance (s)')
        ax.grid(True)
        ax.legend()
        if filename:
            plt.savefig(filename)

    def plot_energy_consumption(trajs, styles, labels, vehicle_mass, dt, filename=None):
        fig, ax = plt.subplots(figsize=(10, 6))
        for traj, style, label in zip(trajs, styles, labels):
            s = [s.p.s for s in traj]
            energy = np.cumsum([np.abs(s.u.a.__float__() * s.v.v1 * vehicle_mass * dt) for s in traj])  # Energy in Joules
            ax.plot(s, energy, style, label=label)
        ax.set_ylabel('Energy Consumption (Joules)')
        ax.set_xlabel('Distance (s)')
        ax.grid(True)
        ax.legend()
        if filename:
            plt.savefig(filename)



    # Example usage
    vehicle_mass = vehicle_config.m  # Define vehicle mass in kilograms
    plot_energy_consumption([kine_mpc_traj, dyna_mpc_traj], [':r', '--g'], ['Kinematic', 'Dynamic'], vehicle_mass, dt, filename=f'barc3d/results/energy_consumption_{surface_name}_v{speed_plot}.png')
    plot_control_inputs([kine_mpc_traj, dyna_mpc_traj], [':r', '--g'], ['Kinematic', 'Dynamic'], filename=f'barc3d/results/control_inputs_{surface_name}_v{speed_plot}.png')
    plot_tracking_error([kine_mpc_traj, dyna_mpc_traj], [':r', '--g'], ['Kinematic', 'Dynamic'], filename=f'barc3d/results/tracking_error_{surface_name}_v{speed_plot}.png')
    figure.close()  # makes sure the second process stops

    def plot_trajectory(trajs, styles, labels, filename=None):
        plt.figure()
        for traj, style, label in zip(trajs, styles, labels):
            s = [s.p.s for s in traj]
            y = [s.p.y for s in traj]
            plt.plot(s, y, style, label=label)
        plt.xlabel('Distance (s)')
        plt.ylabel('Lateral Position (m)')
        plt.legend()
        plt.grid(True)
        if filename:
            plt.savefig(filename)
        plt.show()

    def plot_velocity_components(trajs, styles, labels, filename=None):
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))
        for traj, style, label in zip(trajs, styles, labels):
            s = [s.p.s for s in traj]
            v_longitudinal = [s.v.v1 for s in traj]
            v_lateral = [s.v.v2 for s in traj]
            ax1.plot(s, v_longitudinal, style, label=label)
            ax2.plot(s, v_lateral, style, label=label)
        ax1.set_ylabel('Longitudinal Velocity (m/s)')
        ax2.set_ylabel('Lateral Velocity (m/s)')
        ax2.set_xlabel('Distance (s)')
        ax1.legend()
        ax1.grid(True)
        ax2.grid(True)
        if filename:
            plt.savefig(filename)
        plt.show()

    def plot_acceleration_components(trajs, styles, labels, filename=None):
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))
        for traj, style, label in zip(trajs, styles, labels):
            s = [s.p.s for s in traj]
            a_longitudinal = [s.a.a1 for s in traj]
            a_lateral = [s.a.a2 for s in traj]
            ax1.plot(s, a_longitudinal, style, label=label)
            ax2.plot(s, a_lateral, style, label=label)
        ax1.set_ylabel('Longitudinal Acceleration (m/s^2)')
        ax2.set_ylabel('Lateral Acceleration (m/s^2)')
        ax2.set_xlabel('Distance (s)')
        ax1.legend()
        ax1.grid(True)
        ax2.grid(True)
        if filename:
            plt.savefig(filename)
        plt.show()

    def plot_normal_force(trajs, styles, labels, filename=None):
        plt.figure()
        for traj, style, label in zip(trajs, styles, labels):
            s = [s.p.s for s in traj]
            N = [s.fb.f3 for s in traj]
            plt.plot(s, N, style, label=label)
        plt.xlabel('Distance (s)')
        plt.ylabel('Normal Force (N)')
        plt.legend()
        plt.grid(True)
        if filename:
            plt.savefig(filename)
        plt.show()

    def plot_slip_angles(trajs, styles, labels, filename=None):
        plt.figure()
        for traj, style, label in zip(trajs, styles, labels):
            s = [s.p.s for s in traj]
            slip_front = [s.tfl.alpha for s in traj]
            slip_rear = [s.trl.alpha for s in traj]
            plt.plot(s, slip_front, style, label=label + ' Front')
            plt.plot(s, slip_rear, style, label=label + ' Rear')
        plt.xlabel('Distance (s)')
        plt.ylabel('Slip Angle (rad)')
        plt.legend()
        plt.grid(True)
        if filename:
            plt.savefig(filename)
        plt.show()

    plot_trajectory([kine_mpc_traj, dyna_mpc_traj], [':r', '--g'], ['Kinematic', 'Dynamic'], filename=f'barc3d/results/trajectory_{surface_name}_v{speed_plot}.png')
    plot_control_inputs([kine_mpc_traj, dyna_mpc_traj], [':r', '--g'], ['Kinematic', 'Dynamic'], filename=f'barc3d/results/control_inputs_{surface_name}_v{speed_plot}.png')
    plot_velocity_components([kine_mpc_traj, dyna_mpc_traj], [':r', '--g'], ['Kinematic', 'Dynamic'], filename=f'barc3d/results/velocity_components_{surface_name}_v{speed_plot}.png')
    plot_acceleration_components([kine_mpc_traj, dyna_mpc_traj], [':r', '--g'], ['Kinematic', 'Dynamic'], filename=f'barc3d/results/acceleration_components_{surface_name}_v{speed_plot}.png')
    plot_normal_force([kine_mpc_traj, dyna_mpc_traj], [':r', '--g'], ['Kinematic', 'Dynamic'], filename=f'barc3d/results/normal_force_{surface_name}_v{speed_plot}.png')
    #plot_slip_angles([kine_mpc_traj, dyna_mpc_traj], [':r', '--g'], ['Kinematic', 'Dynamic'], filename=f'barc3d/results/slip_angles_{surface_name}_v{speed_plot}.png')



    
    

    