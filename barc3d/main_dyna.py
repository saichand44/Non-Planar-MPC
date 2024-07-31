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
   
    kine_mpc_traj = run_solo_lap(mpc_nonplanar_kinematic, nonplanar_model, surf, figure = figure, plot = True, lap = 'nonplanar mpc_kinematic')
    dyna_mpc_traj = run_solo_lap(dynamic_mpc, dynamic_model, surf, figure = figure, plot = True, lap = 'planar mpc_dyna')

        
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
        
        ax5.fill_between([min(s), max(s)], [mpc.config.Nmax, mpc.config.Nmax], [mpc.config.Nmin, mpc.config.Nmin], color = 'green',alpha = 0.3)
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



    figure.close()  # makes sure the second process stops




