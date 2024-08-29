import numpy as np
from numpy import pi as pi
import matplotlib.pyplot as plt

from barc3d.surfaces.base_surface import INTERP_PARTIAL_PCHIP

# centerline surfaces
from barc3d.surfaces.frenet_surface import FrenetSurfaceConfig, FrenetSurface
from barc3d.surfaces.tait_bryan_surface import TaitBryanSurfaceConfig, TaitBryanAngleSurface
from barc3d.surfaces.arc_profile_centerline_surface import ArcProfileCenterlineSurface, ArcProfileSurfaceConfig

def generate_surfaces():

    ### FULL SCALE TRACKS:

    """ # l_track
    surf = FrenetSurface()
    ds = np.array([0,4,pi,2,pi/2,2,pi,4,pi/2]) * 10
    s = np.cumsum(ds)
    a = np.array([0,0,np.pi, np.pi, np.pi/2, np.pi/2, 3*np.pi/2, 3*np.pi/2, np.pi*2])
    config = FrenetSurfaceConfig(s = s, a = a, y_min = -4, y_max = 4, use_pchip = False, closed = True)
    surf.initialize(config)   
    surf.save_surface('l_track') """
    

    # l_track
    surf = TaitBryanAngleSurface()
    ds = np.array([0,4,pi,2,pi/2,2,pi,4,pi/2]) * 10
    s = np.cumsum(ds)
    a = np.array([0,0,np.pi, np.pi, np.pi/2, np.pi/2, 3*np.pi/2, 3*np.pi/2, np.pi*2])
    b = np.array([0,0,0,0,0,0,0,0,0])
    c = np.array([0,0,0,0,0,0,0,0,0])
    config = TaitBryanSurfaceConfig(s = s, a = a, b = b, c = c, y_min = -4, y_max = 4, use_pchip = False)
    surf.initialize(config)
    surf.save_surface('l_track')
    #surf.plot_curvature()

    ''' an L-shaped track '''
    ds = np.array([0,4,pi,2,pi/2,2,pi,4,pi/2]) * 10
    s = np.cumsum(ds)
    a = np.array([0,0,np.pi, np.pi, np.pi/2, np.pi/2, 3*np.pi/2, 3*np.pi/2, np.pi*2])
    config = TaitBryanSurfaceConfig(s = s, a = a, b = b, c = c, y_min = -4, y_max = 4, use_pchip = False, closed = False)
    surf.initialize(config)
    surf.save_surface('l_track_2') 
    
    ''' a plus-shaped track '''
    ds = np.array([0,2,4,4,2,4,4,4,2,4,4,4,2,4,4,4,2,2]) * 10
    s = np.cumsum(ds)
    a = np.array([0,0,np.pi, np.pi, np.pi/2,np.pi/2, 1.5*np.pi, 1.5*np.pi, np.pi, np.pi, 2*np.pi, 2*np.pi, 1.5*np.pi, 1.5*np.pi, 2.5*np.pi, 2.5*np.pi, 2*np.pi, 2*np.pi])
    config = TaitBryanSurfaceConfig(s = s, a = a, b = b, c = c, y_min = -4, y_max = 4, use_pchip = False, closed = False)
    surf.save_surface('plus_track')
    
    # itsc
    surf = TaitBryanAngleSurface()
    ds = np.array([0,50, 15,    40,   15,  30,      20,    20,   20,   20,     30,   30            , 10,       30,       10,    10,  10, 20,  15,15,20]) 
    s  = np.cumsum(ds)
    a  = np.array([0, 0, 0 ,   pi ,  pi,  pi/2,   2.5,     2.5,     2.5 ,  pi/2,     pi/2,  pi  ,  pi,       pi/2,    pi/2,   0,  0,  0, 0, 0,0])
    b  = np.array([0, 0, 0,     0,   0  , 0,      pi/2,    pi,    3*pi/2, 2*pi,    2*pi,   2*pi , 2*pi+1  , 2*pi,  2*pi,  2*pi, 2*pi, 2*pi, 1.7*pi, 2*pi,2*pi] )
    c  = np.array([0, 0, -pi/5, -pi/5, 0,   0,      0,    0,    0,         0,       0,     0,      0.3,        0.1,   -0.3, -0.7,   0,  0  , 0, 0,0])
    config = TaitBryanSurfaceConfig(s = s, a = a, b = b, c = c, y_min = -4, y_max = 4, use_pchip = False, closed = False)
    surf.initialize(config)   
    surf.save_surface('itsc')
    #surf.plot_curvature()

    # single_bump
    surf = TaitBryanAngleSurface()
    ds = np.array([0, 2, 1, 1, 2]) *10
    s = np.cumsum(ds)
    a = np.array([0, 0, 0, 0, 0])
    b = np.array([0, 0.5, 0, -0.5, 0]) 
    c = np.array([0, 0, 0, 0, 0])
    config = TaitBryanSurfaceConfig(s = s, a = a, b = b, c = c, y_min = -4, y_max = 4, use_pchip = False, closed = False)  
    surf.initialize(config)
    surf.save_surface('single_bump')
    #surf.plot_curvature()

    # 3D L track
    surf = TaitBryanAngleSurface()
    ds = np.array([0, 2, 1, 1, pi, 2, pi/2, 2, pi, 4, pi/2]) * 10
    s = np.cumsum(ds)
    a = np.array([0, 0, 0, 0, pi, pi, pi/2, pi/2, 3*pi/2, 3*pi/2, pi*2])
    b = np.array([0, 0.3, -0.5, 0, 0, 0, 0, 0, 0, 0, 0.05])
    c = np.array([0, 0, 0, 0, 0, 0, 0, -pi/6, -pi/6, 0, 0])
    config = TaitBryanSurfaceConfig(s = s, a = a, b = b, c = c, y_min = -4, y_max = 4, use_pchip = False)
    surf.initialize(config)
    surf.save_surface('l_3d')
    #surf.plot_curvature()

    ''' a simple twist '''
    ds  = np.array([0,10, 10,     10,     10,      10,      10,      10,      10,       15,     20,       10])
    s = np.cumsum(ds)
    a = np.array([0,  0,      pi/4,   pi/2,   pi/2,   pi/4,   0,      0,      0,      -pi/4,  -pi/2,  -pi/2])
    b = np.array([0,  0,      0,      0,      0.2,    0.4,    0.5,    0.4,    0.25,    0.1,     0,      0])
    c = np.array([0,  -0.2,   -0.3,   -0.2,   0,      0.2,      0.3,    0.2,    0,      0.2,     0,      0])
    config = TaitBryanSurfaceConfig(s = s, a = a, b = b, c = c, y_min = -4, y_max = 4, use_pchip = False, closed=False)
    surf.initialize(config)
    surf.save_surface('twist')

    ''' a simple hill climb '''
    ds  = np.array([0,10, 10,     10,     10,      10,      10,      10,      10,       15,     20,       10])
    s = np.cumsum(ds)
    a = np.array([0,  0,      pi/2,   pi/2,   pi/2,   pi/4,   0,      0,      0,      -pi/2,  -pi/2,  -pi/2])
    b = np.array([0,  0,      0,      0,      0.2,    0.4,    0.5,    0.4,    0.25,    0.1,     0,      0])
    c = np.array([0,  -0.2,   -0.3,   -0.2,   0,      0.2,      0.3,    0.2,    0,      0.2,     0,      0])
    config = TaitBryanSurfaceConfig(s = s, a = a ,b = b, c = c, y_min = -4, y_max = 4)
    surf.initialize(config)
    surf.save_surface('hill_climb')
    
    ''' a single off camber turn '''
    ds = np.array([0, 10,20,20,20,10])
    s = np.cumsum(ds)
    a = np.array([0,0,0,pi/3, pi/3, pi/3])
    b = 0 * a
    c = np.array([0,0,pi/8,pi/8,0,0])
    config = TaitBryanSurfaceConfig(s = s, a = a ,b = b, c = c, y_min = -4, y_max = 4)
    surf.initialize(config)
    surf.save_surface('off_cam_turn')

    ''' a chicane with on-camber turns '''
    ds = np.array([0, 20,20,   15, 15,   40,   15, 15,20])
    s  = np.cumsum(ds)
    a  = np.array([0, 0, 0,    -pi/6, -pi/3, -pi/3, -pi/6, 0, 0])
    b  = a * 0
    c  = np.array([0, 0, pi/12, pi/6, pi/12, -pi/12, -pi/6, -pi/12, 0])
    config = TaitBryanSurfaceConfig(s = s, a = a ,b = b, c = c, y_min = -6, y_max = 6)
    surf.initialize(config)
    surf.save_surface('chicane')

    
    
    # tube turn
    surf = ArcProfileCenterlineSurface()
    config = ArcProfileSurfaceConfig()
    config.s = np.array([0,15,30,50,65,80])*3
    config.a = np.array([0,0,0, np.pi, np.pi,np.pi])
    config.b = np.array([0,0,0,0,0,0])
    config.c = np.array([0,0,0,0,0,0])
    config.k = np.array([.001,.001,.099,0.099,.001,.001])
    config.y_max = 2
    config.y_min = -8
    config.use_pchip = True
    surf.initialize(config)
    surf.save_surface('tube_turn')

    ''' an S-bend with tubular profile '''
    config = ArcProfileSurfaceConfig()
    ds = np.array([0,10,10,20,10,20,10,10])
    config.s = np.cumsum(ds) * 3
    config.a = np.array([0,0,0, np.pi, np.pi,0,0,0])
    config.b = np.array([0,0,0,0,0,0,0,0])
    config.c = np.array([0,0,0,0,0,0,0,0])
    config.k = np.array([.001,.001,.099,0.099,0.099,0.099,.001,.001])
    config.y_max = 10
    config.y_min = -10
    config.use_pchip = True
    surf.initialize(config)
    surf.save_surface('s_bend')

    ''' a quarter pipe turn that loops back '''
    config = ArcProfileSurfaceConfig()
    ds = np.array([0,10,10,20,10,10,5,20,5])
    config.s = np.cumsum(ds) * 3
    config.a = np.array([0,0,0, np.pi, np.pi,np.pi,np.pi,2*np.pi,2*np.pi])
    config.b = np.array([0,0,0,0,0,0,0,0,0])
    config.c = np.array([0,0,0,0,0,0,0,0,0])
    config.k = np.array([.001,.001,.099,0.099,.001,.001,.001,.001,.001])
    config.y_max = 0
    config.y_min = -10
    config.closed = True
    config.use_pchip = True
    surf.initialize(config)
    surf.save_surface('tube_turn_loop')

    

    ### SMALL SCALE TRACKS (1:10):

    """ surf = FrenetSurface()
    ds = np.array([0,4,pi,2,pi/2,2,pi,4,pi/2]) * 1  # Scaled down by 1/10
    s = np.cumsum(ds)
    a = np.array([0,0,np.pi, np.pi, np.pi/2, np.pi/2, 3*np.pi/2, 3*np.pi/2, np.pi*2])
    config = FrenetSurfaceConfig(s = s, a = a, y_min = -0.4, y_max = 0.4, use_pchip = False, closed = True)  # y_min and y_max scaled down
    surf.initialize(config)   
    surf.save_surface('l_track')
     
    # l_track_10
    surf = TaitBryanAngleSurface()
    ds = np.array([0,4,pi,2,pi/2,2,pi,4,pi/2]) * 1  # Scaled down by 1/10
    s = np.cumsum(ds)
    a = np.array([0,0,np.pi, np.pi, np.pi/2, np.pi/2, 3*np.pi/2, 3*np.pi/2, np.pi*2])
    b = np.array([0,0,0,0,0,0,0,0,0])
    c = np.array([0,0,0,0,0,0,0,0,0])
    config = TaitBryanSurfaceConfig(s = s, a = a, b = b, c = c, y_min = -0.4, y_max = 0.4, use_pchip = False, closed = True)  # y_min and y_max scaled down
    surf.initialize(config)
    surf.save_surface('l_track_10')
    #surf.plot_curvature()

    # itsc_10
    surf = TaitBryanAngleSurface()
    ds = np.array([0,5, 1.5, 4, 1.5, 3, 2, 2, 2, 2, 3, 3, 1, 3, 1, 1, 1, 2, 1.5, 1.5, 2])  # Scaled down by 1/10
    s  = np.cumsum(ds)
    a  = np.array([0, 0, 0 , pi , pi, pi/2, 2.5, 2.5, 2.5 , pi/2, pi/2, pi, pi, pi/2, pi/2, 0, 0, 0, 0, 0, 0])
    b  = np.array([0, 0, 0, 0, 0, 0, pi/2, pi, 3*pi/2, 2*pi, 2*pi, 2*pi, 2*pi+1, 2*pi, 2*pi, 2*pi, 2*pi, 2*pi, 1.7*pi, 2*pi, 2*pi])
    c  = np.array([0, 0, -pi/5, -pi/5, 0, 0, 0, 0, 0, 0, 0, 0, 0.3, 0.1, -0.3, -0.7, 0, 0, 0, 0, 0])
    config = TaitBryanSurfaceConfig(s = s, a = a ,b = b, c = c, y_min = -0.4, y_max = 0.4, use_pchip = False)  # y_min and y_max scaled down
    surf.initialize(config)   
    surf.save_surface('itsc_10')
    
    
    # tube turn_10
    surf = ArcProfileCenterlineSurface()
    config = ArcProfileSurfaceConfig()
    config.s = np.array([0,1.5,3,5,6.5,8])*0.3  # Scaled down by 1/10
    config.a = np.array([0,0,0, pi, pi, pi])
    config.b = np.array([0,0,0,0,0,0])
    config.c = np.array([0,0,0,0,0,0])
    config.k = np.array([.001,.001,.099,0.099,.001,.001])
    config.y_max = 0
    config.y_min = -1  # Scaled down by 1/10
    config.use_pchip = True
    surf.initialize(config)
    surf.save_surface('tube_turn_10') 

    # l_3d_10
    surf = TaitBryanAngleSurface()
    ds = np.array([0, 0.05, 0.1, 0.1, 0.2, pi, 0.2, pi/2, 0.02, 0.08, 0.08, 0.02, pi, 0.4, pi/2])  # Scaled down by 1/10
    s  = np.cumsum(ds)
    a  = np.array([0, 0, 0, 0, 0, pi, pi, pi/2, pi/2, pi/2, pi/2, pi/2, 3*pi/2, 3*pi/2, pi*2])
    b  = np.array([0,0, pi/6, -pi/6,0,0,0, 0, 0, 0.35, -0.35, 0, 0, 0.0544, 0])
    c  = np.array([0, 0, 0, 0,-0.85, -0.85, 0, 0,0, 0, 0, 0, 0, 0, 0])
    config = TaitBryanSurfaceConfig(s = s, a = a ,b = b, c = c, y_min = -0.4, y_max = 0.4, use_pchip = False, closed = True)  # y_min and y_max scaled down
    surf.initialize(config)   
    surf.save_surface('l_3d_10')
    #surf.plot_curvature()

    # single_bump
    surf = TaitBryanAngleSurface()
    ds = np.array([0, 0.2, 0.1, 0.1, 0.2])  # Scaled down by 1/10
    s = np.cumsum(ds)
    a = np.array([0, 0, 0, 0, 0])
    b = np.array([0, 0.05, 0, -0.05, 0])
    c = np.array([0, 0, 0, 0, 0])
    config = TaitBryanSurfaceConfig(s = s, a = a, b = b, c = c, y_min = -0.4, y_max = 0.4, use_pchip = False, closed = False)  # y_min and y_max scaled down
    surf.initialize(config)
    surf.save_surface('single_bump_10') """

    
    
if __name__ == '__main__':
    generate_surfaces()
    # plot single bump surface
    

    
