import numpy as np
from dataclasses import dataclass, field
from barc3d.pytypes import PythonMsg, VehicleState, VehicleConfig
from barc3d.control.base_controller import BaseController 
from barc3d.control.pid import PID

@dataclass
class StanleyConfig(PythonMsg):
    k_lat  :float = field(default =  1.0)  # Lateral error gain
    k_soft :float = field(default =  1.0)  # Velocity diminish offset coefficient
    kp_l   :float = field(default =  0.8)  # Longitudinal PID gains
    ki_l   :float = field(default =  0.15)
    kd_l   :float = field(default =  0.0)
    dt     :float = field(default =  0.1)

    vref: float = field(default =    10)  # target speed
    yref: float = field(default =    0)  # target centerline offset
    
class SimpleStanley(BaseController):
    def __init__(self, k_lat, k_soft, max_str, min_str, lat_offset):
        self.k_lat = k_lat
        self.k_soft = k_soft
        self.max_str = max_str
        self.min_str = min_str
        self.lat_offset = lat_offset
        return

    def is_almost_equal(self, a:float, b:float, eps:float):
        return (a > (b-eps)) and (a < (b+eps))

    def is_almost_zero(self, a:float, eps:float):
        return (a > -eps) and (a < eps)

    def step(self, state:VehicleState):
        # First compute the simple control law
        if not self.is_almost_zero(self.k_soft + state.v.v1, 1e-5):
            # Flipping sign for heading and lateral offset because
            # steering sign is flipped
            str_angle = ((-1.0)*state.p.ths) + \
                        np.arctan((self.k_lat * ((-1.0)*(state.p.y - self.lat_offset))) /
                                  (self.k_soft + state.v.v1))
        else:
            str_angle = 0.0
        # Saturate to max str ang
        return max(self.min_str,min(self.max_str, str_angle))



class SimpleStanleyPIDController():
    def __init__(self, 
                 ctrl_config:StanleyConfig = StanleyConfig(),
                 veh_config:VehicleConfig = VehicleConfig()):
        self.ctrl_config = ctrl_config
        self.veh_config = veh_config
        # Lateral controller - Simple Stanley
        self.stanley_controller = SimpleStanley(self.ctrl_config.k_lat,
                                                self.ctrl_config.k_soft,
                                                self.veh_config.y_max,
                                                self.veh_config.y_min,
                                                self.ctrl_config.yref)
        # Longitudinal controller - PID
        self.long_pid_controller = PID(self.ctrl_config.kp_l,
                                    self.ctrl_config.ki_l,
                                    self.ctrl_config.kd_l,
                                    self.ctrl_config.dt)
        self.long_pid_controller.set_ref(self.ctrl_config.vref)
        return
  
    def step(self, state:VehicleState):
        steer = self.stanley_controller.step(state)
        a = self.long_pid_controller.step(state.v.v1)
        a += state.q.e1()[2].__float__() * 9.81  # added term for current slope 

        state.u.a = a
        state.u.y = steer
        return
