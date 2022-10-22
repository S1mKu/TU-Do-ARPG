from casadi import *
from casadi.tools import *
import imp
import numpy as np
import pdb
import sys
import do_mpc

def template_model(parameter):
    # Obtain an instance of the do-mpc model class
    # and select time discretization:
    model_type = 'continuous' # either 'discrete' or 'continuous'
    model = do_mpc.model.Model(model_type)

    # Introduce new states, inputs and other variables to the model, e.g.:
    e_y = model.set_variable(var_type='_x', var_name='e_y', shape=(1,1))
    e_psi = model.set_variable(var_type='_x', var_name='e_psi', shape=(1,1))
    v = model.set_variable(var_type='_x', var_name='v', shape=(1,1))
    t = model.set_variable(var_type='_x', var_name='t', shape=(1,1))

    D = model.set_variable(var_type='_u', var_name='D')             # desired velocity
    delta = model.set_variable(var_type='_u', var_name='delta')

    v_dot = model.set_variable(var_type='_z', var_name='v_dot')
    s_dot = model.set_variable(var_type='_z', var_name='s_dot')

    # Parameter for drivable corridor
    e_ob= model.set_variable(var_type='_tvp', var_name='e_ob')          # error of middle of obstacle to middle of the track
    ob_width= model.set_variable(var_type='_tvp', var_name='ob_width')  # width of obstacle

    # local curvature of sigma, precomputed based on sigma
    rho_sigma= model.set_variable(var_type='_tvp', var_name='rho_sigma')

    # set physical constants
    lr      = parameter['model']['lr']    # distance rear wheels to center of gravitiy of the car
    lf      = parameter['model']['lf']    # distance front wheels to center of gravitiy of the car
    m       = parameter['model']['m']     # mass of the car
    C_1     = (lr / (lr + lf))   # geometry for turning circle radius    0.5
    C_2     = (1 / (lr + lf))    # geometry for angle change of car      17.06
    # TODO sim uses other model, either change prediction model or determine parameters
    # C_m1    = parameter['model']['C_m1']  # independent motor parameter           12.0
    # C_m2    = parameter['model']['C_m2']  # speed-dependent motor parameter       2.17
    # C_r2    = parameter['model']['C_r2']  # second order friction parameter       0.1
    # C_r0    = parameter['model']['C_r0']  # zero order friction parameter         0.6
    
    # compute variables for car dynamics
    v_x = v                                                                         # assumed speed along sigma
    v_y = v * C_1 * delta                                                           # assumed speed orthogonal to sigma
    psi_dot = v * delta * C_2                                                       # angle change based on physical model
    # v_dot = ( C_m1 - C_m2*v ) * D - C_r2 * v**2 - C_r0 - (v*delta)**2 * C_2 * C_1**2   # speed change based on physical model

    # s_dot = ( 1 / ( 1-(e_y/rho_sigma) ) ) * ( v_x * cos(e_psi) - v_y * sin(e_psi))
    # 0 = ( 1 / ( 1-(e_y/rho_sigma) ) ) * ( v_x * cos(e_psi) - v_y * sin(e_psi))- s_dot
    model.set_alg('s_dot', (( 1 / ( 1-(e_y/rho_sigma) ) ) * ( v_x * cos(e_psi) - v_y * sin(e_psi))) - s_dot)

    kappa_sigma = 1 / rho_sigma

    # Calculate desired acceleration from desired velocity
    vel_diff = D - v
    max_a = parameter['model']['a_max']
    max_v = parameter['model']['v_max']
    min_v = parameter['model']['v_min']

    kp = 10.0 * max_a / (-min_v)
    
    # v_dot = kp * vel_diff
    # 0 = kp * vel_diff - v_dot
    model.set_alg('v_dot', kp * vel_diff - v_dot) #  TODO see if we can make this here better somehow? Currently we use kp values for decelerating only, maybe we can make a casadi function that knows when we decelerate/accelerate, but this has lower prio i guess

    # # currently forward
    # if v > 0.:
    #     if (vel_diff > 0):
    #         # accelerate
    #         kp = 10.0 * max_a / max_v
    #         v_dot = kp * vel_diff
    #     else:
    #         # braking
    #         kp = 10.0 * max_a / (-min_v)
    #         v_dot = kp * vel_diff
            
    # # currently backwards
    # else:
    #     if (vel_diff > 0):
    #         # braking
    #         kp = 2.0 * max_a / max_v
    #         v_dot = kp * vel_diff
    #     else:
    #         # accelerating
    #         kp = 2.0 * max_a / (-min_v)
    #         v_dot = kp * vel_diff

    # # positive accl limit
    # v_switch = parameter['model']['v_switch']
    # if v > v_switch:
    #     pos_limit = max_a*v_switch/v
    # else:
    #     pos_limit = max_a

    # # accl limit reached?
    # if (v <= min_v and v_dot <= 0) or (v >= max_v and v_dot >= 0):
    #     v_dot = 0.
    # elif v_dot <= -max_a:
    #     v_dot = -max_a
    # elif v_dot >= pos_limit:
    #     v_dot = pos_limit

    # Set right-hand-side of ODE for all introduced states (_x).
    # Names are inherited from the state definition.
    model.set_rhs('e_y', (v_x * sin(e_psi) + v_y * cos(e_psi)) / s_dot)
    model.set_rhs('e_psi', psi_dot / s_dot - kappa_sigma)
    model.set_rhs('v', v_dot / s_dot )
    model.set_rhs('t', 1 / s_dot )

    # Setup model:
    model.setup()

    return model