import numpy as np
from casadi import *
from casadi.tools import *
import pdb
import sys
import do_mpc
import rospy
import time_optimal_mpc.utils.interpolation #TODO shouldnt that be just be import utils.interpolation? doesnt work when importing this from node
import time_optimal_mpc.utils.obstacle_parameterization

def template_mpc(model, parameter, interpolation, width, obstacles):
    # Obtain an instance of the do-mpc MPC class
    # and initiate it with the model:
    mpc = do_mpc.controller.MPC(model)

    # Set parameters:
    setup_mpc = {
        'n_horizon':    parameter['mpc']['n_horizon'],
        'n_robust':     0,
        't_step':       parameter['mpc']['t_step'],
        'nlpsol_opts': {
            #'ipopt.linear_solver': 'MA27', # change linear solver from mumps to MA27: https://www.do-mpc.com/en/latest/api/do_mpc.controller.MPC.set_param.html
            'ipopt.print_level':0, 'ipopt.sb': 'yes', 'print_time':0, # supress IPOPT output
            'ipopt.max_iter': parameter['mpc']['max_iter'] # TODO test if higher/lower iteration count makes performance/driving better 
        }
    }
    mpc.set_param(**setup_mpc)

    _x = model.x
    _p = model.p
    _u = model.u
    _tvp = model.tvp

    # reference Time, should be not reachable, so mpc will calculate optimal possible time
    # v_max/horizon_distance => not reachable time
    T_ref = (parameter['mpc']['n_horizon'] * parameter['mpc']['t_step']) / (parameter['model']['v_max'])

    # Configure objective function:
    # J = sum(lterm + rterm) + mterm
    mterm = 1*(_x['t'] - T_ref)**2   # terminal cost: time optimal driving = (T - Tref)^2
    # lterm = 0.1*(_x['e_y'])**2 + 0*(_x['e_psi'])**2 # stage cost: for following the centerline, the heiger the weigth on this the less timeoptimal we will drive
    lterm = SX(0)   # stage cost: time optimal driving = 0, not 0 for other objectives, like followiong the centerline

    mpc.set_objective(mterm=mterm, lterm=lterm)
    mpc.set_rterm(D=0., delta = 0.) # TODO penalty factor for input change delta_u = u(k) - u(k-1)

    # State and input bounds:

    # no backwards driving on track
    mpc.bounds['lower', '_x', 'e_psi'] = np.deg2rad(-90.)
    mpc.bounds['upper', '_x', 'e_psi'] = np.deg2rad(90.)

    # v > 0 and v_max = 20
    mpc.bounds['lower', '_x', 'v'] = 0.
    mpc.bounds['upper', '_x', 'v'] = parameter['model']['v_max']

    # no negative time
    mpc.bounds['lower', '_x', 't'] = 0.

    # input constraints
    mpc.bounds['lower', '_u', 'D'] = 0.

    mpc.bounds['lower', '_u', 'delta'] = parameter['model']['s_min']
    mpc.bounds['upper', '_u', 'delta'] = parameter['model']['s_max']

    mpc.bounds['lower', '_z', 'v_dot'] = -parameter['model']['a_max']
    mpc.bounds['upper', '_z', 'v_dot'] = parameter['model']['a_max']

    # General (differentiable) nonlinear inequalities m(x,u,z,p) < m_ub

    # constraint for cornering speed with mu from sim
    mu = parameter['model']['mu']
    l = parameter['model']['lr'] + parameter['model']['lf']
    g = parameter['model']['g']
    mpc.set_nl_cons('corner_speed_limit', _x['v']**2 * fabs(_u['delta']/l) - mu * g, ub=0, soft_constraint=False)
    
    # Stay between track limits
    # |e_y| < (track_width - car_width)/2 - safety_distance =>  |e_y| - (track_width - car_width)/2 + safety_distance  < 0
    # Avoid obstacles
    # distance of e_y to e_ob grater than half of widths + safety_distance
    # |e_y - e_ob| > (ob_width + car_width)/2 + safety_distance => -|e_y - e_ob| + (ob_width + car_width)/2 + safety_distance < 0

    car_width = parameter['model']['width']
    safety_distance = parameter['mpc']['safety_distance']
    track_width = width

    # Track limits
    mpc.set_nl_cons('track_lim', fabs(_x['e_y']) - (track_width - car_width)/2 + safety_distance , ub=0, soft_constraint=False)

    # Avoid obstacles
    mpc.set_nl_cons('ob_lb', -fabs(_x['e_y'] - _tvp['e_ob']) + (_tvp['ob_width'] + car_width)/2 + safety_distance  , ub=0, soft_constraint=False)

    # Set time varying parameters of model
    n_horizon = mpc.n_horizon               # Horizon length
    s_step = mpc.t_step                     # Step size
    tvp_template = mpc.get_tvp_template()

    def tvp_fun(s_now):
        t0 = rospy.get_rostime()

        for k in range(n_horizon+1):
            # delta_s_pred = k*s_step # calculate each s that will be predicted over horizon from current s
            # t2 = rospy.get_rostime()

            # TODO calculate obstacles bounds for each s_pred here!
            # ob_width = (ob_ub - ob_lb)/2  => width of obstacle
            # e_ob = ob_lb + ob_width/2     => error of obstacle to center of track
            try: 
                widths, e_obs_list = obstacles.getParameterisation(interpolation, k, s_step)

                if len(widths) == 0:
                    e_ob_s = 2*obstacles.min_width
                    ob_width_s = 0
                else:
                    e_ob_s = e_obs_list[0]
                    ob_width_s = widths[0]
            except Exception as e: # work on python 3.x
                print('Exception in Obstacles: ' + str(e))
                e_ob_s = 10
                ob_width_s = 0
            

            # e_ob_s = 10
            # ob_width_s = 0
            tvp_template['_tvp',k,'rho_sigma'] = 1 / interpolation.getLocalCurvatureOfCurveSingle(k, s_step) # TODO maybe replace with getLocalCurvatureOfCurveMultiple if it has better performance?
            tvp_template['_tvp',k,'e_ob'] = e_ob_s
            tvp_template['_tvp',k,'ob_width'] = ob_width_s


        # t1 = rospy.get_rostime()
        # rospy.logwarn('[MPC]===Timing================')
        # rospy.logwarn('[MPC] TVP time:    %s[ms]', (t1-t0).to_sec()*1000)

        return tvp_template

    mpc.set_tvp_fun(tvp_fun)

    mpc.prepare_nlp()

    # Create new constraint: Input delta change needs to satisfy delta_delta < sv_min/max
    # we need s_dot and s_step to convert time dependent constraints to space dependent ones
    # delta_delta(k+1) = delta(k+1) - delta(k)                              => unit: deg/(s_step*m)
    # delta_delta_max = sv_max/s_dot * s_step (same with min)               => unit: deg/(s_step*m)
    # |delta_delta| < |delta_delta_max|
    # => | (delta(k+1) - delta(k)) / s_step * s_dot | < |sv_max|            => unit: deg/s
    # or alternativly
    # => sv_min < (delta(k+1) - delta(k)) / s_step * s_dot < sv_max         => unit: deg/s
    delta_delta = ((mpc.opt_x['_u',0, 0, 'delta'] - mpc.opt_p['_u_prev', 'delta']) / s_step) * mpc.opt_x['_z', 0, 0, 0 , 's_dot']
    mpc.nlp_cons.append(delta_delta)

    # # Create appropriate upper and lower bound
    sv_min = parameter['model']['sv_min']
    sv_max = parameter['model']['sv_max']

    mpc.nlp_cons_lb.append(np.matrix([[sv_min]]))
    mpc.nlp_cons_ub.append(np.matrix([[sv_max]]))

    for k in range(n_horizon-1):

        # Create new constraint: Input delta change needs to satisfy delta_delta < sv_min/max
        delta_delta = ((mpc.opt_x['_u', k + 1, 0, 'delta'] - mpc.opt_x['_u',k, 0, 'delta']) / s_step) * mpc.opt_x['_z', k, 0, 0 , 's_dot']
        mpc.nlp_cons.append(delta_delta)

        # Create appropriate upper and lower bound
        mpc.nlp_cons_lb.append(np.matrix([[sv_min]]))
        mpc.nlp_cons_ub.append(np.matrix([[sv_max]]))

    mpc.create_nlp()

    return mpc