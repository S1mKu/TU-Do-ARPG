
import numpy as np
import casadi
import forcespro
import forcespro.nlp
import config


hardware = False

def continuous_dynamics_dynamic(x, u, p):
    # Defines dynamics of the car.
    # parameters:
    # state x = [e_y, e_psi, v, delta, yaw_rate, slip_angle, t]
    # input u = [a, delta_dot, s_j]
    # parameter p = [rho_sigma, track_width, e_ob, ob_width]
    
    nu = len(config.I['inputs']) # number of inputs to shift indices of states

    # get state and input
    e_y         = x[config.I['err_y'] - nu]  # Deviation from centerline
    e_psi       = x[config.I['err_psi'] - nu]  # Yaw angle relative to path
    v           = x[config.I['velocity'] - nu]  # Velocity
    delta       = x[config.I['steer_angle'] - nu]  # Steering angle
    psi_dot     = x[config.I['yaw_angle'] - nu]  # Yaw rate of car    
    beta        = x[config.I['beta'] - nu]  # slip angle at vehicle center
    t           = x[config.I['time'] - nu]  # Time
    a           = u[config.I['accel']]  # acceleration
    delta_dot   = u[config.I['steer_rate']]  # Steering rate


    # set physical constants
    l_r     = config.l_r            # distance rear wheels to center of gravitiy of the car
    l_f     = config.l_f            # distance front wheels to center of gravitiy of the car
    C_1     = (l_r / (l_r + l_f))   # geometry for turning circle radius    0.5
    C_2     = (1 / (l_r + l_f))     # geometry for angle change of car      17.06
    g       = config.g              # gravity constant m/s^2
    mu      = config.mu             # friction coefficient
    C_Sf    = config.C_Sf           # cornering stiffness front
    C_Sr    = config.C_Sr           # cornering stiffness rear 
    h       = config.h              # height of center of gravity
    m       = config.m              # mass of car
    I       = config.I              # Inertia

    # compute variables for car dynamics
    v_x = v * casadi.cos(beta)
    v_y = v * casadi.sin(beta)


    rho_sigma = p[0]  # local radius of sigma, precomputed based on sigma

    s_dot = (1 / (1-(e_y/rho_sigma))) * (v_x * casadi.cos(e_psi) - v_y * casadi.sin(e_psi))

    kappa_sigma = 1 / rho_sigma

    F_f = C_Sf * (g * l_r - a * h)
    F_r = C_Sr * (g * l_f + a * h)

    psi_dot_dot = (mu *m)/(I * (l_r + l_f)) (l_f * F_f * delta - (l_r * F_r + l_f * F_f) * beta + (l_r**2 * F_r - l_f**1 * F_f) * (psi_dot)/(v))
    beta_dot = (mu)/(v * (l_r + l_f)) (F_f * delta - (F_r + F_f) * beta + (F_r * l_r - F_f * l_f) * (psi_dot)/(v)) - (psi_dot)

    # compute derivatives for spatial dynamic system
    return casadi.vertcat(  
        ( v_x * casadi.sin(e_psi) + v_y * casadi.cos(e_psi) ) / s_dot ,       # e_y'
        psi_dot / s_dot - kappa_sigma                                           ,       # e_psi'
        a / s_dot                                                               ,       # v'
        delta_dot / s_dot                                                       ,       # delta'
        psi_dot_dot / s_dot                                                     ,       # psi_dot'
        beta_dot / s_dot                                                        ,       # delta'
        1 / s_dot                                                                       # t'
    )

def continuous_dynamics(x, u, p):
    # Defines dynamics of the car.
    # parameters:
    # state x = [e_y, e_psi, v, delta, t]
    # input u = [a, delta_dot, s_j]
    # parameter p = [rho_sigma, track_width, e_ob, ob_width]
    
    nu = len(config.I['inputs']) # number of inputs to shift indices of states

    # get state and input
    e_y         = x[config.I['err_y'] - nu]  # Deviation from centerline
    e_psi       = x[config.I['err_psi'] - nu]  # Yaw angle relative to path
    v           = x[config.I['velocity'] - nu]  # Velocity in x direction
    delta       = x[config.I['steer_angle'] - nu]  # Steering angle
    t           = x[config.I['time'] - nu]  # Time
    a           = u[config.I['accel']]  # acceleration
    delta_dot   = u[config.I['steer_rate']]  # Steering rate

    # set physical constants
    l_r     = config.l_r               # distance rear wheels to center of gravitiy of the car
    l_f     = config.l_f              # distance front wheels to center of gravitiy of the car
    C_1     = (l_r / (l_r + l_f))   # geometry for turning circle radius    0.5
    C_2     = (1 / (l_r + l_f))     # geometry for angle change of car      17.06

    # compute variables for car dynamics
    v_x = v                                                                             # assumed speed along sigma
    v_y = v * C_1 * delta                                                               # assumed speed orthogonal to sigma
    psi_dot = v * delta * C_2                                                           # angle change based on physical model

    rho_sigma = p[0]  # local radius of sigma, precomputed based on sigma

    s_dot = ( 1 / ( 1-(e_y/rho_sigma) ) ) * ( v_x * casadi.cos(e_psi) - v_y * casadi.sin(e_psi))

    kappa_sigma = 1 / rho_sigma

    # compute derivatives for spatial dynamic system
    return casadi.vertcat(  
        ( v * casadi.sin(e_psi) + v * delta * C_1 * casadi.cos(e_psi) ) / s_dot ,       # e_y'
        psi_dot / s_dot - kappa_sigma                                           ,       # e_psi'
        a / s_dot                                                               ,       # v'
        delta_dot / s_dot                                                       ,       # delta'
        1 / s_dot                                                                       # t'
    )

def obj(z, p):
    """Objective function for time optimal driving (T-Tref)^2 with T: time that the car 
    needs to complete the prediction horizon -> T = t(s + pH), so only the time 
    for the last stage is needed, so objective function is 0 for all other stages"""

    # parameters:
    # variables z = [a,  delta_dot, s_j, e_y, e_psi, v, delta, t]
    # parameter p = [rho_sigma, track_width, e_ob, ob_width]

    e_y      = z[config.I['err_y']]
    delta_dot   = z[config.I['steer_rate']]
    slack = z[config.I['slack']]

    return 0.0005*(delta_dot)**2 + 0.001*(e_y)**2 + 1 * (slack)**2

def obj2(z, p):
    """Objective function for time optimal driving (T-Tref)^2 with T: time that the car 
    needs to complete the prediction horizon -> T = t(s + pH), so only the time 
    for the last stage is needed, so objective function is 0 for all other stages"""

    # parameters:
    # variables z = [a,  delta_dot, s_j, e_y, e_psi, v, delta, t]
    # parameter p = [rho_sigma, track_width, e_ob, ob_width]

    e_y         = z[config.I['err_y']]
    e_psi       = z[config.I['err_psi']]
    v           = z[config.I['velocity']]
    v_ref       = 8.
    delta_dot   = z[config.I['steer_rate']]
    slack       = z[config.I['slack']]

    return 0.0005*(delta_dot) + 10*(e_y)**2 + 1*(e_psi)**2 + 0.1*(v - v_ref)**2 + 10 * (slack)**2 # TODO for testing, trajectory following with constant speed

def objN(z, p):
    """Objective function for time optimal driving (T-Tref)^2 with T: time the car 
    needs to complete the prediction horizon -> T = t(s + pH)"""

    # parameters:
    # variables z = [a,  delta_dot, s_j, e_y, e_psi, v, delta, t]

    # Tref = config.Tref
    # Tref = 0
    Tref = (config.N * config.integrator_ts)/config.v_ub # calculate Tref with prediction distance and max velocity
    T = z[config.I['time']]

    return (T - Tref)**2

def generate_pathplanner():
    """Generates and returns a FORCESPRO solver that calculates a path based on
    constraints and dynamics while minimizing an objective function
    """
    # Model Definition
    # ----------------

    # Problem dimensions
    model       = forcespro.nlp.SymbolicModel()
    model.N     = config.N  # horizon length
    model.nvar  = 8  # number of variables (inputs variables + system variables)
    model.neq   = 5  # number of equality constraints (system variables)
    model.nh    = 3  # number of inequality constraint functions
    
    model.npar  = 4 # number of runtime parameters
    # p[0] list of rho_sigma
    # p[1] list of track widths
    # p[2] list of obstacle errors
    # p[3] list of obstacle widths

    # Objective function
    model.objective = obj # cost 0 for non last stage
    model.objectiveN = objN # costs for the last stage

    # model.objective = obj2 # cost 0 for non last stage

    # The function must be able to handle symbolic evaluation,
    # by passing in CasADi symbols. This means certain numpy funcions are not
    # available.

    model.continuous_dynamics = continuous_dynamics

    # Moved to codeoptions because couldn't figure out how to use parameters p with this
    # We use an explicit RK4 integrator here to discretize continuous dynamics
    # integrator_stepsize = 0.1
    # model.eq = lambda z: forcespro.nlp.integrate(continuous_dynamics, z[2:6], z[0:2],
    #                                              integrator=forcespro.nlp.integrators.RK4,
    #                                              stepsize=integrator_stepsize)

    # Indices on LHS of dynamical constraint - for efficiency reasons, make
    # sure the matrix E has structure [0 I] where I is the identity matrix.
    #
    # in other words i guess: Selection matrix for system variables
    # i: 0   1   2   3   4   5
    #  | Inputs|System Var.   |
    # -----------------------------
    #    0   0   1   0   0   0        => z[2] = x[0]      
    #    0   0   0   1   0   0        => z[3] = x[1]
    #    0   0   0   0   1   0        => z[4] = x[2]
    #    0   0   0   0   0   1        => z[5] = x[3]
    model.E = np.concatenate([np.zeros((5,3)), np.eye(5)], axis=1)

    # Inequality constraints
    #  upper/lower variable bounds lb <= z <= ub
    #                     inputs               |  states
    #                     a             delta_dot       s_j          e_y             e_psi y           v           delta             t
    model.lb = np.array([config.a_lb,   config.ds_lb, -np.inf  ,config.e_y_lb,  config.e_psi_lb,  config.v_lb, config.delta_lb, config.t_lb ])
    model.ub = np.array([config.a_ub,   config.ds_ub,  np.inf  ,config.e_y_ub,  config.e_psi_ub,  config.v_ub, config.delta_ub, config.t_ub ])

    # General (differentiable) nonlinear inequalities hl <= h(z,p) <= hu

    # Stay between track limits
    # |e_y| < (track_width - car_width)/2 - safety_distance =>  0 < (track_width - car_width)/2 - safety_distance - |e_y| < inf

    # Constraint for cornering speed, with mu from sim
    # v < sqrt(mu * g * l/|delta|) => -inf < v^2 * |delta/l| - mu * g < 0

    # dont do it like this, you will divide by zero for straight driving :)
    # v < sqrt(mu * g * l/|delta|) => 0 < mu * g * l/|delta| - v^2 < inf
    
    # Avoid obstacles, distance of e_y to e_ob grater than half of widths + safety_distance
    # |e_y - e_ob| > (ob_width + car_width)/2 + safety_distance => -|e_y - e_ob| + (ob_width + car_width)/2 + safety_distance < 0

    # Parameter for Constraints
    car_width =         config.car_width
    safety_distance =   config.safety_distance
    mu =                config.mu
    l =                 config.l_r + config.l_f
    g =                 config.g

    # variables z = [a,  delta_dot, s_j, e_y, e_psi, v, delta, t]
    # parameter p = [rho_sigma, track_width, e_ob, ob_width]
    model.ineq = lambda z, p: casadi.vertcat(
                                     (p[1] - car_width)/2 - safety_distance - casadi.fabs(z[config.I['err_y']]),            # track limits
                                     z[config.I['velocity']]**2 * casadi.fabs(z[config.I['steer_angle']]/l) - mu * g,                                # cornering speed
                                     -casadi.fabs(z[config.I['err_y']] - p[2]) + (p[3] + car_width)/2 + safety_distance     # avoid obstacles
                                    )

    # Upper/lower bounds for inequalities
    #                    track_width
    model.hl = np.array([config.track_width_lb, config.corner_speed_lb, config.obstacles_lb])
    model.hu = np.array([config.track_width_ub, config.corner_speed_ub, config.obstacles_ub])

    # Initial condition on vehicle states x

    nu = len(config.I['inputs']) # number of inputs
    nvar = len(config.I['inputs']) + len(config.I['states'])  # number of variables
    model.xinitidx = range(nu,nvar) # use this to specify on which variables initial conditions
                                # are imposed

    # Solver generation
    # -----------------
    # Set solver options
    if not hardware:
        codeoptions = forcespro.CodeOptions('FORCESNLPsolver')
    else :
        codeoptions = forcespro.CodeOptions('FORCESNLPsolver_car_tx2')

        codeoptions.platform = 'NVIDIA-Cortex-A57'

    # codeoptions.solvemethod = "SQP_NLP"
    # codeoptions.sqp_nlp.reg_hessian = 1e-3
    # codeoptions.sqp_nlp.maxqps = 200

    # Integrator options
    codeoptions.nlp.integrator.type =   config.integrator_type
    codeoptions.nlp.integrator.Ts =     config.integrator_ts
    codeoptions.nlp.integrator.nodes =  config.integrator_nodes

    # tolerances
    codeoptions.nlp.TolStat = 1e-2 # inf norm tol. on stationarity
    codeoptions.nlp.TolEq = 1e-3   # tol. on equality constraints
    codeoptions.nlp.TolIneq = 1e-3 # tol. on inequality constraints
    codeoptions.nlp.TolComp = 1e-3 # tol. on complementarity

    codeoptions.maxit =         config.maxit        # Maximum number of iterations
    codeoptions.printlevel =    config.printlevel   # 2 full printlevel, set 0 for "production"
    codeoptions.optlevel =      config.optlevel     # 0 no optimization, 1 optimize for size, 
                                                    # 2 optimize for speed, 3 optimize for size & speed
                                                    # set 0 for faster compiling during testing

    # codeoptions.exportBFGS = 1
    # codeoptions.forcenonconvex = 1
    codeoptions.overwrite = 1 
    # codeoptions.timing = 1
    # codeoptions.parallel = 1
    # codeoptions.showinfo = 1
    # codeoptions.cleanup = 1

    # codeoptions.nlp.bfgs_init = 3e-3*np.identity(7) # initialization of the hessian approximation
    codeoptions.noVariableElimination = 1.
    # change this to your server or leave uncommented for using the 
    # standard embotech server at https://forces.embotech.com 
    # codeoptions.server = 'https://forces.embotech.com'
    
    # Creates code for symbolic model formulation given above, then contacts 
    # server to generate new solver
    solver = model.generate_solver(options=codeoptions)

    return model,solver

def main():
    # generate solver
    model, solver = generate_pathplanner()


if __name__ == "__main__":
    main()
