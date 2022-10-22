
import numpy as np
import casadi
import forcespro
import forcespro.nlp
import config


hardware = False

def continuous_dynamics(x, u, p):
    # Defines dynamics of the car.
    # parameters:
    # state x = [e_y, e_psi, v, delta, t]
    # input u = [a, delta_dot]
    # parameter p = [rho_sigma, track_width, e_ob, ob_width]
    

    # get state and input
    e_y         = x[0]  # Deviation from centerline
    e_psi       = x[1]  # Yaw angle relative to path
    v           = x[2]  # Absolute velocity
    delta       = x[3]  # Steering angle
    t           = x[4]  # Time
    a           = u[0]  # acceleration
    delta_dot   = u[1]  # Steering rate

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
    # variables z = [a, delta_dot, e_y, e_psi, v, delta, t]
    # parameter p = [rho_sigma, dist_i, dist_o, v_ref]

    e_y     = z[2]
    e_psi   = z[3]
    v       = z[4]
    v_ref   = p[3]
    delta_dot = z[1]

    return 1*(e_y)**2 + 0.1*(e_psi)**2 + 0.001*(v - v_ref)**2 + 0.1*(delta_dot)**2 # TODO for testing, trajectory following with constant speed
    # return 0.01*(e_y)**2

    # return 0.01*(delta_dot)**2
    # return 0

def objN(z, p):
    """Objective function for time optimal driving (T-Tref)^2 with T: time the car 
    needs to complete the prediction horizon -> T = t(s + pH)"""

    # parameters:
    # state x = [e_y, e_psi, v, delta, t]
    # input u = [a, delta_dot]

    # Tref = config.Tref
    # Tref = 0
    Tref = (config.N * config.integrator_ts)/config.v_ub # calculate Tref with prediction distance and max velocity
    T = z[6] # z[6] = t

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
    model.nvar  = 7  # number of variables (inputs variables + system variables)
    model.neq   = 5  # number of equality constraints (system variables)
    model.nh    = 2  # number of inequality constraint functions
    
    model.npar  = 4 # number of runtime parameters
    # p[0] list of rho_sigma
    # dist_i
    # dist_o

    # Objective function
    model.objective = obj # cost 0 for non last stage
    # model.objectiveN = objN # costs for the last stage
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
    model.E = np.concatenate([np.zeros((5,2)), np.eye(5)], axis=1)

    # Inequality constraints
    #  upper/lower variable bounds lb <= z <= ub
    #                     inputs               |  states
    #                     a             delta_dot            e_y             e_psi y           v           delta             t
    model.lb = np.array([config.a_lb,   config.ds_lb,   config.e_y_lb,  config.e_psi_lb,  config.v_lb, config.delta_lb, config.t_lb ])
    model.ub = np.array([config.a_ub,   config.ds_ub,   config.e_y_ub,  config.e_psi_ub,  config.v_ub, config.delta_ub, config.t_ub ])

    # General (differentiable) nonlinear inequalities hl <= h(z,p) <= hu

    # Stay between track limits
    # |e_y| < (track_width - car_width)/2 - safety_distance =>  0 < (track_width - car_width)/2 - safety_distance - |e_y| < inf

    # Constraint for cornering speed, with mu from sim
    # v < sqrt(mu * g * l/|delta|) => -inf < v^2 * |delta/l| - mu * g < 0

    # dont do it like this, you will divide by zero for straight driving :)
    # v < sqrt(mu * g * l/|delta|) => 0 < mu * g * l/|delta| - v^2 < inf
    
    # TODO fix obstacle avoidance (see do-mpc) 
    # Avoid obstacles, distance of e_y to e_ob grater than half of widths + safety_distance
    # |e_y - e_ob| > (ob_width + car_width)/2 + safety_distance => -|e_y - e_ob| + (ob_width + car_width)/2 + safety_distance < 0

    # Avoid obstacles
    # mpc.set_nl_cons('ob_lb', -fabs(_x['e_y'] - _tvp['e_ob']) + (_tvp['ob_width'] + car_width)/2 + safety_distance  , ub=0, soft_constraint=False)

    # Parameter for Constraints
    car_width =         config.car_width
    safety_distance =   config.safety_distance
    mu =                config.mu
    l =                 config.l_r + config.l_f
    g =                 config.g

    # variables z = [a, delta, e_y, e_psi, v, delta, t]
    # parameter p = [rho_sigma, track_width, e_ob, ob_width]
    model.ineq = lambda z, p: casadi.vertcat(
                                     p[1] - car_width/2 - safety_distance + z[2],            # track limits inner
                                     p[2] - car_width/2 - safety_distance - z[2],            # track limits outer
                                    #  z[4]**2 * casadi.fabs(z[5]/l) - mu * g,                                # cornering speed
                                    )

    # Upper/lower bounds for inequalities
    #                    track_width
    model.hl = np.array([0, 0])
    model.hu = np.array([np.Infinity, np.Infinity])

    # Initial condition on vehicle states x
    model.xinitidx = range(2,7) # use this to specify on which variables initial conditions
                                # are imposed

    # Solver generation
    # -----------------
    # Set solver options
    if not hardware:
        codeoptions = forcespro.CodeOptions('FORCESNLPsolver')
    else :
        codeoptions = forcespro.CodeOptions('FORCESNLPsolver_car_tx2')

        codeoptions.platform = 'NVIDIA-Cortex-A57'

    codeoptions.cleanup = 0
    codeoptions.forcenonconvex = 1

    # codeoptions.solvemethod = "SQP_NLP"
    # codeoptions.sqp_nlp.reg_hessian = 5.

    # Integrator options
    codeoptions.nlp.integrator.type =   config.integrator_type  # TODO check
    codeoptions.nlp.integrator.Ts =     config.integrator_ts    # TODO check
    codeoptions.nlp.integrator.nodes =  config.integrator_nodes # TODO check 

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
    
    # codeoptions.nlp.bfgs_init = 3.0*np.identity(6) # initialization of the hessian approximation
    codeoptions.showinfo = 1
    codeoptions.noVariableElimination = 1
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
