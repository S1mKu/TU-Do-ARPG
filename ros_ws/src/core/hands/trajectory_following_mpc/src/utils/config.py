import numpy as np

# Car parameters
l_r     = 0.17145               # distance rear wheels to center of gravitiy of the car
l_f     = 0.15875               # distance front wheels to center of gravitiy of the car
car_width = 0.31
safety_distance = 0.25
mu = 0.75
g = 9.81

# Bounds for equality constraints
a_lb = -9.51
a_ub = 9.51

ds_lb = -1.2
ds_ub = +1.2

delta_lb = -0.4189
delta_ub = +0.4189

e_y_lb = -np.inf
e_y_ub = +np.inf

e_psi_lb = np.deg2rad(-90.)
e_psi_ub = np.deg2rad(+90.)

v_lb = 0.5
v_ub = 4.

t_lb = 0.
t_ub = np.inf

# Bounds for inequality constraints
track_width_lb = 0
track_width_ub = np.inf

corner_speed_lb = -np.inf
corner_speed_ub = 0

obstacles_lb = -np.inf
obstacles_ub = 0

# Bounds of sim for acceleration calculation
a_ub_sim = 9.51
v_ub_sim = 20.
v_lb_sim = -5.

# Parameters for mpc generation
N = 20                          # Horizon length
integrator_type = 'ERK4'
integrator_ts = 0.2
integrator_nodes = 5
maxit = 100
printlevel = 0
optlevel = 2

