

import time as tmp
import numpy as np
from numpy import pi, cos, sin
import matplotlib.pyplot as plt
from scipy import interpolate
from pylab import *
from casadi import Function, linspace, vertcat, horzcat, DM, interpolant, sum1, MX, hcat, sumsqr
from rockit import *
from rockit import Ocp , FreeTime, MultipleShooting


from MPC_Bubble_tunnel_generation_v2 import generate_bubbles_mpc_v2, get_bubbles_mpc_loop
from MPC_Grid_generation import create_obstacles_mpc, create_global_path_mpc


global_end_goal_x       =    1
global_end_goal_y       =    1
initial_pos_x           =    0
initial_pos_y           =    0
xlim_min                =   -1     
xlim_max                =    5
ylim_min                =   -2
ylim_max                =    6

    
  
N       = 20
dt      = 0.5         #effect unclear = how to connect this to rosrate ?


#------------- Initialize OCP

ocp = Ocp(T = N*dt)   


#----------------------------------- waypoints ------------------------


#---------------- Initialize path

#N+1 goal points
global_path_x = np.linspace(0,3,N+1)
global_path_y = np.linspace(0,3,N+1)

#---------------------- bubbles 

shifted_midpoints_x = global_path_x
shifted_midpoints_y = global_path_y
shifted_radii       = np.ones(N+1)


#------------------------- System model

x       =  ocp.state()
y       =  ocp.state()
theta   =  ocp.state()

v       =  ocp.control()
w       =  ocp.control()

#--------------------------path parameters 

s_obs        =  ocp.state()
sdot_obs     =  ocp.control()

#-----------------------------ODEs

ocp.set_der(x            ,        v*cos(theta))
ocp.set_der(y            ,        v*sin(theta))
ocp.set_der(theta        ,        w)
ocp.set_der(s_obs        ,        sdot_obs)


#-------------------------------------------------------------------------------#
#                            Solve the first iteration                          #
#-------------------------------------------------------------------------------#


#------------------------- Constraints on initial point


X_0 = ocp.parameter(4)
X   = vertcat(x, y, theta, s_obs)

ocp.subject_to(ocp.at_t0(X) == X_0)

current_X = vertcat(initial_pos_x,initial_pos_y,0.0,0.0) 
ocp.set_value(X_0, current_X)


#------------------------- Constraints on Final point

global_goal = vertcat(global_end_goal_x,global_end_goal_y) 


end_goal_x = ocp.parameter(1)
end_goal_y = ocp.parameter(1)

ocp.set_value( end_goal_x, global_end_goal_x)
ocp.set_value( end_goal_y, global_end_goal_y)


#----------------------------- constraints on controls 

ocp.subject_to(  0          <= ( v  <= 1   ))
ocp.subject_to( -pi         <= ( w  <= pi  ))       
ocp.subject_to( sdot_obs    >=   0)


# ------------------------ Initial guess ------------------------

# Initial guess of  ACADOS and  IPOPT are made the same 

# path guess = global path
path_guess_x         = np.linspace(0,global_end_goal_x,N+1)
path_guess_y         = np.linspace(0,global_end_goal_y,N+1)

ocp.set_initial(x, path_guess_x) 
ocp.set_initial(y, path_guess_y) 

# path parameters
s_guess = np.linspace(0,1,N+1)
sdot_guess = (s_guess[-1]-s_guess[0])/(N+1)
sdot_guess = sdot_guess*np.ones(N+1)

ocp.set_initial(sdot_obs, s_guess) 
ocp.set_initial(sdot_obs, sdot_guess)

#controls v,w guess + theta
v_guess     =  np.zeros(N+1)
w_guess     =  np.zeros(N+1)
theta_guess =  np.zeros(N+1)  

for i in range(1,N+1):
    #----------v_guess
    xdot        = (path_guess_x[i] - path_guess_x[i-1])/dt
    ydot        = (path_guess_y[i] - path_guess_y[i-1])/dt
    v_guess[i]  = np.sqrt(xdot**2 + ydot**2)
    #----- theta guess
    theta_guess[i] = np.arcsin(ydot/v_guess[i])

#w_guess
for i in range(1,N+1):
    w_guess[i] = (theta_guess[i]-theta_guess[i-1])/dt


ocp.set_initial(theta, theta_guess)

ocp.set_initial(v , v_guess)
ocp.set_initial(w , w_guess)



# ocp.set_initial(x, np.zeros(N+1)) 
# ocp.set_initial(y, np.zeros(N+1)) 
# ocp.set_initial(theta, np.zeros(N+1))
# ocp.set_initial(sdot_obs, np.zeros(N+1)) 
# ocp.set_initial(sdot_obs, np.zeros(N+1))
# ocp.set_initial(v , np.zeros(N+1))
# ocp.set_initial(w , np.zeros(N+1))


#--------------- Bubbles with s 

tlength1        =  len(shifted_midpoints_x[0:N])
tunnel_s1       =  np.linspace(0,1,tlength1) 

bx       =  ocp.parameter(N)
by       =  ocp.parameter(N)
br       =  ocp.parameter(N)


ocp.set_value(bx, shifted_midpoints_x[0:N])
ocp.set_value(by, shifted_midpoints_y[0:N])
ocp.set_value(br, shifted_radii[0:N])


ocp.subject_to(ocp.at_tf(s_obs) <= 1)    # effect unclear === the path given is long = first ocp iteration will not get there if dt is small

obs_spline_x = interpolant('x','bspline',[tunnel_s1], 1  , {"algorithm": "smooth_linear","smooth_linear_frac":0.49})
obs_spline_y = interpolant('y','bspline',[tunnel_s1], 1  , {"algorithm": "smooth_linear","smooth_linear_frac":0.49})
obs_spline_r = interpolant('r','bspline',[tunnel_s1], 1  , {"algorithm": "smooth_linear","smooth_linear_frac":0.49})


ocp.subject_to(   ( x - obs_spline_x(s_obs,bx) )**2 + ( y - obs_spline_y(s_obs,by) )**2  - (obs_spline_r(s_obs,br))**2 < 0)

# -------------------------------------- Objective function 

ocp.add_objective( 1*ocp.integral((x - end_goal_x)**2 + (y-end_goal_y)**2))  # integral = cause of extra state in acados


# ----------------- Solver

options = {"ipopt": {"print_level": 5}}
options["expand"] = False
options["print_time"] = True
ocp.solver('ipopt', options)


# qp_solvers = ('PARTIAL_CONDENSING_HPIPM', 'FULL_CONDENSING_QPOASES', 'FULL_CONDENSING_HPIPM', 'PARTIAL_CONDENSING_QPDUNES', 'PARTIAL_CONDENSING_OSQP')
# integrator_types = ('ERK', 'IRK', 'GNSF', 'DISCRETE')
# SOLVER_TYPE_values = ['SQP', 'SQP_RTI']
# HESS_APPROX_values = ['GAUSS_NEWTON', 'EXACT']
# REGULARIZATION_values = ['NO_REGULARIZE', 'MIRROR', 'PROJECT', 'PROJECT_REDUC_HESS', 'CONVEXIFY']


# Pick a solution method
# method = external_method('acados', N=N,qp_solver= 'FULL_CONDENSING_HPIPM', nlp_solver_max_iter= 100, hessian_approx='EXACT', regularize_method = 'MIRROR' ,integrator_type='ERK',nlp_solver_type='SQP',qp_solver_cond_N=N)

method = external_method('acados',
                        N=N,
                        intg='rk',
                        qp_solver= 'FULL_CONDENSING_HPIPM',
                        expand=False,
                        nlp_solver_max_iter= 500,
                        hessian_approx='EXACT',
                        regularize_method = 'MIRROR' ,
                        integrator_type='DISCRETE',
                        nlp_solver_type='SQP',
                        qp_solver_cond_N=N
                        )

ocp.method(method)



#-------------------------------- OCP Solution and Results                             


start_time = tmp.time()

sol = ocp.solve()

end_time = tmp.time()

delta_time =  end_time - start_time 


# code generation

# import rockit
# print(rockit)

acados_interface = ocp._method
acados_interface.mmap.generate('rockitmmap.c',{"with_header":True})
import subprocess
subprocess.Popen(["gcc","-g","-shared","-fPIC","rockitmmap.c","-lm","-o","librockitmmap.so"]).wait()



# Log data for post-processing  
t_sol, x_sol            = sol.sample(x,           grid='control')
t_sol, y_sol            = sol.sample(y,           grid='control')
t_sol, theta_sol        = sol.sample(theta,       grid='control')
t_sol, s_obs_sol        = sol.sample(s_obs,       grid='control')
t_sol, v_sol            = sol.sample(v,           grid='control')
t_sol, w_sol            = sol.sample(w,           grid='control')
t_sol, sdot_obs_sol     = sol.sample(sdot_obs,    grid='control')


# print("x_sol", x_sol)
# print("y_sol", y_sol)
# print("theta_sol", theta_sol)
# print("s_sol", s_obs_sol)
# print("v_sol", v_sol)
# print("w_sol", w_sol)
# print("sdot_sol", sdot_obs_sol)   

# -------------------------------------------
#          Plot the results
# -------------------------------------------

occupied_positions_x = []
occupied_positions_y = []
    
npoints =  500  #numbr of points of every circle
ts      =  np.linspace(0, 2*np.pi, npoints) #for creating circles points
    
    
shifted_feasiblebubbles_x = []
shifted_feasiblebubbles_y = []
for it in range (0, len(shifted_midpoints_x)):
        shifted_feasiblebubbles_x.append(shifted_midpoints_x[it] + shifted_radii[it]*np.cos(ts))
        shifted_feasiblebubbles_y.append(shifted_midpoints_y[it] + shifted_radii[it]*np.sin(ts))


plt.figure(dpi=300)
plt.plot(x_sol,y_sol, 'bo', markersize = 5)
plt.plot(global_end_goal_x , global_end_goal_y ,'k^', markersize = 10)
plt.plot(global_path_x, global_path_y, 'g--')
plt.plot(shifted_feasiblebubbles_x, shifted_feasiblebubbles_y, 'ro', markersize = 1)
plt.legend(['solution','end goal',' global path ','Feasible Bubbles'], loc = "best")
plt.title('Solution')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.xlim([xlim_min,xlim_max])
plt.ylim([ylim_min,ylim_max])

plt.show(block = True)


# plt.figure(dpi=300)
# plt.plot(x_sol,y_sol, 'bo', markersize = 5)
# plt.plot(path_guess_x,path_guess_y,'b')
# plt.legend(['solution','guess'], loc = "best")
# plt.title('Solution vs guess')
# plt.xlabel('x [m]')
# plt.ylabel('y [m]')

# plt.figure(dpi=300)
# plt.plot(theta_sol, 'bo', markersize = 5)
# plt.plot(theta_guess,'b')
# plt.legend(['solution','guess'], loc = "best")
# plt.title('Solution vs guess')
# plt.xlabel('indx')
# plt.ylabel('theta')

# plt.show(block = True)



