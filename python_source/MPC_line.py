

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


global_end_goal_x       =    3
global_end_goal_y       =    3
initial_pos_x           =    0
initial_pos_y           =    0
xlim_min                =   -1     
xlim_max                =    5
ylim_min                =   -2
ylim_max                =    6

    

Nsim    = 3

N       = 20
dt      = 0.5    


NB = N
NP = N


#------------- Initialize OCP

ocp = Ocp(T = N*dt)   


#----------------------- waypoints ------

waypoints_x = [1,2,3]
waypoints_y = [1,2,3]

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



ocp.set_initial(x, np.zeros(N+1)) 
ocp.set_initial(y, np.zeros(N+1)) 
ocp.set_initial(theta, np.zeros(N+1))
ocp.set_initial(sdot_obs, np.zeros(N+1)) 
ocp.set_initial(sdot_obs, np.zeros(N+1))
ocp.set_initial(v , np.zeros(N+1))
ocp.set_initial(w , np.zeros(N+1))


#--------------- Bubbles with s 


bubbles_x       =  ocp.parameter(N+1)
bubbles_y       =  ocp.parameter(N+1)
bubbles_radii   =  ocp.parameter(N+1)

ocp.set_value(bubbles_x, shifted_midpoints_x)
ocp.set_value(bubbles_y, shifted_midpoints_y)
ocp.set_value(bubbles_radii,   shifted_radii)

tlength1        =  len(shifted_midpoints_x)
tunnel_s1       =  np.linspace(0,1,tlength1) 

ocp.subject_to(ocp.at_tf(s_obs) <= 1)   

obs_spline_x = interpolant('x','bspline',[tunnel_s1], 1  , {"algorithm": "smooth_linear","smooth_linear_frac":0.49})
obs_spline_y = interpolant('y','bspline',[tunnel_s1], 1  , {"algorithm": "smooth_linear","smooth_linear_frac":0.49})
obs_spline_r = interpolant('r','bspline',[tunnel_s1], 1  , {"algorithm": "smooth_linear","smooth_linear_frac":0.49})


ocp.subject_to( ( ( ( x - obs_spline_x(s_obs,bubbles_x) )**2 + ( y-obs_spline_y(s_obs,bubbles_y) )**2 ) - (obs_spline_r(s_obs,bubbles_radii)**2 ) ) < 0 )


# -------------------------------------- Objective function 

ocp.add_objective( 1*ocp.integral((x - end_goal_x)**2 + (y-end_goal_y)**2))  # integral = cause of extra state in acados


# ----------------- Solver

options = {"ipopt": {"print_level": 5}}
options["expand"] = False
options["print_time"] = True
ocp.solver('ipopt', options)


method = external_method('acados',
                        N=N,
                        intg='rk',
                        qp_solver= 'FULL_CONDENSING_HPIPM',
                        expand=False,
                        nlp_solver_max_iter= 1000,
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



# Log data for post-processing  
t_sol, x_sol            = sol.sample(x,           grid='control')
t_sol, y_sol            = sol.sample(y,           grid='control')
t_sol, theta_sol        = sol.sample(theta,       grid='control')
t_sol, s_obs_sol        = sol.sample(s_obs,       grid='control')
t_sol, v_sol            = sol.sample(v,           grid='control')
t_sol, w_sol            = sol.sample(w,           grid='control')
t_sol, sdot_obs_sol     = sol.sample(sdot_obs,    grid='control')


#----------- MPC

# Get discretised dynamics as CasADi function to simulate the system
Sim_system_dyn = ocp._method.discrete_system(ocp)


#---------------- Initialize Logging variables

time_hist           = np.zeros((Nsim+1, N+1))
x_hist              = np.zeros((Nsim+1, N+1))
y_hist              = np.zeros((Nsim+1, N+1))
theta_hist          = np.zeros((Nsim+1, N+1))
s_path_hist         = np.zeros((Nsim+1, N+1))
s_obs_hist          = np.zeros((Nsim+1, N+1))
v_hist              = np.zeros((Nsim+1, N+1))
w_hist              = np.zeros((Nsim+1, N+1))
sdot_path_hist      = np.zeros((Nsim+1, N+1))
sdot_obs_hist       = np.zeros((Nsim+1, N+1))


waypoints_hist_x         = np.zeros((Nsim+1, 1))
waypoints_hist_y         = np.zeros((Nsim+1, 1))


# for post processing
time_hist[0,:]          = t_sol
x_hist[0,:]             = x_sol
y_hist[0,:]             = y_sol
theta_hist[0,:]         = theta_sol
s_obs_hist[0,:]         = s_obs_sol
v_hist[0,:]             = v_sol
w_hist[0,:]             = w_sol
sdot_obs_hist[0,:]      = sdot_obs_sol



waypoints_hist_x[0] = waypoints_x[0]
waypoints_hist_y[0] = waypoints_y[0]        



clearance = 0.2

clearance_wp = 0.5

waypoint_indx = 0
    
i = 0
    
time = 0 
    
for i in range(Nsim):
    
    
    print("timestep", i+1, "of", Nsim)
    
        
    error = sumsqr(current_X[0:2] - global_goal)
    if error < clearance: 
        break   #solution reached the global end goal 
        
        
    print( f' x: {current_X[0]}' )
    print( f' y: {current_X[1]}' )
      
    
    #------------------- Update initial position ------------------------------
    
    # Combine control inputs
    current_U = vertcat(v_sol[0], w_sol[0] , sdot_obs_sol[0])

    # Simulate dynamics (applying the first control input) and update the current state
    current_X = Sim_system_dyn(x0=current_X, u=current_U, T=t_sol[1]-t_sol[0])["xf"]
    

    initial_pos_x = double(current_X[0])
    initial_pos_y = double(current_X[1])
    
    #------------ Update time spent to reach goal 
    
    time = time + (t_sol[1]-t_sol[0])
   
    #------------------------- Generate grid and path -------------------------

    if waypoint_indx + 1 < len(waypoints_x): #if there are waypoints left
    
        new_waypoint = vertcat(waypoints_x[waypoint_indx+1],waypoints_y[waypoint_indx+1]) 
        waypoint = vertcat(waypoints_x[waypoint_indx],waypoints_y[waypoint_indx]) 
        
        dist_waypoint     = sumsqr(current_X[0:2] - waypoint)
        dist_new_waypoint = sumsqr(current_X[0:2] - new_waypoint)
        
        
        if dist_waypoint < clearance_wp or dist_new_waypoint < clearance_wp :
            
            waypoint_indx = waypoint_indx + 1
            waypoint      = new_waypoint     
     
    waypoints_hist_x[i+1] = waypoints_x[waypoint_indx]
    waypoints_hist_y[i+1] = waypoints_y[waypoint_indx]        

    

    global_path_x, global_path_y, Bspline_obj   = create_global_path_mpc(6,initial_pos_x,initial_pos_y,10)

        
    #---------------- Creating the Bubbles-------------------------------------


    shifted_midpoints_x = global_path_x
    shifted_midpoints_y = global_path_y
    shifted_radii       = np.ones(N+1)

        
    # # --------------- select N points for path spline-------------------------

    # if error > 0.5:
    #     Bspline_obj, u = interpolate.splprep([global_path_x,global_path_y], u = None, s = 0)
    #     u = np.linspace(0,1,NP)
    #     global_path = interpolate.splev(u, Bspline_obj)
    #     global_path_x = np.array(global_path[0])
    #     global_path_y = np.array(global_path[1])

    #------------------- Updating Tunnels ------------------------------------

    # global_path_x           = global_path_x[0:NP]
    # global_path_y           = global_path_y[0:NP]
    
    ocp.set_value(bubbles_x, shifted_midpoints_x)
    ocp.set_value(bubbles_y, shifted_midpoints_y)
    ocp.set_value(bubbles_radii,   shifted_radii)
    
    ocp.set_value(end_goal_x, waypoints_x[waypoint_indx])
    ocp.set_value(end_goal_y, waypoints_y[waypoint_indx])
    
    # Set the parameter X0 to the new current_X
    ocp.set_value(X_0, current_X)

    print("--------------------------------",current_X)
    
    #------------------ set initial 
    
    ocp.set_initial(s_obs,     s_obs_sol) 
    ocp.set_initial(sdot_obs,  sdot_obs_sol) 

    ocp.set_initial(x,         x_sol) 
    ocp.set_initial(y,         y_sol) 
    
    ocp.set_initial(v,         v_sol) 
    ocp.set_initial(w,         w_sol) 
    ocp.set_initial(theta,     theta_sol) 
    


    #------------------------ Plot results every iteration


    plt.figure(dpi=300)
    plt.title('MPC')    
    plt.plot(x_sol, y_sol, 'b-')
    plt.plot(global_path_x, global_path_y, 'g--')
    plt.plot(x_hist[0:i+1,0],y_hist[0:i+1,0], 'bo', markersize = 5)
    plt.plot(x_sol[0], y_sol[0], 'ro', markersize = 5)
    plt.plot(waypoints_x[waypoint_indx], waypoints_y[waypoint_indx], 'go')
    # plt.legend(['solution of current OCP','obstacles','global path', 'accumulated MPC solution', 'current OCP first shooting point','feasible bubbles'],loc = (0.8,0.3))
    plt.xlim([xlim_min,xlim_max])
    plt.ylim([ylim_min,ylim_max])
    plt.pause(1)

    
    #------------------------- Solve the optimization problem

    start_time = tmp.time()
    
    sol = ocp.solve()

    end_time = tmp.time()
    
    delta_time = delta_time + ( end_time - start_time ) 

    #-------------------------- Log data for next iteration  
    
    t_sol, x_sol            = sol.sample(x,           grid='control')
    t_sol, y_sol            = sol.sample(y,           grid='control')
    t_sol, theta_sol        = sol.sample(theta,       grid='control')
    t_sol, s_obs_sol        = sol.sample(s_obs,       grid='control')
    t_sol, v_sol            = sol.sample(v,           grid='control')
    t_sol, w_sol            = sol.sample(w,           grid='control')
    t_sol, sdot_obs_sol     = sol.sample(sdot_obs,    grid='control')
 
    # for post processing
    time_hist[i+1,:]          = t_sol
    x_hist[i+1,:]             = x_sol
    y_hist[i+1,:]             = y_sol
    theta_hist[i+1,:]         = theta_sol
    s_obs_hist[i+1,:]         = s_obs_sol
    v_hist[i+1,:]             = v_sol
    w_hist[i+1,:]             = w_sol
    sdot_obs_hist[i+1,:]      = sdot_obs_sol
    
    
    


# -------------------------------------------
#          Plot the results
# -------------------------------------------

