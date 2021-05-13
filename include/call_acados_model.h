

// standard
#include <stdio.h>
#include <stdlib.h>
#include <dlfcn.h>
#include <exception>
#include <iostream>
#include <vector>
#include <map>
#include <math.h>
#include <fstream>
#include <cmath>
#include <ostream>
#include <type_traits>


// acados
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/sim_interface.h"
#include "acados_c/external_function_interface.h"
#include "c_generated_code/acados_solver_rockit_model.h"
#include "c_generated_code/acados_sim_solver_rockit_model.h"


//casadi function
#include "rockitmmap.h"




template <typename T, unsigned int N>
typename std::enable_if<!std::is_same<T, char>::value, std::ostream &>::type
operator<<(std::ostream & os, const T (&arr)[N])
{
  int i;
  os<<"[ ";
  for(i = 0; i < N; i++)
      os << arr[i] << " , ";
  os <<" ] "<<std::endl;
  return os;
}



class Controller{

  public:

    // same as in oython source code
    int N = 20;
    double dt = 0.5; 

    // booleans
    bool breakstatus            = false;
    bool monitor_initial_guess  = false;
    bool monitor_solution       = false;
    bool print_solver_stats     = false;
    bool monitor_parameters     = false;
    bool monitor_mmap_outputs   = false;
    bool first_run              = true;

    // acados solver
    int nb_solved = 0;
    int status;
    nlp_solver_capsule *acados_ocp_capsule ;
    ocp_nlp_config *nlp_config ;
    ocp_nlp_dims *nlp_dims;
    ocp_nlp_in *nlp_in ;
    ocp_nlp_out *nlp_out;
    ocp_nlp_solver *nlp_solver;
    void *nlp_opts;

    //number of states  
    const int nx = 5;
    //number of controls 
    const int nu = 3;

    // for using mmap
    long long int sz_arg, sz_res, sz_iw, sz_w;
    std::map<std::string, std::vector<casadi_real> > outputs;
    std::map<std::string, casadi_real> outputs_sizes;

    // constraints variables 
    double lbx0[5];
    double ubx0[5];
    double ubu[3];
    double lbu[3];
    double lh[1];
    double uh[1];
    double lh_e[1];
    double uh_e[1];
    double lbx_e[1];
    double ubx_e[1];


    // solution of every ocp
    double x_sol[20];
    double y_sol[20];
    double theta_sol[20];
    double s_sol[20];
    double v_sol[20];
    double w_sol[20];
    double sdot_sol[20];

    // parameters
    int p_size = 66;   //need to set manually for now 
    double p[66];

    // initial guess
    std::vector <double>  path_guess_x ;
    std::vector <double>  path_guess_y ;

    double x_init[21][5];   // N+1 guess for states
    double v_guess[21];
    double w_guess[21];

    double u_init[20][3];
    double theta_guess[20];
    double sdot_guess[20];
    double s_guess[20];


    //bubble data
    double midpoints_x[20] ;
    double midpoints_y[20] ;
    double radii[20];

    // designated goal for controller
    double goal_waypoint_x;
    double goal_waypoint_y;

    // current position of robot in (x,y) and states
    double state_x;
    double state_y;
    double state_theta;
    double state_s;



    // ----------------------------- Function Protytpes --------------------------------------------

    void clear();
    
    void save_data();

    void create_path_guess();

    void init();

    void set_parameters();

    void run_acados_solver();

    void set_constraints();

    void free_solver();

    void monitor();


};





