

#include "call_acados_model.h"
#include "generate_path.h"
#include "generate_bubbles.h"




int main(int argc, char **argv){


  //----- Generate Path and waypoints ---------------

  PathGenerator path;

  path.pos_x = 0;
  path.pos_y = 0;

  path.generate_occupancy_grid();
  path.read_path_points();
  path.generate_local_path();
  path.save_data();

  //------ Generate Bubbles ----------------------

  BubblesGenerator bubbles;

  bubbles.path_X  = path.local_path_X;
  bubbles.path_Y  = path.local_path_Y;

  bubbles.occupied_positions_x = path.occupied_positions_x;
  bubbles.occupied_positions_y = path.occupied_positions_y;

  bubbles.generate(); 
  bubbles.save_data();


  // ----------- Generate acados Model -----------------

  Controller model;

  model.goal_waypoint_x = path.goal_waypoint_x;
  model.goal_waypoint_y = path.goal_waypoint_y;

  model.state_x         = path.pos_x;
  model.state_y         = path.pos_y;
  model.state_theta     = 0;
  model.state_s         = 0; 

  // extend number of bubbles to be = N 

  double bub_size = bubbles.midpoints_x.size();

  for(int i = 0 ; i < model.N; ++i){

      if(i < bub_size){
          model.midpoints_x[i] = bubbles.midpoints_x[i];
          model.midpoints_y[i] = bubbles.midpoints_y[i];
          model.radii[i]       = bubbles.radii[i];
      }
      else{
          model.midpoints_x[i] = bubbles.midpoints_x[bub_size-1];
          model.midpoints_y[i] = bubbles.midpoints_y[bub_size-1];
          model.radii[i]       = bubbles.radii[bub_size-1];
      }
  }
  
  model.init();
  model.monitor_mmap_outputs = true;
  model.monitor_parameters   = false;
  model.print_solver_stats   = true;


  //  Solve 1st time 

  model.run_acados_solver();


  return 0;


}



