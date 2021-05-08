
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>




using namespace std;



class PathGenerator{

  public:

    // occupied positions
    vector<double> occupied_positions_x;
    vector<double> occupied_positions_y;

    // local path used for bubbles generation
    vector<double> local_path_X;
    vector<double> local_path_Y;

    // designated waypoints used for creating local path, generated from file input
    vector<double> waypoints_x;
    vector<double> waypoints_y;

    // designated goal for controller
    double goal_waypoint_x;
    double goal_waypoint_y;

    // current position of robot in (x,y)
    double pos_x;
    double pos_y;

    // tolerances for changing waypoints
    double tol_waypoint = 0.2 ;
    

    
    // ----------------------------- Function Protytpes --------------------------------------------

    void save_data();

    void generate_occupancy_grid();
 
    void clear();

    void generate_local_path();
    
    void read_path_points();

    void modify_waypoints(const double point_x, const double point_y, const  vector <double>  set_x, const vector<double> set_y);

    double get_distance(const double p1x, const double p1y,const double p2x,const double p2y);
        


};










