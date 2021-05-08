
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>



using namespace std;


class BubblesGenerator{

  public:

    // occupied positions to generate from pointcloud
    vector<double> occupied_positions_x;
    vector<double> occupied_positions_y;

    // global path
    vector<double> path_X;
    vector<double> path_Y;

    // for the bubbles
    vector <vector<double>>  feasiblebubbles_x ;
    vector <vector<double>>  feasiblebubbles_y ;
    vector <double>  midpoints_x ;
    vector <double>  midpoints_y ;
    vector <double>  radii;

    double max_radius  = 2;

    // ----------------------------- Function Protytpes --------------------------------------------

    void clear();

    void save_data();

    void generate();

    vector<double> linspaced(const double start, const double end, const int num);

    void defineDiscreteBubbles();

    double get_distance(const double p1x, const double p1y, const double p2x, const double p2y);

    void multiply_scalars_vector(vector <double> &V, const double a, const double b);
        
    void find_closest_point(double &smallest_distance, double &closest_x, double &closest_y, const double point_x, const double point_y, const vector<double>  set_x, const vector<double> set_y);

    bool check_inside_line(const double point_x, const double point_y, const double new_point_x, const double new_point_y);

}; 


