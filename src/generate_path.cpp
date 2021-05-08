
#include "generate_path.h"

using namespace std;






void PathGenerator::save_data(){

    ofstream myfile;
    myfile.open("/home/mohamad/Desktop/code_generation_2/plots/occupancy_grid.txt");
    for (unsigned int i=0; i<occupied_positions_x.size();i++){
            myfile<< occupied_positions_x[i]<<"   "<<occupied_positions_y[i]<<std::endl;
    }
    myfile.close();

    ofstream myfile2;
    myfile.open("/home/mohamad/Desktop/code_generation_2/plots/local_path.txt");
    for (unsigned int i=0; i<local_path_X.size();i++){
            myfile<< local_path_X[i]<<"   "<<local_path_Y[i]<<std::endl;
    }
    myfile2.close();


}

void PathGenerator::clear(){

    local_path_X.clear();
    local_path_Y.clear();

}
    

void PathGenerator::generate_occupancy_grid(){
 
// Generate  predefined Occupancy grid   
// ALso generate obstacles positions in the grid

  double occupancy_grid [10][10] = {
 {0,0,0,0,0,0,0,0,0,0},
 {0,0,0,0,0,0,0,0,0,0},
 {0,0,0,0,0,0,0,0,0,0},
 {0,0,0,0,0,0,0,0,0,0},
 {0,0,0,0,0,0,0,0,0,0},
 {0,0,0,0,0,0,0,0,0,0},
 {0,0,0,0,0,0,0,0,0,0},
 {1,1,1,0,0,0,0,0,0,0},
 {1,1,1,0,0,0,0,0,0,0},
 {1,1,1,0,0,0,0,0,0,0}};


int k = 0;
 for(int i=0;i<10;i++){
     for(int j = 0; j<10;j++){
         if (occupancy_grid[i][j] ==1){
            occupied_positions_x.push_back(j); 
            occupied_positions_y.push_back(i);
            k ++;
        }
     }
 }
 

}



void PathGenerator::read_path_points(){ //subsitiute with reading a file for example


    waypoints_x = {0,1,2,3};
    waypoints_y = {0,1,2,3};

}



void PathGenerator::generate_local_path(){
        

    float x = 0;
    float y = 0;
    float a = 0;
    float b = 0;

    // modify X and Y to get only the points starting from the closeest point to robot
    PathGenerator::modify_waypoints(pos_x, pos_y, waypoints_x, waypoints_y); 

    vector <double> X = waypoints_x;
    vector <double> Y = waypoints_y;   

    for (int i=0; i< (X.size()-1);i++)
    {  

        a = (Y[i+1]- Y[i])/(X[i+1]- X[i]) ;
        b =  Y[i] - a*X[i];
        x =  X[i];
        y =  Y[i];

        if(x > X[i+1]){
            while( x > X[i+1] )
            {
                x = x - 0.01;
                local_path_X.push_back(x);
                local_path_Y.push_back(a*x + b);
            }
        }
        else if(x < X[i+1]){
            
            while( x < X[i+1] )
            {
                x = x + 0.01;
                local_path_X.push_back(x);
                local_path_Y.push_back(a*x + b);
            }

        }
        else if(x == X[i+1]){

            if(y < Y[i+1]){

                while( y < Y[i+1] )
                {
                    y = y + 0.01;
                    local_path_X.push_back(x);
                    local_path_Y.push_back(y);
                }
            }
            else if(y > Y[i+1]){

                while( y > Y[i+1] )
                {
                    y = y - 0.01;
                    local_path_X.push_back(x);
                    local_path_Y.push_back(y);
                }
            }
        }

    }
        
}

void PathGenerator::modify_waypoints(const double point_x, const double point_y, const  vector <double>  set_x, const vector<double> set_y){

    double len = set_x.size();
    double dist1 = 0;
    double dist2 = 0;
    int k = 0;

    for(int j = 0; j < len ; j++) {
            
            dist1 = PathGenerator::get_distance(point_x,point_y,set_x[j],set_y[j]);
            dist2 = PathGenerator::get_distance(point_x,point_y,set_x[j+1],set_y[j+1]);
            

            // ------------------ for chanigng points that create the local path = for bubbles
            
            if (dist1 < tol_waypoint && j != 0 && j != (len-1) ){
                k = j;
            }

            // --------------------- for changing goal waypoint 

            if (dist1 < tol_waypoint && j != 0){
                goal_waypoint_x = waypoints_x[j];
                goal_waypoint_y = waypoints_y[j];
                break;
            }

            if (dist2 > dist1 && j != 0){ // if something wrong happens and robot skips a waypoint (dist1)
                goal_waypoint_x = waypoints_x[j];
                goal_waypoint_y = waypoints_y[j];
                break;
            }

    }

            waypoints_x = vector<double>(set_x.begin() + k, set_x.end());
            waypoints_y = vector<double>(set_y.begin() + k, set_y.end());
     
} 

double PathGenerator::get_distance(const double p1x, const double p1y,const double p2x,const double p2y){
        
    double dist;
    dist =  pow((p1x- p2x),2) + pow((p1y - p2y),2);
    dist  = pow(dist,0.5);
    return dist; 

}




