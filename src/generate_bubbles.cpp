


#include "generate_bubbles.h"

using namespace std;



// ----------------------------- Functions --------------------------------------------


void BubblesGenerator::save_data(){

  ofstream myfile3;
  myfile3.open ("/home/mohamad/Desktop/code_generation_2/plots/bubbles_midpoints.txt");
  for (unsigned int i=0; i<midpoints_x.size();i++){
      myfile3<< midpoints_x[i]<<"   "<<midpoints_y[i]<<endl;
  }
  myfile3.close();

  ofstream myfile4;
  myfile4.open ("/home/mohamad/Desktop/code_generation_2/plots/bubbles.txt");
  for (unsigned int i=0; i<feasiblebubbles_x.size();i++){
      for(unsigned int j = 0; j<feasiblebubbles_x[i].size();j++){
          myfile4<< feasiblebubbles_x[i][j]<<"   "<<feasiblebubbles_y[i][j]<<endl;
      }
  }
  myfile4.close(); 


}


void BubblesGenerator::clear(){

    occupied_positions_x.clear();
    occupied_positions_y.clear();
    
    feasiblebubbles_x.clear();
    feasiblebubbles_y.clear();
    
    midpoints_x.clear();
    midpoints_y.clear();
    radii.clear();

}



void BubblesGenerator::generate()  
{


    // ------------------- create Bubbles ----------------------------------

    BubblesGenerator::defineDiscreteBubbles();



}

vector<double> BubblesGenerator::linspaced(const double start, const double end, const int num){

    vector<double> linspaced;

    if (num== 0) { 
        return linspaced; 
        }
    if (num == 1) 
        {
        linspaced.push_back(start);
        return linspaced;
        }

    double delta = (end - start) / (num - 1);

    for(int i=0; i < num-1; ++i)
        {
        linspaced.push_back(start + delta * i);
        }
    linspaced.push_back(end); 

    return linspaced;
}

void BubblesGenerator::defineDiscreteBubbles(){
    
    
    int len_occupied_positions = occupied_positions_x.size(); 

    double point_x = path_X[0];
    double point_y = path_Y[1];

    int j = 0;
    int k = 0;

    double distance          = 0;
    double smallest_distance = 0;

    vector <double>  closest_points_x;
    vector <double>  closest_points_y;

    vector <double> Ax;
    vector <double> Ay;

    double radius      = 0;
    double delta_x     = 0.0;
    double delta_y     = 0.0;
    double midpoint_x  = 0.0;
    double midpoint_y  = 0.0;

    bool end_of_spline_reached = false;
    bool new_radius_feasible   = false;


    vector <double> ts;
    ts   = BubblesGenerator::linspaced(0, 2*M_PI, 100);
    
    vector <double> costs;
    for(int q = 0 ; q< ts.size() ;q++){
            costs.push_back(cos(ts[q]));
    }

    vector <double> sints;
    for(int q = 0; q< ts.size() ;q++){
            sints.push_back(sin(ts[q]));
    }



    double closest_point_x;
    double closest_point_y;

    int index  = 0;
    int indexp = 0;

    double acceptable_radius = 0.5;

    bool new_point_inside_bubble = true;
    bool edge_point = false;

    int max_index = path_X.size() - 1;

    double shifted_radius;
    double shifted_point_x;
    double shifted_point_y;

    vector <double> new_radius;

    double new_point_x;
    double new_point_y;
    double new_rad;

    bool inside_line = false;

    // ---------------------------------------------    While Loop ------------------------------------------------//
        
        
    while (! end_of_spline_reached){

        // ------------------------------------ update points ------------------------

        point_x = path_X[index];
        point_y = path_Y[index];

        // ------------------------------ check if end of path reached ------------------- 

        if(index == max_index){  
            end_of_spline_reached = true;
        }

        if(index == 0 && index == (max_index - 1) ){
            edge_point = true;
        }
        
        // ------------- get bubble radius ------------------------------------------------

        BubblesGenerator::find_closest_point(smallest_distance, closest_point_x, closest_point_y, point_x, point_y, occupied_positions_x, occupied_positions_y);
        
        radius = smallest_distance;

        if(radius > max_radius)
            radius = max_radius;

        // --------------- for choosing the next point on the path -------------------------

        indexp = index;
        new_point_inside_bubble = true;
        distance = 0;

        while (new_point_inside_bubble == true){

            indexp = indexp + 1;
            if (indexp >= (max_index)){
                index = max_index;
                break;
            }

            midpoint_x = path_X[indexp];
            midpoint_y = path_Y[indexp];

            distance = BubblesGenerator::get_distance(midpoint_x,midpoint_y,point_x,point_y);

            if(distance >= radius){
                new_point_inside_bubble = false;
                index = indexp;
            }
                
        }

        // ---------------------- for shifting the midpoints -------------------------------

        shifted_radius  = radius;
        shifted_point_x = point_x;
        shifted_point_y = point_y;   
        new_rad = 0;
        new_radius = {};
        new_radius.push_back(new_rad); 

        if (radius < acceptable_radius & !edge_point){


            delta_x      = 0.2*(point_x - closest_point_x);
            delta_y      = 0.2*(point_y - closest_point_y);

            new_point_x  = point_x;
            new_point_y  = point_y;

            for(int s = 0; s<10; s++){

                new_point_x = new_point_x + delta_x;
                new_point_y = new_point_y + delta_y;

                inside_line = false;
                inside_line = BubblesGenerator::check_inside_line(point_x, point_y, new_point_x, new_point_y);

                if(inside_line == false){

                    BubblesGenerator::find_closest_point(smallest_distance, closest_point_x, closest_point_y, new_point_x, new_point_y, occupied_positions_x, occupied_positions_y);
                    new_rad = smallest_distance;

                }

                if (new_rad >= new_radius.back()){

                    new_radius.push_back(new_rad);
                    if (new_rad > acceptable_radius){
                        shifted_radius  = new_rad;
                        shifted_point_x = new_point_x;
                        shifted_point_y = new_point_y;
                        break;
                    }
                }

            }

        }

        radius  = shifted_radius;
        point_x = shifted_point_x;
        point_y = shifted_point_y;
            

        //------------------ log data -----------------

        Ax = costs;
        Ay = sints;
        BubblesGenerator::multiply_scalars_vector(Ax, point_x, radius);
        BubblesGenerator::multiply_scalars_vector(Ay, point_y, radius);

        feasiblebubbles_x.push_back(Ax);          
        feasiblebubbles_y.push_back(Ay);

        midpoints_x.push_back(point_x);
        midpoints_y.push_back(point_y);
        radii.push_back(radius);



    } // end of while loop

    // End of  function

}

void BubblesGenerator::find_closest_point(double &smallest_distance, double &closest_x, double &closest_y, const double point_x, const double point_y, const  vector <double>  set_x, const vector<double> set_y){

    double len = set_x.size();
    double dist = 0;
    int k = 0;

    for(int j = 0; j < len; j++) {

            double ptx = set_x[j];
            double pty = set_y[j];
            
            dist = BubblesGenerator::get_distance(point_x,point_y,ptx,pty);
            

            if (j == 0){
                smallest_distance = dist;
                k = j;
            }
            if (dist < smallest_distance){
                smallest_distance = dist;
                k = j;
            }
    }
    
            closest_x = set_x[k]; 
            closest_y = set_y[k];        
} 

double BubblesGenerator::get_distance(const double p1x, const double p1y,const double p2x,const double p2y){
        
    double dist;
    dist =  pow((p1x- p2x),2) + pow((p1y - p2y),2);
    dist  = pow(dist,0.5);
    return dist; 

}

void BubblesGenerator::multiply_scalars_vector(vector <double> &V, const double a, const double b){
        
        for(unsigned int i = 0; i<V.size();i++){
            V[i] = a + b*V[i];
        }
}
    
bool BubblesGenerator::check_inside_line(const double point_x, const double point_y,  const double new_point_x, const double new_point_y){


    bool inside_line = false;
    double corner_new_point_x = new_point_x;
    double corner_new_point_y = new_point_y; 
    double corner_point_x = point_x; 
    double corner_point_y = point_y; 

    for(int i = 0; i<occupied_positions_x.size(); i++){

        if(occupied_positions_x[i] >= corner_point_x & occupied_positions_x[i] <= corner_new_point_x){

            if(occupied_positions_y[i] <= corner_point_y & occupied_positions_y[i] >= corner_new_point_y){
                inside_line = true;
            }
            else if(occupied_positions_y[i] >= corner_point_y & occupied_positions_y[i] <= corner_new_point_y){
                inside_line = true;
            }
        }
        
        else if(occupied_positions_x[i] <= corner_point_x & occupied_positions_x[i] >= corner_new_point_x){

            if(occupied_positions_y[i] <= corner_point_y & occupied_positions_y[i] >= corner_new_point_y){
                inside_line = true;
            }
            else if(occupied_positions_y[i] >= corner_point_y & occupied_positions_y[i] <= corner_new_point_y){
                inside_line = true;
            }
        }
    }

    return inside_line;
    
}







