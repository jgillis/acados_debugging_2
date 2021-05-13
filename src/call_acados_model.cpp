
#include "call_acados_model.h"


void Controller::save_data(){


    std::ofstream myfile;
    myfile.open("/home/mohamad/Desktop/code_generation_2/plots/sol.txt");
    for (unsigned int i=0; i<20;i++){
            myfile<< x_sol[i]<<"   "<<y_sol[i]<<std::endl;
    }
    myfile.close();


}

void Controller::monitor(){

    if(monitor_initial_guess){

        std::cout<<"                                                      "<<std::endl;
        std::cout<<" ------------- MONITOR INITAL GUESS ----------------- "<<std::endl;

        std::cout<<"path_guess_x = [ ";
        for(int i = 0; i <= N; i++){
            std::cout<<path_guess_x[i]<<" , ";
        }
        std::cout<<" ]"<<std::endl;

        std::cout<<"path_guess_y = [ ";
        for(int i = 0; i <= N; i++){
            std::cout<<path_guess_y[i]<<" , ";
        }
        std::cout<<" ]"<<std::endl;

        std::cout<<"theta_guess = [ ";
        for(int i = 0; i <= N; i++){
            std::cout<<theta_guess[i]<<" , ";
        }
        std::cout<<" ]"<<std::endl;

        std::cout<<"s_guess = [ ";
        for(int i = 0; i <= N; i++){
            std::cout<<s_guess[i]<<" , ";
        }
        std::cout<<" ]"<<std::endl;

        std::cout<<"sdot_guess = [ ";
        for(int i = 0; i < N; ++i){
            std::cout<<sdot_guess[i]<<" , ";
        }
        std::cout<<" ]"<<std::endl;

        std::cout<<"v_guess = [ ";
        for(int i = 0; i < N; i++){
            std::cout<<v_guess[i]<<" , ";
        }
        std::cout<<" ]"<<std::endl;

        std::cout<<"w_guess = [ ";
        for(int i = 0; i < N; i++){
            std::cout<<w_guess[i]<<" , ";
        }
        std::cout<<" ]"<<std::endl;

    }

    if(monitor_solution){

        std::cout<<"x_sol = "<<x_sol<<std::endl;
        std::cout<<"y_sol = "<<y_sol<<std::endl;
        std::cout<<"theta_sol = "<<theta_sol<<std::endl;
        std::cout<<"s_sol = "<<s_sol<<std::endl;
        std::cout<<"w_sol = "<<w_sol<<std::endl;
        std::cout<<"v_sol = "<<v_sol<<std::endl;
        std::cout<<"sdot_sol = "<<sdot_sol<<std::endl;

    }

    if(monitor_parameters){
        for(int i = 0; i<p_size; i++){
            std::cout<<" p["<<i<<"] = "<<p[i]<<std::endl;
        }
    }

}

void Controller::create_path_guess(){

    // create path from pos to goal as guess 

    path_guess_x.clear();
    path_guess_y.clear();

    double X1 = state_x;
    double Y1 = state_y;

    double X2 = goal_waypoint_x;
    double Y2 = goal_waypoint_y;

    double a = (Y2- Y1)/(X2- X1) ;
    double b =  Y1 - a*X1;

    double x =  X1;
    double y =  Y1;

    double delta;

    if(x > X2){
        delta = (X1 - X2)/(N+1);
        while( x >= X2 )
        {
            path_guess_x.push_back(x);
            path_guess_y.push_back(a*x + b);
            x = x - delta;
        }
    }
    else if(x < X2){
        delta = (X2 - X1)/(N+1);
        while(x <= X2)
        {
            path_guess_x.push_back(x);
            path_guess_y.push_back(a*x + b);
            x = x + delta;
        }

    }
    else if(x == X2){

        delta = (abs(Y2 - y))/(N+1);

        if(y < Y2){

            while( y <= Y2)
            {
                path_guess_x.push_back(x);
                path_guess_y.push_back(y);
                y = y + delta;
            }
        }
        else if(y > Y2){

            while( y >= Y2)
            {
                path_guess_x.push_back(x);
                path_guess_y.push_back(y);
                y = y - delta;
            }
        }
    }

    std::cout<<"                        "<<std::endl;
    std::cout<<"--------------------------- length of path_guess is = "<<path_guess_x.size()<<std::endl; // should be = 21
    std::cout<<"                        "<<std::endl;

    // guesses for theta , v,  w 

    double xdot;
    double ydot;

    for(int i = 0; i<N ;i++){  //there is N+1 path guesses

        xdot = (path_guess_x[i+1] - path_guess_x[i])/dt;
        ydot = (path_guess_y[i+1] - path_guess_y[i])/dt;
        v_guess[i] = sqrt(xdot*xdot + ydot*ydot);       // v guess is N
        theta_guess[i] = asin(ydot/v_guess[i]);
    } 

    theta_guess[N] = theta_guess[N-1]; // theta guess is N+1

    for(int i = 0; i<N ;i++){
        w_guess[i] = (theta_guess[i+1] - theta_guess[i])/dt; //w guess is N+1
    }


    // --------------- guesses for s and sdot

    double deltas = 1.0/(N+1);
    s_guess[0] = 0;
    sdot_guess[0] = deltas/dt;

    for (int i = 0; i<N ; i++){
        s_guess[i+1] = s_guess[i] + delta;  // s guess is N+1
    }

    for (int i = 1; i<N ; i++){
        sdot_guess[i] = deltas/dt;  // sdot is N
    }


}

void Controller::set_parameters(){


    //--------------------------- set parameters

    std::cout<<"---------- set parameters p ----------------"<<std::endl;
       
    p[0] = state_x; //x0

    // p[0] = 0;

    p[1] = state_y; //y0

    // p[1] = 0; //x0

    p[2] = state_theta; //theta0

    // p[2] = 0;

    p[3] = state_s;

    p[4] = goal_waypoint_x; //end_goal_x

    // p[4] = 1;

    p[5] = goal_waypoint_y; //end_goal_y

    for(int i = 0; i<N; i++){
        p[6+i]     = midpoints_x[i];
        p[6+N+i]   = midpoints_y[i];
        // p[6+2*N+i] = radii[i];
        p[6+2*N+i] = 1; 
    }

}

void Controller::set_constraints(){

    for(int i = 0; i < nx; i++){
        ubx0[i] = outputs["ubx_0"][i];                
        // std::cout << "ubx0["<< i << "] = " << ubx0[i] <<std::endl;
    }

    for(int i = 0; i < nx; i++){
        lbx0[i] = outputs["lbx_0"][i];
        // std::cout<< "lbx0["<< i << "] = " << lbx0[i] <<std::endl;
    }

}

void Controller::init(){

    // ------------------- create model --------------------------------------------

    acados_ocp_capsule = rockit_model_acados_create_capsule();
    status = rockit_model_acados_create(acados_ocp_capsule);

    if (status)
    {
        printf("rockit_model_acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    nlp_config = rockit_model_acados_get_nlp_config(acados_ocp_capsule);
    nlp_dims = rockit_model_acados_get_nlp_dims(acados_ocp_capsule);
    nlp_in = rockit_model_acados_get_nlp_in(acados_ocp_capsule);
    nlp_out = rockit_model_acados_get_nlp_out(acados_ocp_capsule);
    nlp_solver = rockit_model_acados_get_nlp_solver(acados_ocp_capsule);
    nlp_opts = rockit_model_acados_get_nlp_opts(acados_ocp_capsule);


    // to get equivalence with solver from python 

    // char globalization[] = "merit_backtracking"; 
    // ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "globalization",&globalization);

    char globalization[] = "fixed_step";  // because look at json model
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "globalization",&globalization);

 
}

void Controller::run_acados_solver()
{

    Controller::set_parameters();

    //-------------- interact with casadi function to update bounds -----------------------------------

    casadi_int nout = rockitmmap_n_out();

    rockitmmap_work(&sz_arg, &sz_res, &sz_iw, &sz_w);

    std::vector<const casadi_real*> arg(sz_arg);
    std::vector<casadi_real*> res(sz_res);
    std::vector<casadi_int> iw(sz_iw);
    std::vector<casadi_real> w(sz_w);

    casadi_real t = 0;

    arg[0] = p;
    arg[1] = &t; 

    for (int i=0;i<rockitmmap_n_out();++i) {

        std::string name = rockitmmap_name_out(i);
        const casadi_int* sp = rockitmmap_sparsity_out(i);
        casadi_int nel = sp[0]*sp[1];
        outputs[name] = std::vector<casadi_real>(nel);
        outputs_sizes[name] = nel;
        res[i] = outputs[name].data();

    }

    rockitmmap(arg.data(), res.data(), iw.data(), w.data(), 0); //update outputs

    if(monitor_mmap_outputs){
        std::cout<<"----------- original mmap outputs ---------------- "<<std::endl;
    }

    for (int i=0;i<rockitmmap_n_out();++i) {

        std::string name = rockitmmap_name_out(i);

        for(int n=0;n<outputs_sizes[name];++n){ 
            
            //replace infinity by high number
            if(outputs[name][n] == - INFINITY)
                outputs[name][n] = - 100000;
            if(outputs[name][n] == INFINITY)
                outputs[name][n] = 100000;

            // cout 
            if(monitor_mmap_outputs){
                std::cout<<name<<"["<<n<<"] = "<<outputs[name][n]<<std::endl;
            }

        }

    }

    // --------- set constraints ----------------"

    Controller::set_constraints(); //fill data from outputs to constraint variables

    //--------------- INITIAL CONDITIONS 
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx",   lbx0); 
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx",   ubx0);


    // ------------------- set parameters ------------------------------------------------
    

    for (int ii = 0; ii <= N; ii++)
    {
        rockit_model_acados_update_params(acados_ocp_capsule, ii, p, p_size);
    }


    // ----------------------------- prepare evaluation -------------------------------

    double kkt_norm_inf;
    double elapsed_time;
    int sqp_iter;
    int rti_phase = 0;
    double xtraj[nx * (N+1)];
    double utraj[nu * (N)];


    //---------------- initialize solution ----------------------------------------------

    Controller::create_path_guess();


    if (first_run){ // initialization of the first run

        for(int i = 0; i<=N ; i++){

            // states
            x_init[i][0] = path_guess_x[i];
            x_init[i][1] = path_guess_y[i];
            x_init[i][2] = theta_guess[i];
            x_init[i][3] = s_guess[i]; 
            x_init[i][4] = 0;

            // x_init[i][0] = 0;
            // x_init[i][1] = 0;
            // x_init[i][2] = 0;
            // x_init[i][3] = 0; 
            // x_init[i][4] = 0;
        }

        for(int i = 0; i<N ; i++){

            //controls 
            u_init[i][0] = v_guess[i]; 
            u_init[i][1] = w_guess[i];
            u_init[i][2] = sdot_guess[i];

            // u_init[i][0] = 0; 
            // u_init[i][1] = 0;
            // u_init[i][2] = 0;

        }
    }
    else{ //warm starting

        for(int i = 0; i<=N ; i++){
            
            x_init[i][0] = x_sol[i];
            x_init[i][1] = y_sol[i];
            x_init[i][2] = theta_sol[i];
            x_init[i][3] = s_sol[i]; 
        }
        for(int i = 0; i<N ; i++){
            
            u_init[i][0] = v_sol[i];
            u_init[i][1] = w_sol[i];
            u_init[i][2] = sdot_sol[i];

        }
    }


    // ------------------- solve --------------------------------------------------------
    
    int NTIMINGS = 1;
    double min_time = 1e12;

    for (int ii = 0; ii < NTIMINGS; ii++)
    {
        // initialize solution
        for (int i = 0; i <= N; i++) 
        {   
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x_init[i]);
        }
        for (int i = 0; i < N; i++)
        {   
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u_init[i]);
        }

        ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "rti_phase", &rti_phase);
        status = rockit_model_acados_solve(acados_ocp_capsule);
        ocp_nlp_get(nlp_config, nlp_solver, "time_tot", &elapsed_time);
        min_time = MIN(elapsed_time, min_time);
    }

    // ---------------------- get solution ------------------------------------------------

    for (int ii = 0; ii <= nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "x", &xtraj[ii*nx]);

    for (int ii = 0; ii < nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "u", &utraj[ii*nu]);


    if (status == ACADOS_SUCCESS)
    {
        printf("rockit_model_acados_solve(): SUCCESS!\n");
    }
    else
    {
        printf("rockit_model_acados_solve() failed with status %d.\n", status);
        breakstatus = true;
    }

    // get solution   
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "kkt_norm_inf", &kkt_norm_inf);
    ocp_nlp_get(nlp_config, nlp_solver, "sqp_iter", &sqp_iter);

    if(print_solver_stats){
        rockit_model_acados_print_stats(acados_ocp_capsule);
    }

    printf("\nSolver info:\n");
    printf(" SQP iterations %2d\n solution time %f [ms]\n KKT %e\n",
           sqp_iter, elapsed_time*1000, kkt_norm_inf);
    

    for(int i = 0; i<N; i++ ){
        
        v_sol[i]     = utraj[0+i*nu];
        w_sol[i]     = utraj[1+i*nu];
        sdot_sol[i]  = utraj[2+i*nu];

        x_sol[i]     = xtraj[0+i*nx];
        y_sol[i]     = xtraj[1+i*nx];
        theta_sol[i] = xtraj[2+i*nx];
        s_sol[i]     = xtraj[3+i*nx];

    }

    first_run = false; // when this is called = it means that the first solution already exists
    
    Controller::monitor(); // monitor based on bools 

    ocp_nlp_print_problem(nlp_config, nlp_dims, nlp_in, nlp_out); //addition


}

void Controller::free_solver(){ // not used now

    // free solver ------------- not necessary, can be in discturctor
    
    // free solver
    status = rockit_model_acados_free(acados_ocp_capsule);

    if (status) {
        printf("rockit_model_acados_free() returned status %d. \n", status);
    }

    // free solver capsule
    status = rockit_model_acados_free_capsule(acados_ocp_capsule);
    if (status) {
        printf("rockit_model_acados_free_capsule() returned status %d. \n", status);
    }

}
