/*
 * Copyright 2019 Gianluca Frison, Dimitris Kouzoupis, Robin Verschueren,
 * Andrea Zanelli, Niels van Duijkeren, Jonathan Frey, Tommaso Sartor,
 * Branimir Novoselnik, Rien Quirynen, Rezart Qelibari, Dang Doan,
 * Jonas Koenemann, Yutao Chen, Tobias Schöls, Jonas Schlagenhauf, Moritz Diehl
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */
// standard
#include <stdio.h>
#include <stdlib.h>

// acados
#include "acados_c/external_function_interface.h"
#include "acados_c/sim_interface.h"
#include "acados_c/external_function_interface.h"

#include "acados/sim/sim_common.h"
#include "acados/utils/external_function_generic.h"
#include "acados/utils/print.h"


// example specific
#include "rockit_model_model/rockit_model_model.h"
#include "acados_sim_solver_rockit_model.h"


// ** global data **
sim_config  * rockit_model_sim_config;
sim_in      * rockit_model_sim_in;
sim_out     * rockit_model_sim_out;
void        * rockit_model_sim_dims;
sim_opts    * rockit_model_sim_opts;
sim_solver  * rockit_model_sim_solver;




int rockit_model_acados_sim_create()
{
    // initialize
    int nx = 5;
    int nu = 3;
    int nz = 0;

    
    double Tsim = 0.5;

    

    // sim plan & config
    sim_solver_plan plan;
    plan.sim_solver = DISCRETE;

    // create correct config based on plan
    rockit_model_sim_config = sim_config_create(plan);

    // sim dims
    rockit_model_sim_dims = sim_dims_create(rockit_model_sim_config);
    sim_dims_set(rockit_model_sim_config, rockit_model_sim_dims, "nx", &nx);
    sim_dims_set(rockit_model_sim_config, rockit_model_sim_dims, "nu", &nu);
    sim_dims_set(rockit_model_sim_config, rockit_model_sim_dims, "nz", &nz);


    // sim opts
    rockit_model_sim_opts = sim_opts_create(rockit_model_sim_config, rockit_model_sim_dims);
    int tmp_int = 4;
    sim_opts_set(rockit_model_sim_config, rockit_model_sim_opts, "num_stages", &tmp_int);
    tmp_int = 1;
    sim_opts_set(rockit_model_sim_config, rockit_model_sim_opts, "num_steps", &tmp_int);
    tmp_int = 3;
    sim_opts_set(rockit_model_sim_config, rockit_model_sim_opts, "newton_iter", &tmp_int);
    bool tmp_bool = false;
    sim_opts_set(rockit_model_sim_config, rockit_model_sim_opts, "jac_reuse", &tmp_bool);



    // sim in / out
    rockit_model_sim_in  = sim_in_create(rockit_model_sim_config, rockit_model_sim_dims);
    rockit_model_sim_out = sim_out_create(rockit_model_sim_config, rockit_model_sim_dims);
    sim_in_set(rockit_model_sim_config, rockit_model_sim_dims,
               rockit_model_sim_in, "T", &Tsim);

    // model functions

    // sim solver
    rockit_model_sim_solver = sim_solver_create(rockit_model_sim_config,
                                               rockit_model_sim_dims, rockit_model_sim_opts);

    /* initialize parameter values */
    
    // initialize parameters to nominal value
    double p[66];
    
    p[0] = 0;
    p[1] = 0;
    p[2] = 0;
    p[3] = 0;
    p[4] = 1;
    p[5] = 1;
    p[6] = 0;
    p[7] = 0.15;
    p[8] = 0.3;
    p[9] = 0.45;
    p[10] = 0.6;
    p[11] = 0.75;
    p[12] = 0.8999999999999999;
    p[13] = 1.05;
    p[14] = 1.2;
    p[15] = 1.35;
    p[16] = 1.5;
    p[17] = 1.65;
    p[18] = 1.7999999999999998;
    p[19] = 1.95;
    p[20] = 2.1;
    p[21] = 2.25;
    p[22] = 2.4;
    p[23] = 2.55;
    p[24] = 2.6999999999999997;
    p[25] = 2.85;
    p[26] = 0;
    p[27] = 0.15;
    p[28] = 0.3;
    p[29] = 0.45;
    p[30] = 0.6;
    p[31] = 0.75;
    p[32] = 0.8999999999999999;
    p[33] = 1.05;
    p[34] = 1.2;
    p[35] = 1.35;
    p[36] = 1.5;
    p[37] = 1.65;
    p[38] = 1.7999999999999998;
    p[39] = 1.95;
    p[40] = 2.1;
    p[41] = 2.25;
    p[42] = 2.4;
    p[43] = 2.55;
    p[44] = 2.6999999999999997;
    p[45] = 2.85;
    p[46] = 1;
    p[47] = 1;
    p[48] = 1;
    p[49] = 1;
    p[50] = 1;
    p[51] = 1;
    p[52] = 1;
    p[53] = 1;
    p[54] = 1;
    p[55] = 1;
    p[56] = 1;
    p[57] = 1;
    p[58] = 1;
    p[59] = 1;
    p[60] = 1;
    p[61] = 1;
    p[62] = 1;
    p[63] = 1;
    p[64] = 1;
    p[65] = 1;
    

    /* initialize input */
    // x
    double x0[5];
    for (int ii = 0; ii < 5; ii++)
        x0[ii] = 0.0;

    sim_in_set(rockit_model_sim_config, rockit_model_sim_dims,
               rockit_model_sim_in, "x", x0);


    // u
    double u0[3];
    for (int ii = 0; ii < 3; ii++)
        u0[ii] = 0.0;

    sim_in_set(rockit_model_sim_config, rockit_model_sim_dims,
               rockit_model_sim_in, "u", u0);

    // S_forw
    double S_forw[40];
    for (int ii = 0; ii < 40; ii++)
        S_forw[ii] = 0.0;
    for (int ii = 0; ii < 5; ii++)
        S_forw[ii + ii * 5 ] = 1.0;


    sim_in_set(rockit_model_sim_config, rockit_model_sim_dims,
               rockit_model_sim_in, "S_forw", S_forw);

    int status = sim_precompute(rockit_model_sim_solver, rockit_model_sim_in, rockit_model_sim_out);

    return status;
}


int rockit_model_acados_sim_solve()
{
    // integrate dynamics using acados sim_solver
    int status = sim_solve(rockit_model_sim_solver,
                           rockit_model_sim_in, rockit_model_sim_out);
    if (status != 0)
        printf("error in rockit_model_acados_sim_solve()! Exiting.\n");

    return status;
}


int rockit_model_acados_sim_free()
{
    // free memory
    sim_solver_destroy(rockit_model_sim_solver);
    sim_in_destroy(rockit_model_sim_in);
    sim_out_destroy(rockit_model_sim_out);
    sim_opts_destroy(rockit_model_sim_opts);
    sim_dims_destroy(rockit_model_sim_dims);
    sim_config_destroy(rockit_model_sim_config);

    // free external function

    return 0;
}


int rockit_model_acados_sim_update_params(double *p, int np)
{
    int status = 0;
    int casadi_np = 66;

    if (casadi_np != np) {
        printf("rockit_model_acados_sim_update_params: trying to set %i parameters for external functions."
            " External function has %i parameters. Exiting.\n", np, casadi_np);
        exit(1);
    }

    return status;
}

/* getters pointers to C objects*/
sim_config * rockit_model_acados_get_sim_config()
{
    return rockit_model_sim_config;
};

sim_in * rockit_model_acados_get_sim_in()
{
    return rockit_model_sim_in;
};

sim_out * rockit_model_acados_get_sim_out()
{
    return rockit_model_sim_out;
};

void * rockit_model_acados_get_sim_dims()
{
    return rockit_model_sim_dims;
};

sim_opts * rockit_model_acados_get_sim_opts()
{
    return rockit_model_sim_opts;
};

sim_solver  * rockit_model_acados_get_sim_solver()
{
    return rockit_model_sim_solver;
};

