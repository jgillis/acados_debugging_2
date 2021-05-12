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
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados_solver_rockit_model.h"


int main()
{

    nlp_solver_capsule *acados_ocp_capsule = rockit_model_acados_create_capsule();
    int status = rockit_model_acados_create(acados_ocp_capsule);

    if (status)
    {
        printf("rockit_model_acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    ocp_nlp_config *nlp_config = rockit_model_acados_get_nlp_config(acados_ocp_capsule);
    ocp_nlp_dims *nlp_dims = rockit_model_acados_get_nlp_dims(acados_ocp_capsule);
    ocp_nlp_in *nlp_in = rockit_model_acados_get_nlp_in(acados_ocp_capsule);
    ocp_nlp_out *nlp_out = rockit_model_acados_get_nlp_out(acados_ocp_capsule);
    ocp_nlp_solver *nlp_solver = rockit_model_acados_get_nlp_solver(acados_ocp_capsule);
    void *nlp_opts = rockit_model_acados_get_nlp_opts(acados_ocp_capsule);

    // initial condition
    int idxbx0[6];
    idxbx0[0] = 0;
    idxbx0[1] = 0;
    idxbx0[2] = 1;
    idxbx0[3] = 2;
    idxbx0[4] = 3;
    idxbx0[5] = 4;

    double lbx0[6];
    double ubx0[6];
    lbx0[0] = 0;
    ubx0[0] = 100000;
    lbx0[1] = 0;
    ubx0[1] = 0;
    lbx0[2] = 0;
    ubx0[2] = 0;
    lbx0[3] = 0;
    ubx0[3] = 0;
    lbx0[4] = 0;
    ubx0[4] = 0;
    lbx0[5] = 0;
    ubx0[5] = 0;

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);

    // initialization for state values
    double x_init[5];
    x_init[0] = 0.0;
    x_init[1] = 0.0;
    x_init[2] = 0.0;
    x_init[3] = 0.0;
    x_init[4] = 0.0;

    // initial value for control input
    double u0[3];
    u0[0] = 0.0;
    u0[1] = 0.0;
    u0[2] = 0.0;
    // set parameters
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
    
    p[46] = 100;
    
    p[47] = 100;
    
    p[48] = 100;
    
    p[49] = 100;
    
    p[50] = 100;
    
    p[51] = 100;
    
    p[52] = 100;
    
    p[53] = 100;
    
    p[54] = 100;
    
    p[55] = 100;
    
    p[56] = 100;
    
    p[57] = 100;
    
    p[58] = 100;
    
    p[59] = 100;
    
    p[60] = 100;
    
    p[61] = 100;
    
    p[62] = 100;
    
    p[63] = 100;
    
    p[64] = 100;
    
    p[65] = 100;
    

    for (int ii = 0; ii <= 20; ii++)
    {
        rockit_model_acados_update_params(acados_ocp_capsule, ii, p, 66);
    }
  

    // prepare evaluation
    int NTIMINGS = 1;
    double min_time = 1e12;
    double kkt_norm_inf;
    double elapsed_time;
    int sqp_iter;

    double xtraj[5 * (20+1)];
    double utraj[3 * (20)];


    // solve ocp in loop
    int rti_phase = 0;

    for (int ii = 0; ii < NTIMINGS; ii++)
    {
        // initialize solution
        for (int i = 0; i <= nlp_dims->N; i++)
        {
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x_init);
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
        }
        ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "rti_phase", &rti_phase);
        status = rockit_model_acados_solve(acados_ocp_capsule);
        ocp_nlp_get(nlp_config, nlp_solver, "time_tot", &elapsed_time);
        min_time = MIN(elapsed_time, min_time);
    }

    /* print solution and statistics */
    for (int ii = 0; ii <= nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "x", &xtraj[ii*5]);
    for (int ii = 0; ii < nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "u", &utraj[ii*3]);

    printf("\n--- xtraj ---\n");
    d_print_exp_tran_mat( 5, 20+1, xtraj, 5 );
    printf("\n--- utraj ---\n");
    d_print_exp_tran_mat( 3, 20, utraj, 3 );
    // ocp_nlp_out_print(nlp_solver->dims, nlp_out);

    printf("\nsolved ocp %d times, solution printed above\n\n", NTIMINGS);

    if (status == ACADOS_SUCCESS)
    {
        printf("rockit_model_acados_solve(): SUCCESS!\n");
    }
    else
    {
        printf("rockit_model_acados_solve() failed with status %d.\n", status);
    }

    // get solution
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "kkt_norm_inf", &kkt_norm_inf);
    ocp_nlp_get(nlp_config, nlp_solver, "sqp_iter", &sqp_iter);

    rockit_model_acados_print_stats(acados_ocp_capsule);

    printf("\nSolver info:\n");
    printf(" SQP iterations %2d\n minimum time for %d solve %f [ms]\n KKT %e\n",
           sqp_iter, NTIMINGS, min_time*1000, kkt_norm_inf);

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

    return status;
}
