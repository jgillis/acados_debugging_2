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
#include "acados_c/sim_interface.h"
#include "acados_sim_solver_rockit_model.h"


int main()
{
    int status = 0;
    status = rockit_model_acados_sim_create();

    if (status)
    {
        printf("acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    // initial condition
    double x_current[5];
    x_current[0] = 0.0;
    x_current[1] = 0.0;
    x_current[2] = 0.0;
    x_current[3] = 0.0;
    x_current[4] = 0.0;

  
    x_current[0] = 0;
    x_current[1] = 0;
    x_current[2] = 0;
    x_current[3] = 0;
    x_current[4] = 0;
    
  


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
    

    rockit_model_acados_sim_update_params(p, 66);
  

    int n_sim_steps = 3;
    // solve ocp in loop
    for (int ii = 0; ii < n_sim_steps; ii++)
    {
        sim_in_set(rockit_model_sim_config, rockit_model_sim_dims,
            rockit_model_sim_in, "x", x_current);
        status = rockit_model_acados_sim_solve();

        if (status != ACADOS_SUCCESS)
        {
            printf("acados_solve() failed with status %d.\n", status);
        }

        sim_out_get(rockit_model_sim_config, rockit_model_sim_dims,
               rockit_model_sim_out, "x", x_current);
        
        printf("\nx_current, %d\n", ii);
        for (int jj = 0; jj < 5; jj++)
        {
            printf("%e\n", x_current[jj]);
        }
    }

    printf("\nPerformed %d simulation steps with acados integrator successfully.\n\n", n_sim_steps);

    // free solver
    status = rockit_model_acados_sim_free();
    if (status) {
        printf("rockit_model_acados_sim_free() returned status %d. \n", status);
    }

    return status;
}
