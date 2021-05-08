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
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

// example specific
#include "rockit_model_model/rockit_model_model.h"



#include "rockit_model_constraints/rockit_model_h_constraint.h"


#include "rockit_model_constraints/rockit_model_h_e_constraint.h"

#include "rockit_model_cost/rockit_model_external_cost.h"

#include "rockit_model_cost/rockit_model_external_cost_e.h"


#include "acados_solver_rockit_model.h"

#define NX     5
#define NZ     0
#define NU     3
#define NP     66
#define NBX    0
#define NBX0   5
#define NBU    3
#define NSBX   0
#define NSBU   0
#define NSH    0
#define NSG    0
#define NSPHI  0
#define NSHN   0
#define NSGN   0
#define NSPHIN 0
#define NSBXN  0
#define NS     0
#define NSN    0
#define NG     0
#define NBXN   1
#define NGN    0
#define NY     0
#define NYN    0
#define N      20
#define NH     1
#define NPHI   0
#define NHN    1
#define NPHIN  0
#define NR     0


// ** solver data **

nlp_solver_capsule * rockit_model_acados_create_capsule()
{
    void* capsule_mem = malloc(sizeof(nlp_solver_capsule));
    nlp_solver_capsule *capsule = (nlp_solver_capsule *) capsule_mem;

    return capsule;
}


int rockit_model_acados_free_capsule(nlp_solver_capsule *capsule)
{
    free(capsule);
    return 0;
}


int rockit_model_acados_create(nlp_solver_capsule * capsule)
{
    int status = 0;

    // number of expected runtime parameters
    capsule->nlp_np = NP;

    /************************************************
    *  plan & config
    ************************************************/
    ocp_nlp_plan * nlp_solver_plan = ocp_nlp_plan_create(N);
    capsule->nlp_solver_plan = nlp_solver_plan;
    nlp_solver_plan->nlp_solver = SQP;
    

    nlp_solver_plan->ocp_qp_solver_plan.qp_solver = FULL_CONDENSING_HPIPM;
    for (int i = 0; i < N; i++)
        nlp_solver_plan->nlp_cost[i] = EXTERNAL;

    nlp_solver_plan->nlp_cost[N] = EXTERNAL;

    for (int i = 0; i < N; i++)
    {
        
        nlp_solver_plan->nlp_dynamics[i] = DISCRETE_MODEL;
        // discrete dynamics does not need sim solver option, this field is ignored
        nlp_solver_plan->sim_solver_plan[i].sim_solver = INVALID_SIM_SOLVER;
        
    }

    for (int i = 0; i < N; i++)
    {
        nlp_solver_plan->nlp_constraints[i] = BGH;
    }
    nlp_solver_plan->nlp_constraints[N] = BGH;
    nlp_solver_plan->regularization = MIRROR;
    ocp_nlp_config * nlp_config = ocp_nlp_config_create(*nlp_solver_plan);
    capsule->nlp_config = nlp_config;


    /************************************************
    *  dimensions
    ************************************************/
    int nx[N+1];
    int nu[N+1];
    int nbx[N+1];
    int nbu[N+1];
    int nsbx[N+1];
    int nsbu[N+1];
    int nsg[N+1];
    int nsh[N+1];
    int nsphi[N+1];
    int ns[N+1];
    int ng[N+1];
    int nh[N+1];
    int nphi[N+1];
    int nz[N+1];
    int ny[N+1];
    int nr[N+1];
    int nbxe[N+1];

    for (int i = 0; i < N+1; i++)
    {
        // common
        nx[i]     = NX;
        nu[i]     = NU;
        nz[i]     = NZ;
        ns[i]     = NS;
        // cost
        ny[i]     = NY;
        // constraints
        nbx[i]    = NBX;
        nbu[i]    = NBU;
        nsbx[i]   = NSBX;
        nsbu[i]   = NSBU;
        nsg[i] = NSG;
        nsh[i]    = NSH;
        nsphi[i]  = NSPHI;
        ng[i]     = NG;
        nh[i]     = NH;
        nphi[i]   = NPHI;
        nr[i]     = NR;
        nbxe[i]   = 0;
    }

    // for initial state
    nbx[0]  = NBX0;
    nsbx[0] = 0;
    ns[0] = NS - NSBX;
    nbxe[0] = 0;

    // terminal - common
    nu[N]   = 0;
    nz[N]   = 0;
    ns[N]   = NSN;
    // cost
    ny[N]   = NYN;
    // constraint
    nbx[N]   = NBXN;
    nbu[N]   = 0;
    ng[N]    = NGN;
    nh[N]    = NHN;
    nphi[N]  = NPHIN;
    nr[N]    = 0;

    nsbx[N]  = NSBXN;
    nsbu[N]  = 0;
    nsg[N]   = NSGN;
    nsh[N]   = NSHN;
    nsphi[N] = NSPHIN;

    /* create and set ocp_nlp_dims */
    ocp_nlp_dims * nlp_dims = ocp_nlp_dims_create(nlp_config);
    capsule->nlp_dims = nlp_dims;

    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nx", nx);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nu", nu);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nz", nz);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "ns", ns);

    for (int i = 0; i <= N; i++)
    {
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbx", &nbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbu", &nbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbx", &nsbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbu", &nsbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "ng", &ng[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsg", &nsg[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbxe", &nbxe[i]);
    }

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nh", &nh[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsh", &nsh[i]);
    }
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nh", &nh[N]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nsh", &nsh[N]);



    /************************************************
    *  external functions
    ************************************************/
    capsule->nl_constr_h_fun_jac = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        capsule->nl_constr_h_fun_jac[i].casadi_fun = &rockit_model_constr_h_fun_jac_uxt_zt;
        capsule->nl_constr_h_fun_jac[i].casadi_n_in = &rockit_model_constr_h_fun_jac_uxt_zt_n_in;
        capsule->nl_constr_h_fun_jac[i].casadi_n_out = &rockit_model_constr_h_fun_jac_uxt_zt_n_out;
        capsule->nl_constr_h_fun_jac[i].casadi_sparsity_in = &rockit_model_constr_h_fun_jac_uxt_zt_sparsity_in;
        capsule->nl_constr_h_fun_jac[i].casadi_sparsity_out = &rockit_model_constr_h_fun_jac_uxt_zt_sparsity_out;
        capsule->nl_constr_h_fun_jac[i].casadi_work = &rockit_model_constr_h_fun_jac_uxt_zt_work;
        external_function_param_casadi_create(&capsule->nl_constr_h_fun_jac[i], 66);
    }
    capsule->nl_constr_h_fun = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        capsule->nl_constr_h_fun[i].casadi_fun = &rockit_model_constr_h_fun;
        capsule->nl_constr_h_fun[i].casadi_n_in = &rockit_model_constr_h_fun_n_in;
        capsule->nl_constr_h_fun[i].casadi_n_out = &rockit_model_constr_h_fun_n_out;
        capsule->nl_constr_h_fun[i].casadi_sparsity_in = &rockit_model_constr_h_fun_sparsity_in;
        capsule->nl_constr_h_fun[i].casadi_sparsity_out = &rockit_model_constr_h_fun_sparsity_out;
        capsule->nl_constr_h_fun[i].casadi_work = &rockit_model_constr_h_fun_work;
        external_function_param_casadi_create(&capsule->nl_constr_h_fun[i], 66);
    }
    
    capsule->nl_constr_h_fun_jac_hess = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        capsule->nl_constr_h_fun_jac_hess[i].casadi_fun = &rockit_model_constr_h_fun_jac_uxt_hess;
        capsule->nl_constr_h_fun_jac_hess[i].casadi_n_in = &rockit_model_constr_h_fun_jac_uxt_hess_n_in;
        capsule->nl_constr_h_fun_jac_hess[i].casadi_n_out = &rockit_model_constr_h_fun_jac_uxt_hess_n_out;
        capsule->nl_constr_h_fun_jac_hess[i].casadi_sparsity_in = &rockit_model_constr_h_fun_jac_uxt_hess_sparsity_in;
        capsule->nl_constr_h_fun_jac_hess[i].casadi_sparsity_out = &rockit_model_constr_h_fun_jac_uxt_hess_sparsity_out;
        capsule->nl_constr_h_fun_jac_hess[i].casadi_work = &rockit_model_constr_h_fun_jac_uxt_hess_work;

        external_function_param_casadi_create(&capsule->nl_constr_h_fun_jac_hess[i], 66);
    }
    
    
    capsule->nl_constr_h_e_fun_jac.casadi_fun = &rockit_model_constr_h_e_fun_jac_uxt_zt;
    capsule->nl_constr_h_e_fun_jac.casadi_n_in = &rockit_model_constr_h_e_fun_jac_uxt_zt_n_in;
    capsule->nl_constr_h_e_fun_jac.casadi_n_out = &rockit_model_constr_h_e_fun_jac_uxt_zt_n_out;
    capsule->nl_constr_h_e_fun_jac.casadi_sparsity_in = &rockit_model_constr_h_e_fun_jac_uxt_zt_sparsity_in;
    capsule->nl_constr_h_e_fun_jac.casadi_sparsity_out = &rockit_model_constr_h_e_fun_jac_uxt_zt_sparsity_out;
    capsule->nl_constr_h_e_fun_jac.casadi_work = &rockit_model_constr_h_e_fun_jac_uxt_zt_work;
    external_function_param_casadi_create(&capsule->nl_constr_h_e_fun_jac, 66);

    capsule->nl_constr_h_e_fun.casadi_fun = &rockit_model_constr_h_e_fun;
    capsule->nl_constr_h_e_fun.casadi_n_in = &rockit_model_constr_h_e_fun_n_in;
    capsule->nl_constr_h_e_fun.casadi_n_out = &rockit_model_constr_h_e_fun_n_out;
    capsule->nl_constr_h_e_fun.casadi_sparsity_in = &rockit_model_constr_h_e_fun_sparsity_in;
    capsule->nl_constr_h_e_fun.casadi_sparsity_out = &rockit_model_constr_h_e_fun_sparsity_out;
    capsule->nl_constr_h_e_fun.casadi_work = &rockit_model_constr_h_e_fun_work;
    external_function_param_casadi_create(&capsule->nl_constr_h_e_fun, 66);

    
    capsule->nl_constr_h_e_fun_jac_hess.casadi_fun = &rockit_model_constr_h_e_fun_jac_uxt_hess;
    capsule->nl_constr_h_e_fun_jac_hess.casadi_n_in = &rockit_model_constr_h_e_fun_jac_uxt_hess_n_in;
    capsule->nl_constr_h_e_fun_jac_hess.casadi_n_out = &rockit_model_constr_h_e_fun_jac_uxt_hess_n_out;
    capsule->nl_constr_h_e_fun_jac_hess.casadi_sparsity_in = &rockit_model_constr_h_e_fun_jac_uxt_hess_sparsity_in;
    capsule->nl_constr_h_e_fun_jac_hess.casadi_sparsity_out = &rockit_model_constr_h_e_fun_jac_uxt_hess_sparsity_out;
    capsule->nl_constr_h_e_fun_jac_hess.casadi_work = &rockit_model_constr_h_e_fun_jac_uxt_hess_work;
    external_function_param_casadi_create(&capsule->nl_constr_h_e_fun_jac_hess, 66);
    


    // discrete dynamics
    capsule->discr_dyn_phi_fun = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N; i++)
    {
        capsule->discr_dyn_phi_fun[i].casadi_fun = &rockit_model_dyn_disc_phi_fun;
        capsule->discr_dyn_phi_fun[i].casadi_n_in = &rockit_model_dyn_disc_phi_fun_n_in;
        capsule->discr_dyn_phi_fun[i].casadi_n_out = &rockit_model_dyn_disc_phi_fun_n_out;
        capsule->discr_dyn_phi_fun[i].casadi_sparsity_in = &rockit_model_dyn_disc_phi_fun_sparsity_in;
        capsule->discr_dyn_phi_fun[i].casadi_sparsity_out = &rockit_model_dyn_disc_phi_fun_sparsity_out;
        capsule->discr_dyn_phi_fun[i].casadi_work = &rockit_model_dyn_disc_phi_fun_work;
        external_function_param_casadi_create(&capsule->discr_dyn_phi_fun[i], 66);
    }
    
    capsule->discr_dyn_phi_fun_jac_ut_xt = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N; i++)
    {
        capsule->discr_dyn_phi_fun_jac_ut_xt[i].casadi_fun = &rockit_model_dyn_disc_phi_fun_jac;
        capsule->discr_dyn_phi_fun_jac_ut_xt[i].casadi_n_in = &rockit_model_dyn_disc_phi_fun_jac_n_in;
        capsule->discr_dyn_phi_fun_jac_ut_xt[i].casadi_n_out = &rockit_model_dyn_disc_phi_fun_jac_n_out;
        capsule->discr_dyn_phi_fun_jac_ut_xt[i].casadi_sparsity_in = &rockit_model_dyn_disc_phi_fun_jac_sparsity_in;
        capsule->discr_dyn_phi_fun_jac_ut_xt[i].casadi_sparsity_out = &rockit_model_dyn_disc_phi_fun_jac_sparsity_out;
        capsule->discr_dyn_phi_fun_jac_ut_xt[i].casadi_work = &rockit_model_dyn_disc_phi_fun_jac_work;
        external_function_param_casadi_create(&capsule->discr_dyn_phi_fun_jac_ut_xt[i], 66);
    }
    capsule->discr_dyn_phi_fun_jac_ut_xt_hess = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    
    for (int i = 0; i < N; i++)
    {
        capsule->discr_dyn_phi_fun_jac_ut_xt_hess[i].casadi_fun = &rockit_model_dyn_disc_phi_fun_jac_hess;
        capsule->discr_dyn_phi_fun_jac_ut_xt_hess[i].casadi_n_in = &rockit_model_dyn_disc_phi_fun_jac_hess_n_in;
        capsule->discr_dyn_phi_fun_jac_ut_xt_hess[i].casadi_n_out = &rockit_model_dyn_disc_phi_fun_jac_hess_n_out;
        capsule->discr_dyn_phi_fun_jac_ut_xt_hess[i].casadi_sparsity_in = &rockit_model_dyn_disc_phi_fun_jac_hess_sparsity_in;
        capsule->discr_dyn_phi_fun_jac_ut_xt_hess[i].casadi_sparsity_out = &rockit_model_dyn_disc_phi_fun_jac_hess_sparsity_out;
        capsule->discr_dyn_phi_fun_jac_ut_xt_hess[i].casadi_work = &rockit_model_dyn_disc_phi_fun_jac_hess_work;
        external_function_param_casadi_create(&capsule->discr_dyn_phi_fun_jac_ut_xt_hess[i], 66);
    }
    // external cost
    capsule->ext_cost_fun = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N; i++)
    {
        capsule->ext_cost_fun[i].casadi_fun = &rockit_model_cost_ext_cost_fun;
        capsule->ext_cost_fun[i].casadi_n_in = &rockit_model_cost_ext_cost_fun_n_in;
        capsule->ext_cost_fun[i].casadi_n_out = &rockit_model_cost_ext_cost_fun_n_out;
        capsule->ext_cost_fun[i].casadi_sparsity_in = &rockit_model_cost_ext_cost_fun_sparsity_in;
        capsule->ext_cost_fun[i].casadi_sparsity_out = &rockit_model_cost_ext_cost_fun_sparsity_out;
        capsule->ext_cost_fun[i].casadi_work = &rockit_model_cost_ext_cost_fun_work;

        external_function_param_casadi_create(&capsule->ext_cost_fun[i], 66);
    }

    capsule->ext_cost_fun_jac = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N; i++)
    {
        // residual function
        capsule->ext_cost_fun_jac[i].casadi_fun = &rockit_model_cost_ext_cost_fun_jac;
        capsule->ext_cost_fun_jac[i].casadi_n_in = &rockit_model_cost_ext_cost_fun_jac_n_in;
        capsule->ext_cost_fun_jac[i].casadi_n_out = &rockit_model_cost_ext_cost_fun_jac_n_out;
        capsule->ext_cost_fun_jac[i].casadi_sparsity_in = &rockit_model_cost_ext_cost_fun_jac_sparsity_in;
        capsule->ext_cost_fun_jac[i].casadi_sparsity_out = &rockit_model_cost_ext_cost_fun_jac_sparsity_out;
        capsule->ext_cost_fun_jac[i].casadi_work = &rockit_model_cost_ext_cost_fun_jac_work;

        external_function_param_casadi_create(&capsule->ext_cost_fun_jac[i], 66);
    }

    capsule->ext_cost_fun_jac_hess = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N; i++)
    {
        // residual function
        capsule->ext_cost_fun_jac_hess[i].casadi_fun = &rockit_model_cost_ext_cost_fun_jac_hess;
        capsule->ext_cost_fun_jac_hess[i].casadi_n_in = &rockit_model_cost_ext_cost_fun_jac_hess_n_in;
        capsule->ext_cost_fun_jac_hess[i].casadi_n_out = &rockit_model_cost_ext_cost_fun_jac_hess_n_out;
        capsule->ext_cost_fun_jac_hess[i].casadi_sparsity_in = &rockit_model_cost_ext_cost_fun_jac_hess_sparsity_in;
        capsule->ext_cost_fun_jac_hess[i].casadi_sparsity_out = &rockit_model_cost_ext_cost_fun_jac_hess_sparsity_out;
        capsule->ext_cost_fun_jac_hess[i].casadi_work = &rockit_model_cost_ext_cost_fun_jac_hess_work;

        external_function_param_casadi_create(&capsule->ext_cost_fun_jac_hess[i], 66);
    }
    // external cost
    capsule->ext_cost_e_fun.casadi_fun = &rockit_model_cost_ext_cost_e_fun;
    capsule->ext_cost_e_fun.casadi_n_in = &rockit_model_cost_ext_cost_e_fun_n_in;
    capsule->ext_cost_e_fun.casadi_n_out = &rockit_model_cost_ext_cost_e_fun_n_out;
    capsule->ext_cost_e_fun.casadi_sparsity_in = &rockit_model_cost_ext_cost_e_fun_sparsity_in;
    capsule->ext_cost_e_fun.casadi_sparsity_out = &rockit_model_cost_ext_cost_e_fun_sparsity_out;
    capsule->ext_cost_e_fun.casadi_work = &rockit_model_cost_ext_cost_e_fun_work;
    external_function_param_casadi_create(&capsule->ext_cost_e_fun, 66);

    // external cost
    capsule->ext_cost_e_fun_jac.casadi_fun = &rockit_model_cost_ext_cost_e_fun_jac;
    capsule->ext_cost_e_fun_jac.casadi_n_in = &rockit_model_cost_ext_cost_e_fun_jac_n_in;
    capsule->ext_cost_e_fun_jac.casadi_n_out = &rockit_model_cost_ext_cost_e_fun_jac_n_out;
    capsule->ext_cost_e_fun_jac.casadi_sparsity_in = &rockit_model_cost_ext_cost_e_fun_jac_sparsity_in;
    capsule->ext_cost_e_fun_jac.casadi_sparsity_out = &rockit_model_cost_ext_cost_e_fun_jac_sparsity_out;
    capsule->ext_cost_e_fun_jac.casadi_work = &rockit_model_cost_ext_cost_e_fun_jac_work;
    external_function_param_casadi_create(&capsule->ext_cost_e_fun_jac, 66);

    // external cost
    capsule->ext_cost_e_fun_jac_hess.casadi_fun = &rockit_model_cost_ext_cost_e_fun_jac_hess;
    capsule->ext_cost_e_fun_jac_hess.casadi_n_in = &rockit_model_cost_ext_cost_e_fun_jac_hess_n_in;
    capsule->ext_cost_e_fun_jac_hess.casadi_n_out = &rockit_model_cost_ext_cost_e_fun_jac_hess_n_out;
    capsule->ext_cost_e_fun_jac_hess.casadi_sparsity_in = &rockit_model_cost_ext_cost_e_fun_jac_hess_sparsity_in;
    capsule->ext_cost_e_fun_jac_hess.casadi_sparsity_out = &rockit_model_cost_ext_cost_e_fun_jac_hess_sparsity_out;
    capsule->ext_cost_e_fun_jac_hess.casadi_work = &rockit_model_cost_ext_cost_e_fun_jac_hess_work;
    external_function_param_casadi_create(&capsule->ext_cost_e_fun_jac_hess, 66);

    /************************************************
    *  nlp_in
    ************************************************/
    ocp_nlp_in * nlp_in = ocp_nlp_in_create(nlp_config, nlp_dims);
    capsule->nlp_in = nlp_in;

    double time_steps[N];
    time_steps[0] = 0.5;
    time_steps[1] = 0.5;
    time_steps[2] = 0.5000000000000002;
    time_steps[3] = 0.4999999999999998;
    time_steps[4] = 0.5;
    time_steps[5] = 0.5000000000000004;
    time_steps[6] = 0.5;
    time_steps[7] = 0.4999999999999995;
    time_steps[8] = 0.5;
    time_steps[9] = 0.5;
    time_steps[10] = 0.5;
    time_steps[11] = 0.5000000000000009;
    time_steps[12] = 0.4999999999999991;
    time_steps[13] = 0.5000000000000009;
    time_steps[14] = 0.4999999999999991;
    time_steps[15] = 0.5;
    time_steps[16] = 0.5;
    time_steps[17] = 0.5;
    time_steps[18] = 0.5;
    time_steps[19] = 0.5;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &time_steps[i]);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &time_steps[i]);
    }

    /**** Dynamics ****/
    for (int i = 0; i < N; i++)
    {
        ocp_nlp_dynamics_model_set(nlp_config, nlp_dims, nlp_in, i, "disc_dyn_fun", &capsule->discr_dyn_phi_fun[i]);
        ocp_nlp_dynamics_model_set(nlp_config, nlp_dims, nlp_in, i, "disc_dyn_fun_jac",
                                   &capsule->discr_dyn_phi_fun_jac_ut_xt[i]);
        ocp_nlp_dynamics_model_set(nlp_config, nlp_dims, nlp_in, i, "disc_dyn_fun_jac_hess",
                                   &capsule->discr_dyn_phi_fun_jac_ut_xt_hess[i]);
    }


    /**** Cost ****/
    for (int i = 0; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "ext_cost_fun", &capsule->ext_cost_fun[i]);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "ext_cost_fun_jac", &capsule->ext_cost_fun_jac[i]);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "ext_cost_fun_jac_hess", &capsule->ext_cost_fun_jac_hess[i]);
    }




    // terminal cost

    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "ext_cost_fun", &capsule->ext_cost_e_fun);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "ext_cost_fun_jac", &capsule->ext_cost_e_fun_jac);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "ext_cost_fun_jac_hess", &capsule->ext_cost_e_fun_jac_hess);



    /**** Constraints ****/

    // bounds for initial stage

    // x0
    int idxbx0[5];
    
    idxbx0[0] = 0;
    idxbx0[1] = 1;
    idxbx0[2] = 2;
    idxbx0[3] = 3;
    idxbx0[4] = 4;

    double lbx0[5];
    double ubx0[5];
    
    lbx0[0] = 0;
    ubx0[0] = 0;
    lbx0[1] = 0;
    ubx0[1] = 0;
    lbx0[2] = 0;
    ubx0[2] = 0;
    lbx0[3] = 0;
    ubx0[3] = 0;
    lbx0[4] = 0;
    ubx0[4] = 0;

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);



    /* constraints that are the same for initial and intermediate */



    // u
    int idxbu[NBU];
    
    idxbu[0] = 0;
    idxbu[1] = 1;
    idxbu[2] = 2;
    double lbu[NBU];
    double ubu[NBU];
    
    lbu[0] = 0;
    ubu[0] = 1;
    lbu[1] = -3.141592653589793;
    ubu[1] = 3.141592653589793;
    lbu[2] = 0;
    ubu[2] = 100000;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "idxbu", idxbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbu", lbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubu", ubu);
    }















    // set up nonlinear constraints for stage 0 to N-1 
    double lh[NH];
    double uh[NH];

    
    lh[0] = -100000;

    
    uh[0] = 0;
    
    for (int i = 0; i < N; i++)
    {
        // nonlinear constraints for stages 0 to N-1
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "nl_constr_h_fun_jac",
                                      &capsule->nl_constr_h_fun_jac[i]);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "nl_constr_h_fun",
                                      &capsule->nl_constr_h_fun[i]);
        
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i,
                                      "nl_constr_h_fun_jac_hess", &capsule->nl_constr_h_fun_jac_hess[i]);
        
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lh", lh);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "uh", uh);
    }




    /* terminal constraints */

    // set up bounds for last stage
    // x
    int idxbx_e[NBXN];
    
    idxbx_e[0] = 3;
    double lbx_e[NBXN];
    double ubx_e[NBXN];
    
    lbx_e[0] = -100000;
    ubx_e[0] = 1;
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, N, "idxbx", idxbx_e);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, N, "lbx", lbx_e);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, N, "ubx", ubx_e);












    // set up nonlinear constraints for last stage 
    double lh_e[NHN];
    double uh_e[NHN];

    
    lh_e[0] = -100000;

    
    uh_e[0] = 0;

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, N, "nl_constr_h_fun_jac", &capsule->nl_constr_h_e_fun_jac);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, N, "nl_constr_h_fun", &capsule->nl_constr_h_e_fun);
    
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, N, "nl_constr_h_fun_jac_hess",
                                  &capsule->nl_constr_h_e_fun_jac_hess);
    
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, N, "lh", lh_e);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, N, "uh", uh_e);




    /************************************************
    *  opts
    ************************************************/

    capsule->nlp_opts = ocp_nlp_solver_opts_create(nlp_config, nlp_dims);


    bool nlp_solver_exact_hessian = true;
    // TODO: this if should not be needed! however, calling the setter with false leads to weird behavior. Investigate!
    if (nlp_solver_exact_hessian)
    {
        ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "exact_hess", &nlp_solver_exact_hessian);
    }
    int exact_hess_dyn = 1;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "exact_hess_dyn", &exact_hess_dyn);

    int exact_hess_cost = 1;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "exact_hess_cost", &exact_hess_cost);

    int exact_hess_constr = 1;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "exact_hess_constr", &exact_hess_constr);
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "globalization", "fixed_step");

    double nlp_solver_step_length = 1;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "step_length", &nlp_solver_step_length);

    double levenberg_marquardt = 0;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "levenberg_marquardt", &levenberg_marquardt);

    /* options QP solver */

    int qp_solver_iter_max = 1000;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "qp_iter_max", &qp_solver_iter_max);
    // set SQP specific options
    double nlp_solver_tol_stat = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "tol_stat", &nlp_solver_tol_stat);

    double nlp_solver_tol_eq = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "tol_eq", &nlp_solver_tol_eq);

    double nlp_solver_tol_ineq = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "tol_ineq", &nlp_solver_tol_ineq);

    double nlp_solver_tol_comp = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "tol_comp", &nlp_solver_tol_comp);

    int nlp_solver_max_iter = 500;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "max_iter", &nlp_solver_max_iter);

    int initialize_t_slacks = 0;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "initialize_t_slacks", &initialize_t_slacks);

    int print_level = 0;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "print_level", &print_level);


    int ext_cost_num_hess = 0;
    for (int i = 0; i < N; i++)
    {
        ocp_nlp_solver_opts_set_at_stage(nlp_config, capsule->nlp_opts, i, "cost_numerical_hessian", &ext_cost_num_hess);
    }
    ocp_nlp_solver_opts_set_at_stage(nlp_config, capsule->nlp_opts, N, "cost_numerical_hessian", &ext_cost_num_hess);


    /* out */
    ocp_nlp_out * nlp_out = ocp_nlp_out_create(nlp_config, nlp_dims);
    capsule->nlp_out = nlp_out;

    // initialize primal solution
    double x0[5];

    // initialize with x0
    
    x0[0] = 0;
    x0[1] = 0;
    x0[2] = 0;
    x0[3] = 0;
    x0[4] = 0;


    double u0[NU];
    
    u0[0] = 0.0;
    u0[1] = 0.0;
    u0[2] = 0.0;

    for (int i = 0; i < N; i++)
    {
        // x0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x0);
        // u0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
    }
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x0);
    
    capsule->nlp_solver = ocp_nlp_solver_create(nlp_config, nlp_dims, capsule->nlp_opts);



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

    for (int i = 0; i <= N; i++)
    {
        rockit_model_acados_update_params(capsule, i, p, NP);
    }

    status = ocp_nlp_precompute(capsule->nlp_solver, nlp_in, nlp_out);

    if (status != ACADOS_SUCCESS)
    {
        printf("\nocp_precompute failed!\n\n");
        exit(1);
    }

    return status;
}


int rockit_model_acados_update_params(nlp_solver_capsule * capsule, int stage, double *p, int np)
{
    int solver_status = 0;

    int casadi_np = 66;
    if (casadi_np != np) {
        printf("acados_update_params: trying to set %i parameters for external functions."
            " External function has %i parameters. Exiting.\n", np, casadi_np);
        exit(1);
    }
    if (stage < 20)
    {
        capsule->discr_dyn_phi_fun[stage].set_param(capsule->discr_dyn_phi_fun+stage, p);
        capsule->discr_dyn_phi_fun_jac_ut_xt[stage].set_param(capsule->discr_dyn_phi_fun_jac_ut_xt+stage, p);
        capsule->discr_dyn_phi_fun_jac_ut_xt_hess[stage].set_param(capsule->discr_dyn_phi_fun_jac_ut_xt_hess+stage, p);
    

        // constraints
    
        capsule->nl_constr_h_fun_jac[stage].set_param(capsule->nl_constr_h_fun_jac+stage, p);
        capsule->nl_constr_h_fun[stage].set_param(capsule->nl_constr_h_fun+stage, p);
        capsule->nl_constr_h_fun_jac_hess[stage].set_param(capsule->nl_constr_h_fun_jac_hess+stage, p);

        // cost
        capsule->ext_cost_fun[stage].set_param(capsule->ext_cost_fun+stage, p);
        capsule->ext_cost_fun_jac[stage].set_param(capsule->ext_cost_fun_jac+stage, p);
        capsule->ext_cost_fun_jac_hess[stage].set_param(capsule->ext_cost_fun_jac_hess+stage, p);

    }
    else // stage == N
    {
        // terminal shooting node has no dynamics
        // cost
        capsule->ext_cost_e_fun.set_param(&capsule->ext_cost_e_fun, p);
        capsule->ext_cost_e_fun_jac.set_param(&capsule->ext_cost_e_fun_jac, p);
    //
        capsule->ext_cost_e_fun_jac_hess.set_param(&capsule->ext_cost_e_fun_jac_hess, p);
    //
    
        // constraints
    
        capsule->nl_constr_h_e_fun_jac.set_param(&capsule->nl_constr_h_e_fun_jac, p);
        capsule->nl_constr_h_e_fun.set_param(&capsule->nl_constr_h_e_fun, p);
        capsule->nl_constr_h_e_fun_jac_hess.set_param(&capsule->nl_constr_h_e_fun_jac_hess, p);
    
    }


    return solver_status;
}



int rockit_model_acados_solve(nlp_solver_capsule * capsule)
{
    // solve NLP 
    int solver_status = ocp_nlp_solve(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    return solver_status;
}


int rockit_model_acados_free(nlp_solver_capsule * capsule)
{
    // free memory
    ocp_nlp_solver_opts_destroy(capsule->nlp_opts);
    ocp_nlp_in_destroy(capsule->nlp_in);
    ocp_nlp_out_destroy(capsule->nlp_out);
    ocp_nlp_solver_destroy(capsule->nlp_solver);
    ocp_nlp_dims_destroy(capsule->nlp_dims);
    ocp_nlp_config_destroy(capsule->nlp_config);
    ocp_nlp_plan_destroy(capsule->nlp_solver_plan);

    /* free external function */
    // dynamics
    for (int i = 0; i < 20; i++)
    {
        external_function_param_casadi_free(&capsule->discr_dyn_phi_fun[i]);
        external_function_param_casadi_free(&capsule->discr_dyn_phi_fun_jac_ut_xt[i]);
        external_function_param_casadi_free(&capsule->discr_dyn_phi_fun_jac_ut_xt_hess[i]);
    }
    free(capsule->discr_dyn_phi_fun);
    free(capsule->discr_dyn_phi_fun_jac_ut_xt);
    free(capsule->discr_dyn_phi_fun_jac_ut_xt_hess);

    // cost
    for (int i = 0; i < 20; i++)
    {
        external_function_param_casadi_free(&capsule->ext_cost_fun[i]);
        external_function_param_casadi_free(&capsule->ext_cost_fun_jac[i]);
        external_function_param_casadi_free(&capsule->ext_cost_fun_jac_hess[i]);
    }
    free(capsule->ext_cost_fun);
    free(capsule->ext_cost_fun_jac);
    free(capsule->ext_cost_fun_jac_hess);
    external_function_param_casadi_free(&capsule->ext_cost_e_fun);
    external_function_param_casadi_free(&capsule->ext_cost_e_fun_jac);
    external_function_param_casadi_free(&capsule->ext_cost_e_fun_jac_hess);

    // constraints
    for (int i = 0; i < 20; i++)
    {
        external_function_param_casadi_free(&capsule->nl_constr_h_fun_jac[i]);
        external_function_param_casadi_free(&capsule->nl_constr_h_fun[i]);
    }
    for (int i = 0; i < 20; i++)
    {
        external_function_param_casadi_free(&capsule->nl_constr_h_fun_jac_hess[i]);
    }
    free(capsule->nl_constr_h_fun_jac);
    free(capsule->nl_constr_h_fun);
    free(capsule->nl_constr_h_fun_jac_hess);
    external_function_param_casadi_free(&capsule->nl_constr_h_e_fun_jac);
    external_function_param_casadi_free(&capsule->nl_constr_h_e_fun);
    external_function_param_casadi_free(&capsule->nl_constr_h_e_fun_jac_hess);

    return 0;
}

ocp_nlp_in *rockit_model_acados_get_nlp_in(nlp_solver_capsule * capsule) { return capsule->nlp_in; }
ocp_nlp_out *rockit_model_acados_get_nlp_out(nlp_solver_capsule * capsule) { return capsule->nlp_out; }
ocp_nlp_solver *rockit_model_acados_get_nlp_solver(nlp_solver_capsule * capsule) { return capsule->nlp_solver; }
ocp_nlp_config *rockit_model_acados_get_nlp_config(nlp_solver_capsule * capsule) { return capsule->nlp_config; }
void *rockit_model_acados_get_nlp_opts(nlp_solver_capsule * capsule) { return capsule->nlp_opts; }
ocp_nlp_dims *rockit_model_acados_get_nlp_dims(nlp_solver_capsule * capsule) { return capsule->nlp_dims; }
ocp_nlp_plan *rockit_model_acados_get_nlp_plan(nlp_solver_capsule * capsule) { return capsule->nlp_solver_plan; }


void rockit_model_acados_print_stats(nlp_solver_capsule * capsule)
{
    int sqp_iter, stat_m, stat_n, tmp_int;
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "sqp_iter", &sqp_iter);
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "stat_n", &stat_n);
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "stat_m", &stat_m);

    
    double stat[5000];
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "statistics", stat);

    int nrow = sqp_iter+1 < stat_m ? sqp_iter+1 : stat_m;

    printf("iter\tres_stat\tres_eq\t\tres_ineq\tres_comp\tqp_stat\tqp_iter\n");
    for (int i = 0; i < nrow; i++)
    {
        for (int j = 0; j < stat_n + 1; j++)
        {
            if (j == 0 || j > 4)
            {
                tmp_int = (int) stat[i + j * nrow];
                printf("%d\t", tmp_int);
            }
            else
            {
                printf("%e\t", stat[i + j * nrow]);
            }
        }
        printf("\n");
    }
}

