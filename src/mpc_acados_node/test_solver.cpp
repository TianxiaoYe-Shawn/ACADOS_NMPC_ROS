#include "types_lib.h"
#include <vector>
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <fstream>

using namespace std;

#define NX          JACKAL_NX  
#define NU          JACKAL_NU  
#define NP          JACKAL_NP  
#define NY          JACKAL_NY  
#define NYN         JACKAL_NYN 
#define N           JACKAL_N   
#define TF 2                    
#define Ts 0.1                

int main() {
    /***************************************** ACADOS_C_POINTERS *****************************************/
    jackal_solver_capsule *acados_ocp_capsule = jackal_acados_create_capsule();
    int status = jackal_acados_create(acados_ocp_capsule);
    if (status) {
        std::cerr << "jackal_acados_create() returned status " << status << ". Exiting." << std::endl;
        return 1;
    }
    ocp_nlp_config *nlp_config = jackal_acados_get_nlp_config(acados_ocp_capsule);
    ocp_nlp_dims *nlp_dims = jackal_acados_get_nlp_dims(acados_ocp_capsule);
    ocp_nlp_in *nlp_in = jackal_acados_get_nlp_in(acados_ocp_capsule);
    ocp_nlp_out *nlp_out = jackal_acados_get_nlp_out(acados_ocp_capsule);
    ocp_nlp_solver *nlp_solver = jackal_acados_get_nlp_solver(acados_ocp_capsule);
    void *nlp_opts = jackal_acados_get_nlp_opts(acados_ocp_capsule);
    /******************************************************************************************************/

    /***************************************** ACADOS_C_INTERFACE *****************************************/
    // x0
    double acados_x0[NX];

    // ref
    vector<double> ref_states((N + 1) * NX);

	// set ref (part of a circle)
	double xc = 0.0;       
	double yc = 0.0;       
	double radius = 0.5;   

	double theta_start = 0.0;           
	double theta_end = M_PI / 2;        

	for (int i = 0; i <= N; i++) {
		double theta = theta_start + (theta_end - theta_start) * i / N;
		ref_states[i * NX + 0] = xc + radius * cos(theta);            
		ref_states[i * NX + 1] = yc + radius * sin(theta);            
		ref_states[i * NX + 2] = theta + M_PI / 2;                    
	}
	
	/*
	// set ref (line)
    double x_start = 0.0;
    double y_start = 0.0;
    double x_end = 3.0;
    double y_end = 3.0;

    for (int i = 0; i <= N; i++) {
        double alpha = (double)i / N;
        ref_states[i * NX + 0] = x_start + alpha * (x_end - x_start); 
        ref_states[i * NX + 1] = y_start + alpha * (y_end - y_start); 
        ref_states[i * NX + 2] = 0.78; 
    }
    */

    // set x0
    acados_x0[0] = ref_states[0];
    acados_x0[1] = ref_states[1];
    acados_x0[2] = ref_states[2];

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", acados_x0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", acados_x0);

    // yref and yrefN
    vector<double> yref(NY);
    vector<double> yref_N(NYN);

    
	// ref_v and ref_w
	std::vector<double> velocities(N);
	std::vector<double> angular_velocities(N);

	// set ref_v and ref_w
	for (int i = 0; i < N; i++) {
		// dx and dy
		double dx = ref_states[(i + 1) * NX + 0] - ref_states[i * NX + 0];
		double dy = ref_states[(i + 1) * NX + 1] - ref_states[i * NX + 1];
		// ref_v
		velocities[i] = sqrt(dx * dx + dy * dy) / Ts;

		// dtheta
		double theta_current = ref_states[i * NX + 2];
		double theta_next = ref_states[(i + 1) * NX + 2];
		double dtheta = theta_next - theta_current;
		while (dtheta > M_PI) dtheta -= 2 * M_PI;
		while (dtheta < -M_PI) dtheta += 2 * M_PI;
		//ref_w
		angular_velocities[i] = dtheta / Ts;
	}

    // set yref
	for (int i = 0; i < N; i++)
	{
		yref[0] = ref_states[i * NX + 0];       // ref_x
		yref[1] = ref_states[i * NX + 1];       // ref_y 
		yref[2] = ref_states[i * NX + 2];       // ref_theta
		yref[3] = velocities[i];                // ref_v
		yref[4] = angular_velocities[i];        // ref_w
		ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref.data());
	}

	// set yref_N
	yref_N[0] = ref_states[N * NX + 0];       // ref_x_N
	yref_N[1] = ref_states[N * NX + 1];       // ref_y_N
	yref_N[2] = ref_states[N * NX + 2];       // ref_theta_N
	ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", yref_N.data());

    // W and WN
    double Weights[NY * NY] = {0};
    double Weights_N[NYN * NYN] = {0};


	// set W and WN
	Weights[0 * NY + 0] = 1e2;  // w_x 
	Weights[1 * NY + 1] = 1e2;  // w_y 
	Weights[2 * NY + 2] = 1e-1; // w_theta
	Weights[3 * NY + 3] = 5e-2; // w_v
	Weights[4 * NY + 4] = 5e-2; // w_w

	Weights_N[0 * NYN + 0] = 1e2;  // w_x_N
	Weights_N[1 * NYN + 1] = 1e2;  // w_y_N
	Weights_N[2 * NYN + 2] = 1e-1; // w_theta_N

	for (int i = 0; i < N; i++) {
		ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "W", Weights);
	}
	ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "W", Weights_N);

    // solve ocp
    int solver_status = jackal_acados_solve(acados_ocp_capsule);
    if (solver_status != 0) {
        std::cerr << "jackal_acados_solve() failed with status " << solver_status << "." << std::endl;
    } else {
        std::cout << "jackal_acados_solve() succeeded." << std::endl;
    }

    // get solution time 
	double elapsed_time;
	ocp_nlp_get(nlp_config, nlp_solver, "time_tot", &elapsed_time);
	std::cout << "Total solution time: " << elapsed_time << " seconds." << std::endl;
	
	// get cost
	ocp_nlp_eval_cost(nlp_solver, nlp_in, nlp_out);
	double total_cost;
	ocp_nlp_get(nlp_config, nlp_solver, "cost_value", &total_cost);
	std::cout << "Total solution cost: " << total_cost << std::endl;

    // get solutions
    vector<vector<double>> control_output(N, vector<double>(NU));
    vector<vector<double>> state_output(N + 1, vector<double>(NX));

    for (int i = 0; i < N; i++)
    {
        double u_out[NU];
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i, "u", (void *)u_out);
        control_output[i] = vector<double>(u_out, u_out + NU);

        double x_out[NX];
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i, "x", (void *)x_out);
        state_output[i] = vector<double>(x_out, x_out + NX);
    }
    double x_out_n[NX];
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, N, "x", (void *)x_out_n);
    state_output[N] = vector<double>(x_out_n, x_out_n + NX);

    /******************************************************************************************************/

    /***************************************** PRINT RESULTS **********************************************/

	std::cout << std::left
		      << std::setw(10) << "Time"
		      << std::setw(15) << "Ref_X"
		      << std::setw(15) << "Ref_Y"
		      << std::setw(15) << "Sol_X"
		      << std::setw(15) << "Sol_Y" << std::endl;

	for (int i = 0; i <= N; i++) {
		double time = i * Ts;
		double ref_x = ref_states[i * NX + 0];
		double ref_y = ref_states[i * NX + 1];
		double sol_x = state_output[i][0];
		double sol_y = state_output[i][1];

		std::cout << std::left
		          << std::setw(10) << time
		          << std::setw(15) << ref_x
		          << std::setw(15) << ref_y
		          << std::setw(15) << sol_x
		          << std::setw(15) << sol_y << std::endl;
	}

    // write results in a CSV file
	std::ofstream outfile("results.csv");
	if (!outfile.is_open()) {
		std::cerr << "Failed to open file for writing." << std::endl;
		return 1;
	}
	outfile << "Time,Ref_X,Ref_Y,Sol_X,Sol_Y\n";
	for (int i = 0; i <= N; i++) {
		double time = i * Ts;
		double ref_x = ref_states[i * NX + 0];
		double ref_y = ref_states[i * NX + 1];
		double sol_x = state_output[i][0];
		double sol_y = state_output[i][1];
		outfile << time << "," << ref_x << "," << ref_y << "," << sol_x << "," << sol_y << "\n";
	}
	outfile.close();
	std::cout << "Results have been written to results.csv." << std::endl;
    
    /******************************************************************************************************/

    /***************************************** FREE_ACADOS_SOLVER *****************************************/
    int status_free = jackal_acados_free(acados_ocp_capsule);
    if (status_free) {
        std::cerr << "jackal_acados_free() returned status " << status_free << "." << std::endl;
    }
    status_free = jackal_acados_free_capsule(acados_ocp_capsule);
    if (status_free) {
        std::cerr << "jackal_acados_free_capsule() returned status " << status_free << "." << std::endl;
    }
    /******************************************************************************************************/

    return 0;
}

