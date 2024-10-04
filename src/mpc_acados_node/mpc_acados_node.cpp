#include "types_lib.h"

using namespace std;

/* Some convenient definitions. */
#define NX          JACKAL_NX  /* Number of differential state variables.  */
#define NU          JACKAL_NU  /* Number of control inputs. */

#define NP          JACKAL_NP  /* Number of parameters in cost functions. */

#define NY          JACKAL_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         JACKAL_NYN /* Number of measurements/references on node N. */

#define N           JACKAL_N   /* Number of intervals in the horizon. */

#define NUM_STEPS   10        /* Number of real-time iterations. */
#define VERBOSE     1         /* Show iterations: 1, silent: 0.  */

#define TF 2                  /* (s) predict horizon */     
#define Ts (TF / (double)N)   /* (s) sampling time. */

/* Function declaration. */
void stateCallback(const gazebo_msgs::ModelStates& msg);
void pathCallback(const nav_msgs::Path& msg);
float quaternion2Yaw(geometry_msgs::Quaternion orientation);
bool is_target(geometry_msgs::Pose cur, double goal_x, double goal_y);
vector<double> calculate_ref_states(const vector<double> &ref_x,
									const vector<double> &ref_y,
									const vector<double> &ref_q,
									const vector<double> &ref_v,
									const vector<double> &ref_w);
vector<double> motion_prediction(const vector<double> &cur_states,
                                const vector<vector<double>> &prev_u);
vector<double> update_states(vector<double> state, double v_cmd, double w_cmd);


/* Function definition. */

double weight_x, weight_y, weight_q, weight_v, weight_w;

geometry_msgs::Pose odom_pose;
geometry_msgs::Twist odom_twist;
void stateCallback(const gazebo_msgs::ModelStates& msg) {
    for (size_t i = 0; i < msg.name.size(); ++i) {
        if (msg.name[i] == "jackal") {
            odom_pose = msg.pose[i];
            odom_twist = msg.twist[i];
            break;
        }
    }
}

bool receive_traj = false;
nav_msgs::Path path;
void pathCallback(const nav_msgs::Path& msg) 
{ 
	if (!receive_traj) 
	{ 
		path = msg; 
		receive_traj = true; 
		ROS_INFO("Received new trajectory with %d poses.", int(msg.poses.size()));
	} 
	else 
	{
        ROS_INFO("New trajectory received but already have one.");
    }
}

bool is_target(geometry_msgs::Pose cur, double goal_x, double goal_y)
{
	if(abs(cur.position.x - goal_x) < 0.05 && abs(cur.position.y - goal_y) < 0.05)
	{
		return true;
	}
	else return false;
}

float quaternion2Yaw(geometry_msgs::Quaternion orientation)
{
    double q0 = orientation.x;
    double q1 = orientation.y;
    double q2 = orientation.z;
    double q3 = orientation.w;

    float yaw = atan2(2.0*(q2*q3 + q0*q1), 1.0 - 2.0*(q1*q1 + q2*q2));
    return yaw;
}

vector<double> calculate_ref_states(const vector<double> &ref_x,
									const vector<double> &ref_y,
									const vector<double> &ref_q,
                                    const vector<double> &ref_v,
                                    const vector<double> &ref_w)
{
	vector<double> result;
	for (int i = 0; i <= N; i++)
	{
		result.push_back(ref_x[i]);
		result.push_back(ref_y[i]);
		result.push_back(ref_q[i]);
        result.push_back(ref_v[i]);
        result.push_back(ref_w[i]);
	}
	return result;
}

vector<double> update_states(vector<double> state, double v_cmd, double w_cmd)
{
	// based on kinematic model
	double x0 = state[0];
	double y0 = state[1];
	double q0 = state[2];
	double v0 = v_cmd;
	double w0 = w_cmd;

	double x1 = x0 + (v0 * cos(q0))* Ts;
	double y1 = y0 + (v0 * sin(q0))* Ts;
	double q1 = q0 + w0 * Ts;
	return {x1, y1, q1};
}

vector<double> motion_prediction(const vector<double> &cur_states,
                                const vector<vector<double>> &prev_u)
{
	vector<double> old_v_cmd(N);
	vector<double> old_w_cmd(N);

	for (int i = 0; i < N; ++i) {
		old_v_cmd[i] = prev_u[i][0];  
		old_w_cmd[i] = prev_u[i][1];
	}

	vector<vector<double>> predicted_states;
	predicted_states.push_back(cur_states);

	for (int i = 0; i < N; i++)
	{
		vector<double> cur_state = predicted_states[i];
		// yaw angle compensation of overflow
		if (cur_state[2] > M_PI)
		{
			cur_state[2] -= 2 * M_PI;
		}
		if (cur_state[2] < -M_PI)
		{
			cur_state[2] += 2 * M_PI;
		}
		vector<double> next_state = update_states(cur_state, old_v_cmd[i], old_w_cmd[i]);
		predicted_states.push_back(next_state);
	}

	vector<double> result;
	for (int i = 0; i < (N + 1); ++i)
	{
		for (int j = 0; j < NX; ++j)
		{
			result.push_back(predicted_states[i][j]);
		}
	}
	return result;
}


/* Main Function. */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpc_acados_node");
    ros::NodeHandle nh("~");

    ros::Subscriber state_sub = nh.subscribe("/gazebo/model_states", 10, stateCallback);
    ros::Subscriber path_sub = nh.subscribe("/trajectory", 1, pathCallback);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Publisher predict_pub = nh.advertise<nav_msgs::Path>("/predict_path", 10);
    ros::Publisher ref_pub = nh.advertise<nav_msgs::Path>("/reference_path", 10);


  	/* ROS PARAM */
  	ros::param::get("~weight_x", weight_x);
  	ros::param::get("~weight_y", weight_y);
  	ros::param::get("~weight_q", weight_q);
  	ros::param::get("~weight_v", weight_v);
  	ros::param::get("~weight_w", weight_w);

  	cout << weight_x << " " << weight_y << " " << weight_q << " " << weight_v << " " << weight_w << " " << endl;	
    
    ros::Rate r(10);
	double goal_x, goal_y;
	int count = 0;

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



/******************************************************************************************************/
/********************************************* Main Loop **********************************************/
    while(ros::ok())
    {
		if (path.poses.size() == 0)
		  {
		  	ros::spinOnce();
		  	r.sleep();
		  	continue;
		  }

		goal_x = path.poses[path.poses.size()-1].pose.position.x;
		goal_y = path.poses[path.poses.size()-1].pose.position.y;

		// x0
		double px = odom_pose.position.x; // state[0];
		double py = odom_pose.position.y; // state[1];

		double q0 = odom_pose.orientation.x;
		double q1 = odom_pose.orientation.y;
		double q2 = odom_pose.orientation.z;
		double q3 = odom_pose.orientation.w;
		double t0 = -2.0 * (q1*q1 + q2*q2) + 1.0;
		double t1 = +2.0 * (q2*q3 + q0*q1);

		double pq = atan2(t1, t0); // state[2]; 

		vector<double> cur_state = {px, py, pq};
		double acados_x0[NX];
		acados_x0[0] = px;
		acados_x0[1] = py;
		acados_x0[2] = pq;

		// set_x0 
		ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", acados_x0);
		ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", acados_x0);
		
		// ref
		vector<double> ref_states((N + 1) * NX);
		vector<double> velocities(N);
		vector<double> angular_velocities(N);

        // x_ref
		for (int i = 0; i <= N; i++)
		{
			if (count + i >= path.poses.size())
			{
				ref_states[i * NX + 0] = path.poses[path.poses.size()-1].pose.position.x;
				ref_states[i * NX + 1] = path.poses[path.poses.size()-1].pose.position.y;
				ref_states[i * NX + 2] = quaternion2Yaw(path.poses[path.poses.size()-1].pose.orientation);
			}
			else
			{
				ref_states[i * NX + 0] = path.poses[count+i].pose.position.x;
				ref_states[i * NX + 1] = path.poses[count+i].pose.position.y;
				ref_states[i * NX + 2] = quaternion2Yaw(path.poses[count+i].pose.orientation);
			}
		}

		// publish x_ref
		nav_msgs::Path ref_path;
		ref_path.header.frame_id = "/map";
		ref_path.header.stamp = ros::Time::now();
		for (int i = 0; i <= N; i++)
		{
			geometry_msgs::PoseStamped ref_pose;
			ref_pose.header = ref_path.header;
			ref_pose.pose.position.x = ref_states[i * NX + 0];
			ref_pose.pose.position.y = ref_states[i * NX + 1];
			ref_pose.pose.orientation = tf::createQuaternionMsgFromYaw(ref_states[i * NX + 2]);
			ref_path.poses.push_back(ref_pose);
		}
		ref_pub.publish(ref_path);

		// u_ref
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

		// yref and yrefN
		vector<double> yref(NY);
		vector<double> yref_N(NYN);

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
		Weights[0 * NY + 0] = weight_x;  // w_x 
		Weights[1 * NY + 1] = weight_y;  // w_y 
		Weights[2 * NY + 2] = weight_q; // w_theta
		Weights[3 * NY + 3] = weight_v; // w_v
		Weights[4 * NY + 4] = weight_w; // w_w

		Weights_N[0 * NYN + 0] = weight_x;  // w_x_N
		Weights_N[1 * NYN + 1] = weight_y;  // w_y_N
		Weights_N[2 * NYN + 2] = weight_q; // w_theta_N
		

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

		// publish x_out
		nav_msgs::Path predict_path;
		predict_path.header.frame_id = "/map";
		predict_path.header.stamp = ros::Time::now();
		for (int i = 0; i <= N; i++)
		{
			geometry_msgs::PoseStamped pred_pose;
			pred_pose.header = predict_path.header;
			pred_pose.pose.position.x = state_output[i][0];
			pred_pose.pose.position.y = state_output[i][1];
			pred_pose.pose.orientation.w = state_output[i][2];;
			predict_path.poses.push_back(pred_pose);
		}
		predict_pub.publish(predict_path);

       // publish u_out
		geometry_msgs::Twist vel;
		bool goal = is_target(odom_pose, goal_x, goal_y) && count >= path.poses.size();
		if(goal)
		{
			vel.linear.x = 0;
			vel.angular.z = 0;
		}
		else
		{
			vel.linear.x = control_output[0][0];
			vel.angular.z = control_output[0][1];
		}
		vel_pub.publish(vel);

		ROS_INFO("Control output: linear = %f, angular = %f", control_output[0][0], control_output[0][1]);

        ros::spinOnce();
        r.sleep();
		count++;
    }
/********************************************* Loop  end **********************************************/
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



