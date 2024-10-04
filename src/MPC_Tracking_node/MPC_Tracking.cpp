#include "MPC_Tracking.h"

MPC_Tracking::MPC_Tracking()
{
    ros::NodeHandle nh("~");
    
    ros::Subscriber state_sub = nh.subscribe("/gazebo/model_states", 10, &MPC_Tracking::stateCallback, this);
    ros::Subscriber path_sub = nh.subscribe("/trajectory", 1, &MPC_Tracking::pathCallback, this);
    
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Publisher predict_pub = nh.advertise<nav_msgs::Path>("/predict_path", 10);

    /* ROS PARAM */
    ros::param::get("~weight_x", weight_x);
    ros::param::get("~weight_y", weight_y);
    ros::param::get("~weight_theta", weight_theta);
    ros::param::get("~weight_v", weight_v);
    ros::param::get("~weight_w", weight_w);
}

std::vector<std::vector<double>> MPC_Tracking::init_acado()
{
    /* Initialize the solver. */
    acado_initializeSolver();

    /* Initialize the states and controls. */
    for (int i = 0; i < NX * (N + 1); ++i) acadoVariables.x[i] = 0.0;
    for (int i = 0; i < NU * N; ++i) acadoVariables.u[i] = 0.0;

    /* Initialize the measurements/reference. */
    for (int i = 0; i < NY * N; ++i) acadoVariables.y[i] = 0.0;
    for (int i = 0; i < NYN; ++i) acadoVariables.yN[i] = 0.0;

    acado_preparationStep();
    init_weight();
    
    std::vector<double> control_output_v, control_output_w;
    for (int i = 0; i < ACADO_N; ++i) 
    {
        control_output_v.push_back(acadoVariables.u[i * ACADO_NU]);
        control_output_w.push_back(acadoVariables.u[i * ACADO_NU + 1]);
    }
    
    return {control_output_v, control_output_w};
}

void MPC_Tracking::init_weight()
{
    for (int i = 0; i < N; ++i) 
    {
        acadoVariables.W[NY * NY * i + (NY + 1) * 0] = weight_x;
        acadoVariables.W[NY * NY * i + (NY + 1) * 1] = weight_y;
        acadoVariables.W[NY * NY * i + (NY + 1) * 2] = weight_theta;
        acadoVariables.W[NY * NY * i + (NY + 1) * 3] = weight_v;
        acadoVariables.W[NY * NY * i + (NY + 1) * 4] = weight_w;
    }
    acadoVariables.WN[(NYN + 1) * 0] = weight_x;
    acadoVariables.WN[(NYN + 1) * 1] = weight_y;
    acadoVariables.WN[(NYN + 1) * 2] = weight_theta;
}

std::vector<std::vector<double>> MPC_Tracking::run_mpc_acado(std::vector<double> states, std::vector<double> ref_states, std::vector<std::vector<double>> previous_u)
{
    for (int i = 0; i < NX * (N + 1); ++i)  
        acadoVariables.x[i] = (real_t)states[i];

    for (int i = 0; i < NX; ++i)
        acadoVariables.x0[i] = (real_t)states[i];

    for (int i = 0; i < NY * N; ++i)
        acadoVariables.y[i] = (real_t)ref_states[i];

    for (int i = 0; i < NYN; ++i)
        acadoVariables.yN[i] = (real_t)ref_states[NY * (N - 1) + i];

    acado_preparationStep();

    for (int i = 0; i < NUM_STEPS; ++i)
        acado_feedbackStep();

    std::vector<double> control_output_v, control_output_w;
    real_t *u = acado_getVariablesU();
    for (int i = 0; i < ACADO_N; ++i)
    {
        control_output_v.push_back((double)u[i * ACADO_NU]);
        control_output_w.push_back((double)u[i * ACADO_NU + 1]);
    }

    return {control_output_v, control_output_w};
}

std::vector<double> MPC_Tracking::calculate_ref_states(const std::vector<double> &ref_x, const std::vector<double> &ref_y, const std::vector<double> &ref_theta,
                                                       const double &reference_v, const double &reference_w)
{
    std::vector<double> result;
    for (int i = 0; i < N; i++) 
    {
        result.push_back(ref_x[i]);
        result.push_back(ref_y[i]);
        result.push_back(ref_theta[i]);
        result.push_back(reference_v);
        result.push_back(reference_w);
    }
    return result;
}

std::vector<double> MPC_Tracking::motion_prediction(const std::vector<double> &cur_states, const std::vector<std::vector<double>> &prev_u)
{
    std::vector<double> old_v_cmd = prev_u[0];
    std::vector<double> old_w_cmd = prev_u[1];
    std::vector<std::vector<double>> predicted_states;
    predicted_states.push_back(cur_states);

    for (int i = 0; i < N; i++) 
    {
        std::vector<double> cur_state = predicted_states[i];
        if (cur_state[3] > M_PI) cur_state[3] -= 2 * M_PI;
        if (cur_state[3] < -M_PI) cur_state[3] += 2 * M_PI;
        std::vector<double> next_state = update_states(cur_state, old_v_cmd[i], old_w_cmd[i]);
        predicted_states.push_back(next_state);
    }

    std::vector<double> result;
    for (int i = 0; i < (ACADO_N + 1); ++i) 
    {
        for (int j = 0; j < NX; ++j)
            result.push_back(predicted_states[i][j]);
    }
    return result;
}

std::vector<double> MPC_Tracking::update_states(std::vector<double> state, double v_cmd, double w_cmd)
{
    double x0 = state[0];
    double y0 = state[1];
    double q0 = state[2];
    double x1 = x0 + (v_cmd * cos(q0)) * Ts;
    double y1 = y0 + (v_cmd * sin(q0)) * Ts;
    double q1 = q0 + w_cmd * Ts;
    return {x1, y1, q1};
}

bool MPC_Tracking::is_target(geometry_msgs::Pose cur, double goal_x, double goal_y)
{
    return (abs(cur.position.x - goal_x) < 0.05 && abs(cur.position.y - goal_y) < 0.05);
}

float MPC_Tracking::quaternion2Yaw(geometry_msgs::Quaternion orientation)
{
    double q0 = orientation.x;
    double q1 = orientation.y;
    double q2 = orientation.z;
    double q3 = orientation.w;

    return atan2(2.0 * (q2 * q3 + q0 * q1), 1.0 - 2.0 * (q1 * q1 + q2 * q2));
}

void MPC_Tracking::pathCallback(const nav_msgs::Path& msg)
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

void MPC_Tracking::stateCallback(const gazebo_msgs::ModelStates& msg)
{
    for (size_t i = 0; i < msg.name.size(); ++i) 
    {
        if (msg.name[i] == "jackal") 
        {
            odom_pose = msg.pose[i];
            odom_twist = msg.twist[i];
            break;
        }
    }
}
