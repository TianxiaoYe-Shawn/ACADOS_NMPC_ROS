#ifndef MPC_TRACKING_H
#define MPC_TRACKING_H
#include "types_lib.h"

/* Some convenient definitions. */
#define NX          JACKAL_NX  /* Number of differential state variables.  */
#define NU          JACKAL_NU  /* Number of control inputs. */

#define NY0         JACKAL_NY0 /* Number of measurements/references on nodes 0. */
#define NY          JACKAL_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         JACKAL_NYN /* Number of measurements/references on node N. */

#define N           JACKAL_N   /* Number of intervals in the horizon. */

#define NBX0        JACKAL_NBX0/* Number of constraints on initial state X0. */
#define NBU         JACKAL_NBU /* Number of constraints on control inputs. */

#define Ts 0.1 // sampling time

class MPC_Tracking 
{
public:
    /* ACADO variables */
    ACADOvariables acadoVariables;
    ACADOworkspace acadoWorkspace;
    
    /* Common variables */
    double weight_x, weight_y, weight_theta, weight_v, weight_w;
    bool receive_traj = false;
    
    /* ROS variables */
    geometry_msgs::Pose odom_pose;
    geometry_msgs::Twist odom_twist;
    nav_msgs::Path path;

    /* ROS publishers and subscribers */
    ros::Subscriber state_sub;
    ros::Subscriber path_sub;
    ros::Publisher vel_pub;
    ros::Publisher predict_pub;
    
    /* Constructor */
    MPC_Tracking();
    
    /* ACADO Functions */
    std::vector<std::vector<double>> init_acado();
    void init_weight();
    std::vector<std::vector<double>> run_mpc_acado(std::vector<double> states, 
                                                   std::vector<double> ref_states, 
                                                   std::vector<std::vector<double>> previous_u);
    std::vector<double> calculate_ref_states(const std::vector<double> &ref_x,
                                             const std::vector<double> &ref_y,
                                             const std::vector<double> &ref_theta,
                                             const double &reference_v,
                                             const double &reference_w);
    std::vector<double> motion_prediction(const std::vector<double> &cur_states, const std::vector<std::vector<double>> &prev_u);
    std::vector<double> update_states(std::vector<double> state, double v_cmd, double w_cmd);
    
    /* ROS Functions */
    bool is_target(geometry_msgs::Pose cur, double goal_x, double goal_y);
    float quaternion2Yaw(geometry_msgs::Quaternion orientation);
    void pathCallback(const nav_msgs::Path& msg);
    void stateCallback(const gazebo_msgs::ModelStates& msg);
};

#endif
