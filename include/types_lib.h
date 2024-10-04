#ifndef DATA_LIB_TYPES
#define DATA_LIB_TYPES

// ROS
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h> 
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "gazebo_msgs/ModelStates.h"

// C++
#include <iostream>
#include <vector>
#include <stdio.h>
#include <std_msgs/String.h>
#include <string.h>
#include <cmath>
#include <math.h>
#include <chrono>
#include <vector>
#include <fstream>
#include <stdlib.h>
#include <algorithm>
#include <bits/stdc++.h>

// ACADOS
#include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados/ocp_nlp/ocp_nlp_constraints_bgh.h"
#include "acados/ocp_nlp/ocp_nlp_cost_ls.h"

// blasfeo
#include "blasfeo/include/blasfeo_d_aux.h"
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

// Generated C code
#include "jackal_model/jackal_model.h"
#include "acados_solver_jackal.h"

using namespace std;
using std::vector;
using namespace std::chrono;

struct OptTraj
{
    bool   is_solved;
    double x[JACKAL_N+1];
    double y[JACKAL_N+1];
    double theta[JACKAL_N+1];
    double time_solving;
    double cost;
    double v[JACKAL_N] , w[JACKAL_N];
};

struct X0
{
    double x;
    double y;
    double theta;
};

struct Yref
{
    double x[JACKAL_N];
    double y[JACKAL_N];
    double theta[JACKAL_N];
    double v[JACKAL_N] , w[JACKAL_N];
};

struct Yref_e
{
    double x;
    double y;
    double theta;
};

#endif

