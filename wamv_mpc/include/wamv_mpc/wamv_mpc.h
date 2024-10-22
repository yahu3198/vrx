#ifndef WAMV_MPC_H
#define WAMV_MPC_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <gazebo_msgs/ModelStates.h>

#include <iostream>
#include <fstream>
#include <cmath>
#include <tuple>
#include <iomanip>
#include <random>

// #include "acados/utils/print.h"
// #include "acados_c/ocp_nlp_interface.h"
// #include "acados_c/external_function_interface.h"
// #include "acados/ocp_nlp/ocp_nlp_constraints_bgh.h"
// #include "acados/ocp_nlp/ocp_nlp_cost_ls.h"

// #include "blasfeo/include/blasfeo_d_aux.h"
// #include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

// #include "bluerov2_model/bluerov2_model.h"
// #include "acados_solver_bluerov2.h"

using namespace Eigen;

class WAMV_MPC{
    private:

    struct Thrust{
        double left_thrust_angle;
        double left_thrust_cmd;
        double right_thrust_angle;
        double right_thrust_cmd;
    };
    struct LinearPos{
        double x;
        double y;
        double z;
    };

    struct LinearVel{
        double u;
        double v;
        double w;
    };

    struct Euler{
        double phi;
        double theta;
        double psi;
    };

    struct AngularVel{
        double p;
        double q;
        double r;
    };

    LinearPos local_pos;
    Euler local_euler;
    LinearVel linear_vel_inertial;
    LinearVel linear_vel_body;
    AngularVel angular_vel_inertial;
    AngularVel angular_vel_body;

    // Time
    ros::Time current_time;

    // ros subscriber & publisher
    ros::Subscriber states_sub;

    // Other variables
    tf::Quaternion tf_quaternion;
    int cout_counter = 0;

    public:

    bool is_start;

    WAMV_MPC(ros::NodeHandle&);                        // constructor
    Euler q2rpy(const geometry_msgs::Quaternion&);          // quaternion to euler angle
    geometry_msgs::Quaternion rpy2q(const Euler&);          // euler angle to quaternion
    void states_cb(const gazebo_msgs::ModelStates::ConstPtr& msg);  // subscribe pos and vel
    void solve();                                           // solve MPC
};

#endif