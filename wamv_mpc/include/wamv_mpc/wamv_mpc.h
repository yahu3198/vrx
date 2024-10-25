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

    enum SystemStates{
        x = 0,
        y = 1,
        psi = 2,
        u = 3,
        v = 4,
        r = 5,
    };

    enum ControlInputs{
        u1 = 0,
        u2 = 1,
        u3 = 2,
        u4 = 3,
    };

    struct SolverInput{
        double x0[WAMV_NX];
        double yref[WAMV_N+1][WAMV_NY];
    };

    struct SolverOutput{
        double u0[WAMV_NU];
        double x1[WAMV_NX];
        double status, kkt_res, cpu_time;
    };

    // struct Thrust{
    //     double left_thrust_angle;
    //     double left_thrust_cmd;
    //     double right_thrust_angle;
    //     double right_thrust_cmd;
    // };
    struct LinearPos{
        double x;
        double y;
        double z;
        double u;
        double v;
        double w;
        double p;
        double q;
        double r;
    };

    // struct LinearVel{
    //     double u;
    //     double v;
    //     double w;
    // };

    struct Euler{
        double phi;
        double theta;
        double psi;
    };

    // struct AngularVel{
    //     double p;
    //     double q;
    //     double r;
    // };

    LinearPos local_pos;
    Euler local_euler;
    // LinearVel linear_vel_inertial;
    // LinearVel linear_vel_body;
    // AngularVel angular_vel_inertial;
    // AngularVel angular_vel_body;

    // ROS message variables
    std_msgs::Float32 left_thrust_angle;
    std_msgs::Float32 left_thrust_cmd;
    std_msgs::Float32 right_thrust_angle;
    std_msgs::Float32 right_thrust_cmd;
    gazebo_msgs::ModelStates ref_states;
    gazebo_msgs::ModelStates error_states;

    // Acados variables
    SolverInput acados_in;
    SolverOutput acados_out;
    // double acados_param[WAMV_N+1][WAMV_NP];  // disturbances
    int acados_status;   
    wamv_solver_capsule * mpc_capsule = wamv_acados_create_capsule();
    
    std::string REF_TRAJ;
    std::string WRENCH_FX;
    std::string WRENCH_FY;
    std::string WRENCH_FZ;
    std::string WRENCH_TZ;
    int READ_WRENCH;        // 0: periodic disturbance; 1: random disturbance; 2: read wrench from text
    bool COMPENSATE_D;       // 0: no compensate; 1: compensate
    SolverParam solver_param;

    // dynamics parameters
    Matrix<double,3,3> R_ib;            // rotation matrix for linear from inertial to body frame
    Matrix<double,3,1> v_body;      // velocity u, v, r in body frame
    Matrix<double,3,1> v_inertial;  // velocity u, v, r in inertial frame

    // Time
    ros::Time current_time;

    // ros subscriber & publisher
    ros::Subscriber states_sub;
    
    ros::Publisher left_thrust_angle_pub;
    ros::Publisher left_thrust_cmd_pub;
    ros::Publisher right_thrust_angle_pub;
    ros::Publisher right_thrust_cmd_pub;
    
    ros::Publisher ref_states_pub;
    ros::Publisher error_states_pub;

    // Trajectory variables
    std::vector<std::vector<double>> trajectory;
    int line_number = 0;
    int number_of_steps = 0;

    // Other variables
    tf::Quaternion tf_quaternion;
    int cout_counter = 0;

    float yaw_sum = 0;      // yaw degree as continous number
    float pre_yaw = 0;      // former state yaw degree
    float yaw_diff;         // yaw degree difference in every step
    float yaw_ref;          // yaw degree reference in form of (-pi, pi)
    float yaw_error;        // yaw degree error

    public:

    bool is_start;

    WAMV_MPC(ros::NodeHandle&);                        // constructor
    Euler q2rpy(const geometry_msgs::Quaternion&);          // quaternion to euler angle
    geometry_msgs::Quaternion rpy2q(const Euler&);          // euler angle to quaternion
    void states_cb(const gazebo_msgs::ModelStates::ConstPtr& msg);  // subscribe pos and vel
    int readDataFromFile(const char* fileName, std::vector<std::vector<double>> &data);     // read trajectory
    void ref_cb(int line_to_read);
    void solve();                                           // solve MPC
};

#endif