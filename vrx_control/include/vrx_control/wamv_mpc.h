#ifndef WAMV_MPC_H
#define WAMV_MPC_H

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <eigen3/Eigen/Dense>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <std_msgs/msg/float64.hpp>

#include <iostream>
#include <fstream>
#include <cmath>
#include <tuple>
#include <iomanip>
#include <random>

#include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados/ocp_nlp/ocp_nlp_constraints_bgh.h"
#include "acados/ocp_nlp/ocp_nlp_cost_ls.h"

#include "blasfeo/include/blasfeo_d_aux.h"
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

#include "wamv_model/wamv_model.h"
#include "acados_solver_wamv.h"


using namespace Eigen;

class WAMV_MPC : public rclcpp::Node
{
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

    struct LocalPos{
        double x;
        double y;
        double z;
        double u;
        double v;
        double w;
        double phi;
        double theta;
        double psi;
        double p;
        double q;
        double r;
    };

    LocalPos local_pos;

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
    // SolverParam solver_param;

    // dynamics parameters
    Matrix<double,3,3> R_ib;            // rotation matrix for linear from inertial to body frame
    Matrix<double,3,1> v_body;      // velocity u, v, r in body frame
    Matrix<double,3,1> v_inertial;  // velocity u, v, r in inertial frame

    std_msgs::msg::Float64 Tp;
    std_msgs::msg::Float64 Ts;
    std_msgs::msg::Float64 delta_p;
    std_msgs::msg::Float64 delta_s;

    geometry_msgs::msg::TwistStamped control_inputs;
    nav_msgs::msg::Odometry ref_pose;
    nav_msgs::msg::Odometry error_pose;

    // Time
    rclcpp::Time current_time;

    // ros subscriber & publisher
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr states_sub;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_thrust_angle_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_thrust_cmd_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_thrust_angle_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_thrust_cmd_pub;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr control_inputs_pub;
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ref_pose_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr error_pose_pub;

    // Trajectory variables
    std::vector<std::vector<double>> trajectory;
    int line_number = 0;
    int number_of_steps = 0;

    // Other variables
    tf2::Quaternion tf_quaternion;
    int cout_counter = 0;

    float yaw_sum = 0;      // yaw degree as continous number
    float pre_yaw = 0;      // former state yaw degree
    float yaw_diff;         // yaw degree difference in every step
    float yaw_ref;          // yaw degree reference in form of (-pi, pi)
    float yaw_error;        // yaw degree error

    public:

    bool is_start;

    WAMV_MPC();                        // constructor
    // void states_cb(const gazebo_msgs::ModelStates::ConstPtr& msg);  // subscribe pos and vel
    void states_cb(const nav_msgs::msg::Odometry::SharedPtr msg);
    int readDataFromFile(const char* fileName, std::vector<std::vector<double>> &data);     // read trajectory
    void ref_cb(int line_to_read);
    void solve();                                           // solve MPC
    void publish_cin(double Tp_mpc, double Ts_mpc, double delta_p_mpc, double delta_s_mpc);
};

#endif