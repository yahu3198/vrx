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
#include <sensor_msgs/msg/imu.hpp>

#include <iostream>
#include <fstream>
#include <cmath>
#include <tuple>
#include <iomanip>
#include <random>
#include <deque>    
#include <numeric>

#include <Eigen/Dense>
#include <fstream>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

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
    };

    enum ThrusterFaultType {
        NO_FAULT = 0,
        LEFT_THRUST_FAILURE = 1,
        RIGHT_THRUST_FAILURE = 2
    };

    // Thruster configuration analysis
    struct ThrusterConfiguration {
        enum MotionType {
            FORWARD,        // Both thrusters forward
            TURNING,        // Thrusters angled for turning
            LATERAL,        // Thrusters angled for lateral motion
            COMPLEX,        // Mixed or complex configuration
            UNKNOWN         // Undefined configuration
        };
        
        MotionType type;
        double expected_wpsi;    // Expected yaw disturbance for this configuration
        double turn_direction;   // +1 for right turn, -1 for left turn, 0 for straight
    };
    
    ThrusterConfiguration current_config_;

    struct OnlineLogisticRegression {
        // Model parameters
        MatrixXd weights;  // Weights for the logistic regression model
        double bias;       // Bias term
        double learning_rate;  // Learning rate for gradient descent
        double lambda;     // Regularization parameter
        
        // Data buffers for features and targets
        int buffer_size;   // Size of the sliding window
        std::deque<VectorXd> feature_buffer;  // Buffer for features
        std::deque<int> label_buffer;         // Buffer for labels (fault or no fault)
        
        // Feature extraction parameters
        int feature_dim;   // Dimension of feature vector
        double detect_threshold;  // Threshold for fault detection
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

    struct Acc{
        double x;
        double y;
        double z;
        double psi;
    };

    struct SolverParam{
        double Tp_pre;
        double Ts_pre;
    };
    struct ImuFilter{
        double p_smoothed, q_smoothed, r_smoothed;
        double phi_smoothed, theta_smoothed, psi_smoothed;
        double x_smoothed, y_smoothed, z_smoothed;
    }imu_filter;

    LocalPos local_pos;
    LocalPos imu_pos;
    LocalPos pre_ekf_pos;
    Acc imu_acc;
    Acc ekf_acc;

    // Acados variables
    SolverInput acados_in;
    SolverOutput acados_out;
    double acados_param[WAMV_N+1][WAMV_NP];  // disturbances
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

    double dt = 0.01;
    double mass = 180;
    double LCG = 2.373776;
    double B = 2.05427;
    double xu = -100;
    double xuu = -150;
    double yv = -100;
    double yvv = -100;
    double nr = -800;
    double nrr = -800;
    Matrix<double,1,3> M_values;
    Matrix<double,3,3> M;           // mass matrix
    Matrix<double,3,3> invM;        // inverse mass matrix

    std_msgs::msg::Float64 Tp;
    std_msgs::msg::Float64 Ts;
    std_msgs::msg::Float64 delta_p;
    std_msgs::msg::Float64 delta_s;

    geometry_msgs::msg::TwistStamped control_inputs;
    geometry_msgs::msg::TwistStamped disturbance;
    geometry_msgs::msg::TwistStamped confidence_level;
    nav_msgs::msg::Odometry ref_pose;
    nav_msgs::msg::Odometry error_pose;
    nav_msgs::msg::Odometry ekf_pose;

    // Time
    rclcpp::Time current_time;

    // ros subscriber & publisher
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr states_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_thrust_angle_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_thrust_cmd_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_thrust_angle_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_thrust_cmd_pub;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr control_inputs_pub;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr disturbance_pub;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr confidence_pub;
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ref_pose_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr error_pose_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ekf_pose_pub;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr fault_confidence_pub;

    // Trajectory variables
    std::vector<std::vector<double>> trajectory;
    int line_number = 0;
    int number_of_steps = 0;

    // EKF parameters
    // Matrix<double,6,1> wf_disturbance; // world frame disturbance 
    Matrix<double,2,1> meas_u;      // inputs
    int n = 9;                     // state dimension
    int m = 9;                     // measurement dimension
    Matrix<double,9,1> meas_y;     // measurement vector
    MatrixXd P0 = MatrixXd::Identity(m, m);     // initial covariance
    Matrix<double,9,1> esti_x;     // estimate states
    Matrix<double,9,9> esti_P;    // estimate covariance
    Matrix<double,1,9> Q_cov;      // process noise value
    Matrix<double,9,9> noise_Q;   // process noise matrix
    // MatrixXd noise_R = MatrixXd::Identity(m, m)*(pow(dt,4)/4); // measurement noise matrix
    Matrix<double,1,9> R_cov;
    Matrix<double,9,9> noise_R;
    Matrix<double,3,1> tau;
    bool imu_data_available = false, odom_data_available = false;
    Matrix<double, 4, 4> noise_R_imu;
    Matrix<double,1,4> R_imu_cov;

    // Other variables
    // tf2::Quaternion tf_quaternion;
    int cout_counter = 0;
    double start_time;

    float yaw_sum = 0;      // yaw degree as continous number
    float pre_yaw = 0;      // former state yaw degree
    float yaw_diff;         // yaw degree difference in every step
    float yaw_ref;          // yaw degree reference in form of (-pi, pi)
    float yaw_error;        // yaw degree error

    size_t iteration_count = 0;  // Add counter
    const size_t fault_trigger = 400;  // 20s at 20 Hz

    // Buffers for low-pass filtering
    const double alpha = 0.2; // Smoothing factor (0 < alpha < 1, lower = smoother)
    bool first_imu = true; // To initialize smoothed values

    // Parameters for feature extraction
    int window_size;                  // Size of sliding window for feature extraction
    std::deque<Vector3d> dist_buffer; // Buffer for disturbance values
    int detection_count_threshold;    // Number of consecutive detections needed to confirm fault
    int detection_counter;            // Counter for consecutive detections
    bool fault_detected;              // Flag indicating if a fault is currently detected
    int current_fault_type;           // Current detected fault type
    double fault_detection_confidence; // Confidence level of fault detection

    // Fault diagnosis model
    OnlineLogisticRegression fault_model;
    
    // Threshold values for fault detection
    double wx_threshold;
    double wy_threshold;
    double wpsi_threshold;

    // Previous thruster commands for comparison
    double prev_Tp, prev_Ts;
    
    // Publishers for fault diagnosis results
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fault_diagnosis_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr fault_features_pub;

    // Add these for disturbance model calibration
    std::vector<std::pair<double, double>> calibration_data;  // (thrust_diff, wpsi) pairs
    double wpsi_coefficient;                                  // Relationship between thrust difference and yaw disturbance
    int calibration_counter;                                  // Counter for calibration timing
    bool calibration_enabled;                                 // Flag to enable/disable calibration
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr wpsi_coefficient_pub; // Publisher for coefficient

    std::vector<double> fault_confidences; // Vector to store confidence values for each fault type
    bool warmup_completed = false;
    const size_t warmup_iterations = 180;
    std::deque<Vector2d> command_history; // For tracking thrust command history

    // Direct fault detection from simulation
    bool fault_simulation_active = false;
    int simulated_fault_type = NO_FAULT;
    
    // Improved fault tracking
    double fault_detection_time = 0.0;
    int consecutive_fault_detections = 0;
    int consecutive_normal_detections = 0;
    const int fault_confirmation_threshold = 3;  // Require this many consecutive detections
    const int normal_confirmation_threshold = 5;  // Require more confirmations to clear a fault

    // Fault detection state tracking
    int last_detected_fault_type = NO_FAULT;
    bool fault_state_active = false;
    
    // Turning point detection
    double prev_wx = 0.0;
    double prev_wy = 0.0;
    double prev_wpsi = 0.0;
    double prev_wx_trend = 0.1;
    double prev_wy_trend = 0.1;
    double prev_wpsi_trend = 0.1;
    
    // Constants for filtering
    const int FAULT_CONFIRMATION_COUNT = 2;  // Need this many consecutive detections
    const int NORMAL_CONFIRMATION_COUNT = 5; // Need more to clear a fault


    

    public:

    bool is_start;

    WAMV_MPC();                        // constructor
    void states_cb(const nav_msgs::msg::Odometry::SharedPtr msg);
    void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg);
    int readDataFromFile(const char* fileName, std::vector<std::vector<double>> &data);     // read trajectory
    void ref_cb(int line_to_read);
    void solve();                                           // solve MPC
    void publish_cin(double Tp_mpc, double Ts_mpc);
    void EKF();  
    MatrixXd RK4(MatrixXd x, MatrixXd u);                                           // EKF predict and update
    MatrixXd f(MatrixXd x, MatrixXd u);                     // system process model
    MatrixXd h(MatrixXd x);                                 // measurement model
    MatrixXd compute_jacobian_F(MatrixXd x, MatrixXd u);    // compute Jacobian of system process model
    MatrixXd compute_jacobian_H(MatrixXd x);                // compute Jacobian of measurement model
    MatrixXd h_imu(MatrixXd x);
    MatrixXd compute_jacobian_H_imu(MatrixXd x);
    void initializeFaultDiagnosis();
    void updateFaultModel();
    void extractFeatures(VectorXd& features);
    bool detectFault(const VectorXd& features, int& fault_type, std::vector<double>& fault_confidences);
    void logisticRegressionUpdate(const VectorXd& features, int label);
    Vector3d calculateDisturbanceStats(const std::deque<Vector3d>& buffer);
    void publishFaultDiagnosis(int fault_type, std::vector<double>& faault_confidences);
    void saveFaultModel(const std::string& filename);
    void loadFaultModel(const std::string& filename);
    void calibrateDisturbanceModel();
    double getCalibrated_wpsi() const;
    ThrusterConfiguration analyzeThrusterConfiguration(double tp, double ts, double delta_p, double delta_s);
    bool detectFaultForConfiguration(const VectorXd& features, int& fault_type, 
        std::vector<double>& fault_confidences,
        const ThrusterConfiguration& config);

};

#endif