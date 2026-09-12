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
#include <algorithm>   // [manifold] std::max/std::min
#include <cstdint>     // [manifold] SIZE_MAX

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
    double nr = -300;
    double nrr = -300;
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
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr disturbance_world_pub;  
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr confidence_pub;
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ref_pose_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr error_pose_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ekf_pose_pub;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr fault_confidence_pub;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr operational_mode_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr thruster_health_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr usv_state_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr environmental_assistance_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr planning_status_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr prediction_metrics_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr learned_features_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mission_metrics_pub;

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
    // [manifold] fault_trigger is now a ROS parameter ("fault_trigger_iters",
    // in solve() iterations at 20 Hz; 300 = 15 s, the ICRA setting).
    size_t fault_trigger = 300;

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

    // [manifold] severity and fault type are ROS parameters
    // ("thruster_degrade_percentage", "fault_type_sim") so trial campaigns do
    // not need a recompile per cell.
    float thruster_degrade_percentage = 0.5;
    enum FaultSimulationType {
        NO_FAULT_SIM = 0,
        LEFT_THRUSTER_FAULT_SIM = 1,
        RIGHT_THRUSTER_FAULT_SIM = 2
    };

    bool ENABLE_ENV_ASSIST = true;
    
    int FAULT_TYPE_TO_SIMULATE = LEFT_THRUSTER_FAULT_SIM;
    
    // Constants for filtering
    const int FAULT_CONFIRMATION_COUNT = 2;  // Need this many consecutive detections
    const int NORMAL_CONFIRMATION_COUNT = 5; // Need more to clear a 
    

    struct PlanningResult {
        int selected_harbor_zone;        // 0, 1, or 2 (-1 if none feasible)
        Vector2d target_point;           // Specific entry point in selected zone
        double path_distance;            // Direct distance to target
        double required_heading_change;  // Heading change needed (radians)
        double environmental_alignment;  // -1 to 1, how well env forces help
        bool obstacle_free;              // True if path avoids dock areas
        double feasibility_score;        // Overall feasibility (higher = better)
        bool is_valid;                   // True if any feasible path found
    };
    
    struct HarborZone {
        std::vector<Vector2d> vertices;  // Zone boundary points
        Vector2d center;                 // Zone center point
        double area;                     // Zone area (for reference)
    };

    enum OperationalMode {
        FOLLOW_PRESET_TRAJECTORY = 0,    // Follow pre-read .txt file
        STATION_KEEPING = 1,             // Stationary after fault
        ADAPTIVE_ASSISTED_RETURN = 2,    // Fast planned return to port
        MANIFOLD_RETURN = 3              // [manifold] track an externally supplied recovery reference
    };

    // ---- [manifold] external recovery reference -----------------------------
    // ref_source: "internal" (default, ICRA behaviour) or "manifold".
    // In "manifold" mode, after the fault trigger the node holds the preset
    // course until a reference arrives on /wamv/manifold_ref, then tracks it.
    // If none arrives within manifold_timeout_s it falls back to the internal
    // planner and records the fallback in /wamv/manifold_status.
    //
    // /wamv/manifold_ref (std_msgs/Float64MultiArray) layout:
    //   data[0] = t0, the ROS time (seconds) at which row 0 applies
    //   data[1] = dt, row spacing in seconds (0.05 expected)
    //   data[2:] = rows of 8: [x, y, psi_bounded, u, v, r, Tp, Ts]
    // psi is converted to the node's continuous yaw on receipt. Rows already
    // elapsed at receipt (sidecar latency) are skipped, not replayed.
    std::string REF_SOURCE = "internal";
    double manifold_timeout_s = 20.0;
    // Position-based fault trigger (trial campaign): when enabled, the fault
    // fires at the first solve() with local_pos.x <= fault_trigger_x, so the
    // recovery always starts at the canonical start regardless of node start
    // time or ramp-up. Implemented by re-arming fault_trigger = iteration+1 so
    // every existing "iteration_count >= fault_trigger" check is untouched.
    bool use_position_trigger = false;
    double fault_trigger_x = -459.5;
    bool position_trigger_fired = false;
    bool arrival_strict = false;          // true: inside a zone polygon only (no 15 m fallback)
    bool manifold_ref_received = false;
    bool manifold_fallback = false;
    double manifold_ref_latency_s = -1.0; // trigger -> first reference, seconds
    double manifold_ref_t0 = 0.0;
    double manifold_ref_dt = 0.05;
    std::vector<std::vector<double>> manifold_trajectory;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr manifold_ref_sub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr manifold_status_pub;
    void manifold_ref_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void publishManifoldStatus();
    // -------------------------------------------------------------------------

    std::vector<HarborZone> harbor_zones;
    std::vector<std::vector<Vector2d>> dock_areas;
    PlanningResult current_plan;
    double last_replan_time;
    static constexpr double MIN_REPLAN_INTERVAL = 2.0;  // seconds

    OperationalMode current_mode;
    std::vector<std::vector<double>> generated_trajectory;  // For on-demand trajectory
    int generated_line_number;                              // Current position in generated trajectory
    bool trajectory_generation_active;                      // Flag for trajectory generation
    bool mission_completed;
    double arrival_time;
    static constexpr double ARRIVAL_DISTANCE_THRESHOLD = 15.0;  // meters
    static constexpr double ARRIVAL_CONFIRMATION_TIME = 2.0;   // seconds

    // Environmental assistance structure
    struct EnvironmentalAssistance {
        Vector3d current_forces;          // [w_x, w_y, w_psi] from EKF
        Vector3d predicted_forces;        // For future enhancement
        double assistance_capability;     // 0.0 to 1.0 - how much we can rely on env forces
        bool is_reliable;                // Environmental force quality flag
        double surge_assistance_factor;   // α for w_x utilization (0.0 to 1.0)
        double sway_assistance_factor;    // α for w_y utilization (0.0 to 1.0)
        double yaw_assistance_factor;     // α for w_psi utilization (0.0 to 1.0)
    };

    // Fault-adaptive MPC configuration
    struct AdaptiveMPCWeights {
        bool use_environmental_assistance;
        double environmental_weight_factor;    // Boost/reduce env assistance reliance
        double thruster_penalty_factor;       // Increase penalty for failed thruster
        double fault_compensation_gain;       // How aggressively to compensate with env forces
    };

    EnvironmentalAssistance environmental_assistance;
    AdaptiveMPCWeights adaptive_weights;

    // Environmental prediction variables
    Vector3d prev_env_forces;
    bool prev_forces_initialized;
    double prediction_dt;
    
    // Validation structure - shorter horizons for better relevance
    struct PredictionValidation {
        std::deque<Vector3d> predicted_0_5s_body;   
        std::deque<Vector3d> predicted_1s_body;     
        std::deque<Vector3d> predicted_2s_body;     
        std::deque<Vector3d> actual_forces_body;    
        std::deque<double> prediction_headings;     
        std::deque<double> actual_headings;         
        std::deque<double> timestamps;              
        
        // Component-wise RMSE metrics
        double rmse_0_5s_wx, rmse_0_5s_wy, rmse_0_5s_wpsi;
        double rmse_1s_wx, rmse_1s_wy, rmse_1s_wpsi;
        double rmse_2s_wx, rmse_2s_wy, rmse_2s_wpsi;
        
        // Overall RMSE for comparison
        double rmse_0_5s_overall, rmse_1s_overall, rmse_2s_overall;
        
        // Component-wise MAE metrics
        double mae_0_5s_wx, mae_0_5s_wy, mae_0_5s_wpsi;
        double mae_1s_wx, mae_1s_wy, mae_1s_wpsi;
        double mae_2s_wx, mae_2s_wy, mae_2s_wpsi;
        
        int validation_samples;
        bool validation_ready;
        
        PredictionValidation() : 
            rmse_0_5s_wx(0), rmse_0_5s_wy(0), rmse_0_5s_wpsi(0),
            rmse_1s_wx(0), rmse_1s_wy(0), rmse_1s_wpsi(0),
            rmse_2s_wx(0), rmse_2s_wy(0), rmse_2s_wpsi(0),
            rmse_0_5s_overall(0), rmse_1s_overall(0), rmse_2s_overall(0),
            mae_0_5s_wx(0), mae_0_5s_wy(0), mae_0_5s_wpsi(0),
            mae_1s_wx(0), mae_1s_wy(0), mae_1s_wpsi(0),
            mae_2s_wx(0), mae_2s_wy(0), mae_2s_wpsi(0),
            validation_samples(0), validation_ready(false) {}
    };

    struct DataDrivenEnvironmentalPredictor {
        // Feature extraction from force history
        struct Features {
            double mean_magnitude;      // Average force magnitude
            double dominant_frequency;  // From FFT analysis
            double variance;            // Force variability
            double trend_x;            // Recent trend in x
            double trend_y;            // Recent trend in y
            double trend_yaw;          // Recent trend in yaw
            double phase_estimate;     // Estimated position in oscillation
            
            Eigen::VectorXd toVector() const {
                Eigen::VectorXd vec(7);
                vec << mean_magnitude, dominant_frequency, variance, 
                       trend_x, trend_y, trend_yaw, phase_estimate;
                return vec;
            }
        };
        
        // RLS parameters for online learning
        struct RLSModel {
            Eigen::MatrixXd P;          // Covariance matrix (7x7)
            Eigen::MatrixXd theta;      // Parameter matrix (7x3) - CHANGED! maps features to forces
            double lambda = 0.98;       
            double regularization = 0.01;
            bool initialized = false;
            
            void initialize() {
                P = Eigen::MatrixXd::Identity(7, 7) * 100.0;
                theta = Eigen::MatrixXd::Zero(7, 3);  // CHANGED from (3, 7) to (7, 3)
                initialized = true;
            }
            
            void update(const Eigen::VectorXd& features, const Eigen::Vector3d& forces) {
                if (!initialized) initialize();
                
                // RLS update equations - FIXED
                Eigen::Vector3d y_pred = theta.transpose() * features;  // (3x7) * (7x1) = (3x1)
                Eigen::Vector3d error = forces - y_pred;
                
                // Kalman gain
                double denominator = lambda + features.transpose() * P * features;
                Eigen::MatrixXd K = P * features / denominator;  // (7x7) * (7x1) / scalar = (7x1)
                
                // Update parameters
                theta = theta + K * error.transpose();  // (7x3) + (7x1) * (1x3) = (7x3)
                
                // Update covariance
                P = (P - K * features.transpose() * P) / lambda;  // (7x7)
                
                // Add regularization
                P += Eigen::MatrixXd::Identity(7, 7) * regularization;
            }
        };

        // Historical data management
        struct HistoryBuffer {
            std::deque<Eigen::Vector3d> forces;
            std::deque<double> timestamps;
            std::deque<double> headings;
            std::deque<Eigen::Vector2d> velocities;
            static constexpr size_t MAX_SIZE = 200;  // 10 seconds at 20Hz
            
            void add(const Eigen::Vector3d& force, double time, double heading, const Eigen::Vector2d& vel) {
                forces.push_back(force);
                timestamps.push_back(time);
                headings.push_back(heading);
                velocities.push_back(vel);
                
                while (forces.size() > MAX_SIZE) {
                    forces.pop_front();
                    timestamps.pop_front();
                    headings.pop_front();
                    velocities.pop_front();
                }
            }
            
            bool hasEnoughData() const {
                return forces.size() >= 40;  // Need at least 2 seconds
            }
        };
        
        // FFT for frequency analysis
        struct SpectralAnalyzer {
            double findDominantFrequency(const std::deque<Eigen::Vector3d>& forces, 
                                        const std::deque<double>& timestamps) {
                if (forces.size() < 20) return 0.0;
                
                // Simple peak detection in force magnitude
                std::vector<double> magnitudes;
                for (const auto& f : forces) {
                    magnitudes.push_back(f.norm());
                }
                
                // Count zero crossings to estimate frequency
                double mean = std::accumulate(magnitudes.begin(), magnitudes.end(), 0.0) / magnitudes.size();
                int crossings = 0;
                for (size_t i = 1; i < magnitudes.size(); ++i) {
                    if ((magnitudes[i-1] - mean) * (magnitudes[i] - mean) < 0) {
                        crossings++;
                    }
                }
                
                double time_span = timestamps.back() - timestamps.front();
                return crossings / (2.0 * time_span);  // Frequency in Hz
            }
            
            double estimatePhase(const std::deque<Eigen::Vector3d>& forces, double frequency) {
                if (frequency < 0.01 || forces.empty()) return 0.0;
                
                // Estimate current phase based on recent force pattern
                double current_magnitude = forces.back().norm();
                double mean_magnitude = 0.0;
                for (const auto& f : forces) {
                    mean_magnitude += f.norm();
                }
                mean_magnitude /= forces.size();
                
                // Simple phase estimate based on deviation from mean
                double normalized = (current_magnitude - mean_magnitude) / (mean_magnitude + 0.01);
                return asin(std::max(-1.0, std::min(1.0, normalized)));
            }
        };
        
        // Main components
        RLSModel rls_model;
        HistoryBuffer history;
        SpectralAnalyzer spectral;
        Features current_features;
        
        // Confidence tracking
        double prediction_confidence = 0.0;
        std::deque<double> recent_errors;
        static constexpr size_t ERROR_HISTORY_SIZE = 20;
        
        // Extract features from current history
        Features extractFeatures() {
            Features feat;
            
            if (!history.hasEnoughData()) {
                return feat;  // Return zeros if not enough data
            }
            
            // Calculate mean magnitude
            feat.mean_magnitude = 0.0;
            for (const auto& f : history.forces) {
                feat.mean_magnitude += f.norm();
            }
            feat.mean_magnitude /= history.forces.size();
            
            // Find dominant frequency
            feat.dominant_frequency = spectral.findDominantFrequency(history.forces, history.timestamps);
            
            // Calculate variance
            feat.variance = 0.0;
            for (const auto& f : history.forces) {
                double diff = f.norm() - feat.mean_magnitude;
                feat.variance += diff * diff;
            }
            feat.variance /= history.forces.size();
            feat.variance = sqrt(feat.variance);
            
            // Calculate recent trends (last 1 second)
            size_t trend_points = std::min(size_t(20), history.forces.size());
            if (trend_points >= 2) {
                double dt = history.timestamps.back() - history.timestamps[history.forces.size() - trend_points];
                if (dt > 0) {
                    Eigen::Vector3d recent_change = history.forces.back() - 
                                                   history.forces[history.forces.size() - trend_points];
                    feat.trend_x = recent_change.x() / dt;
                    feat.trend_y = recent_change.y() / dt;
                    feat.trend_yaw = recent_change.z() / dt;
                }
            }
            
            // Estimate phase
            feat.phase_estimate = spectral.estimatePhase(history.forces, feat.dominant_frequency);
            
            return feat;
        }
        
        // Main prediction function
        Eigen::Vector3d predict(double horizon_seconds) {
            // Clamp prediction horizon
            horizon_seconds = std::min(3.0, std::max(0.0, horizon_seconds));
            
            if (!history.hasEnoughData() || !rls_model.initialized) {
                // Fallback to simple decay if not enough data
                if (!history.forces.empty()) {
                    return history.forces.back() * exp(-0.25 * horizon_seconds);
                }
                return Eigen::Vector3d::Zero();
            }
            
            // Extract current features
            Features feat = extractFeatures();
            Eigen::VectorXd feature_vec = feat.toVector();
            
            // Evolve features forward in time
            Eigen::VectorXd evolved_features = evolveFeatures(feature_vec, horizon_seconds);
            
            // Use RLS model for prediction
            Eigen::Vector3d predicted = rls_model.theta.transpose() * evolved_features;
            
            // Apply confidence-based scaling
            double confidence_scale = calculateConfidenceScale(horizon_seconds);
            predicted *= confidence_scale;
            
            // Enforce physical constraints
            double max_force = 100.0;  // N - reasonable maximum
            for (int i = 0; i < 3; ++i) {
                predicted(i) = std::max(-max_force, std::min(max_force, predicted(i)));
            }
            
            return predicted;
        }
        
        // Update model with new observation
        void update(const Eigen::Vector3d& measured_forces, double current_time, 
                   double heading, const Eigen::Vector2d& velocity) {
            // Add to history
            history.add(measured_forces, current_time, heading, velocity);
            
            if (!history.hasEnoughData()) {
                return;  // Wait for more data
            }
            
            // Extract features
            current_features = extractFeatures();
            
            // Update RLS model
            rls_model.update(current_features.toVector(), measured_forces);
            
            // Track prediction errors for confidence estimation
            if (history.forces.size() >= 21) {  // Can check 1-second-ago prediction
                Eigen::Vector3d predicted_before = predict(1.0);
                Eigen::Vector3d actual = history.forces.back();
                double error = (predicted_before - actual).norm();
                
                recent_errors.push_back(error);
                while (recent_errors.size() > ERROR_HISTORY_SIZE) {
                    recent_errors.pop_front();
                }
                
                // Update confidence based on recent performance
                if (recent_errors.size() >= 10) {
                    double mean_error = std::accumulate(recent_errors.begin(), 
                                                       recent_errors.end(), 0.0) / recent_errors.size();
                    prediction_confidence = exp(-mean_error / 30.0);  // Increased denominator from 20.0
                    prediction_confidence = std::max(0.2, prediction_confidence); // Set minimum floor
                }
            }
            // Store current 1s prediction for future validation
            Vector3d pred_1s = predict(1.0);
            validation_metrics.predictions_1s_ago.push_back(pred_1s);
            validation_metrics.timestamps_for_validation.push_back(current_time);
            
            // Check if we have a 1-second-old prediction to validate
            if (validation_metrics.timestamps_for_validation.size() >= 20) { // 1s at 20Hz
                validation_metrics.actual_forces_1s_later.push_back(measured_forces);
                
                // Keep buffer size manageable
                if (validation_metrics.predictions_1s_ago.size() > 40) {
                    validation_metrics.predictions_1s_ago.pop_front();
                    validation_metrics.actual_forces_1s_later.pop_front();
                    validation_metrics.timestamps_for_validation.pop_front();
                }
                
                // Update RMSE calculation
                validation_metrics.updateRMSE();
            }
        }
        
        // Get prediction uncertainty
        Eigen::Matrix3d getPredictionCovariance(double horizon_seconds) {
            double base_uncertainty = 2.0;  // REDUCED from 5.0
            
            if (!recent_errors.empty()) {
                base_uncertainty = std::min(5.0,  // Cap maximum
                    std::accumulate(recent_errors.begin(), 
                                  recent_errors.end(), 0.0) / recent_errors.size());
            }
            
            // More conservative growth
            double uncertainty = base_uncertainty * (1.0 + 0.5 * horizon_seconds); // Reduced from quadratic
            return Eigen::Matrix3d::Identity() * uncertainty * uncertainty;
        }

        struct ValidationMetrics {
            std::deque<Vector3d> predictions_1s_ago;
            std::deque<Vector3d> actual_forces_1s_later;
            std::deque<double> timestamps_for_validation;
            double rmse_1s = 0.0;
            int validation_count = 0;
            
            void updateRMSE() {
                if (predictions_1s_ago.size() < 20) return; // Need enough samples
                
                double sum_squared_error = 0.0;
                int count = 0;
                
                for (size_t i = 0; i < std::min(predictions_1s_ago.size(), 
                                                actual_forces_1s_later.size()); i++) {
                    Vector3d error = predictions_1s_ago[i] - actual_forces_1s_later[i];
                    sum_squared_error += error.squaredNorm();
                    count++;
                }
                
                if (count > 0) {
                    rmse_1s = sqrt(sum_squared_error / count);
                    validation_count = count;
                }
            }
        };
        
        ValidationMetrics validation_metrics;
        
    private:
        // Evolve features forward in time
        Eigen::VectorXd evolveFeatures(const Eigen::VectorXd& current, double dt) {
            Eigen::VectorXd evolved = current;
            
            // Phase evolves based on frequency
            if (current(1) > 0.01) {  // If there's a dominant frequency
                evolved(6) = fmod(current(6) + 2.0 * M_PI * current(1) * dt, 2.0 * M_PI);
            }
            
            // Trends decay over time
            double trend_decay = exp(-dt / 2.0);  // 2-second decay constant
            evolved(3) *= trend_decay;  // trend_x
            evolved(4) *= trend_decay;  // trend_y
            evolved(5) *= trend_decay;  // trend_yaw
            
            // Variance typically decreases with averaging
            evolved(2) *= sqrt(1.0 + dt);  // variance increases with uncertainty
            
            return evolved;
        }
        
        // Calculate confidence-based scaling factor
        double calculateConfidenceScale(double horizon_seconds) {
            // Start with time-based decay
            double time_decay = exp(-horizon_seconds / 3.0);  // 3-second decay constant
            
            // Modify based on prediction confidence
            double confidence_factor = 0.5 + 0.5 * prediction_confidence;
            
            // Combine factors
            return time_decay * confidence_factor;
        }
    };

    DataDrivenEnvironmentalPredictor env_predictor;   
    
    PredictionValidation pred_validation;
    static const int VALIDATION_HISTORY_SIZE = 60;  // 3 seconds at 20Hz

    // Enhanced planning stability variables
    int committed_zone_index = -1;
    double commitment_distance_threshold = 50.0;  // Progressive commitment threshold
    double zone_switching_penalty = 0.3;         // Penalty for switching zones
    double zone_commitment_bias = 0.2;           // Bonus for maintaining current zone
    bool zone_locked = false;                    // Hard lock when very close to target
    double last_zone_switch_time = 0.0;
    static constexpr double MIN_ZONE_SWITCH_INTERVAL = 3.0;  // Minimum seconds between switches

    // Drift compensation parameters
    double drift_compensation_factor = 0.8;      // How much to compensate for drift (0-1)
    double max_drift_offset = 20.0;              // Maximum drift compensation offset
    Vector2d last_planned_target;

    std::deque<double> recent_heading_changes;
    static constexpr size_t HEADING_HISTORY_SIZE = 10;
    double accumulated_heading_change = 0.0;
    double last_planned_heading = 0.0;  // Track last planned heading for oscillation detection

    // Mission metrics tracking
    double mission_start_time = -1.0;      // Time when fault occurs
    double mission_end_time = -1.0;        // Time when harbor reached
    double mission_duration = 0.0;         // Total mission time
    double mission_energy_consumed = 0.0;  // Total energy used (Joules)
    double instantaneous_power = 0.0;      // Current power consumption (Watts)
    bool mission_metrics_active = false;   // Track if we're counting metrics
    double min_distance_to_harbor;


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
    // void assessCurrentSituation();
    // fast planning
    void initializeHarborZones();
    bool pointInPolygon(const Vector2d& point, const std::vector<Vector2d>& polygon);
    bool lineIntersectsPolygon(const Vector2d& start, const Vector2d& end, const std::vector<Vector2d>& polygon);
    bool isAboveBoundaryLine(const Vector2d& point);
    bool isBelowBoundaryLine(const Vector2d& point);
    bool isPathObstacleFree(const Vector2d& start, const Vector2d& end);
    double calculateEnvironmentalAlignment(const Vector2d& path_direction);
    void fastPlanning();
    bool needsReplanning();
    void updatePlanningAndReference();
    void initializeOperationalMode();
    void generateStationKeepingTrajectory();
    void generateAdaptiveReturnTrajectory();
    void updateOperationalMode();
    void ref_cb_enhanced(int line_to_read);
    double convertToContinuousPsi(double target_heading_bounded, double current_continuous_psi);
    bool hasArrivedAtHarborZone();
    bool isPathCorridorFree(const Vector2d& start, const Vector2d& end, double corridor_width);
    // In wamv_mpc.h, add these private methods:
    Vector2d findBestTargetInZone(int zone_idx, const Vector2d& current_pos);
    Vector2d findAlternativeTargetInZone(int zone_idx, const Vector2d& current_pos);
    Vector2d calculateDriftCompensatedTarget(const HarborZone& zone, 
                                            const Vector2d& current_pos,
                                            const Vector2d& drift_estimate);
    double calculateZoneScore(int zone_idx, const Vector2d& target,
                            const Vector2d& current_pos, 
                            const Vector2d& path_dir,
                            bool obstacle_free);

    void updateEnvironmentalAssistance();
    void adaptMPCWeights();

    void updateEnvironmentalPrediction();
    void validatePredictions();
    void publishPredictionMetrics();
    Vector3d transformBodyToInertial(const Vector3d& forces_body, double heading);

    double calculatePowerFromThrust(double thrust);
};

#endif