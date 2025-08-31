#include <vrx_control/wamv_mpc.h>

WAMV_MPC::WAMV_MPC() 
: Node("wamv_mpc_node"),
  wpsi_coefficient(0.05),      // Initial guess for the coefficient
  calibration_counter(0),
  calibration_enabled(true)    // Enable calibration by default
{
    // Declare parameters with default values
    this->declare_parameter<int>("read_wrench", 0);
    this->declare_parameter<bool>("compensate_d", false);
    this->declare_parameter<std::string>("ref_traj", "");

    // Get parameters
    this->get_parameter("read_wrench", READ_WRENCH);
    this->get_parameter("compensate_d", COMPENSATE_D);
    this->get_parameter("ref_traj", REF_TRAJ);
    
    // Pre-load the trajectory
    // REF_TRAJ = "/home/yang/usv_ws/src/vrx/vrx_control/traj/stationary.txt";
    const char * c = REF_TRAJ.c_str();
	number_of_steps = readDataFromFile(c, trajectory);
	if (number_of_steps == 0)
    {
        RCLCPP_WARN(this->get_logger(), "Cannot load CasADi optimal trajectory!");
    } 
    else 
    {
        RCLCPP_INFO(this->get_logger(), "Number of steps of selected trajectory: %d", number_of_steps);
    }

    // Initialize MPC
    int create_status = 1;
    create_status = wamv_acados_create(mpc_capsule);
    if (create_status != 0){
        RCLCPP_ERROR(this->get_logger(), "acados_create() returned status %d. Exiting.", create_status);
        rclcpp::shutdown();
        exit(1);
    }

    // ros subsriber & publisher
    states_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/wamv/sensors/position/ground_truth_odometry",
        20,
        std::bind(&WAMV_MPC::states_cb, this, std::placeholders::_1));
    left_thrust_cmd_pub = this->create_publisher<std_msgs::msg::Float64>(
        "/wamv/thrusters/left/thrust", 20);
    right_thrust_cmd_pub = this->create_publisher<std_msgs::msg::Float64>(
        "/wamv/thrusters/right/thrust", 20);
    ref_pose_pub = this->create_publisher<nav_msgs::msg::Odometry>(
        "/wamv/ref_pose", 20);
    error_pose_pub = this->create_publisher<nav_msgs::msg::Odometry>(
        "/wamv/error_pose", 20);
    control_inputs_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/wamv/control_inputs", 20);
    ekf_pose_pub = this->create_publisher<nav_msgs::msg::Odometry>(
        "/wamv/ekf_pose", 20);
    disturbance_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/wamv/disturbance", 20);
    confidence_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/wamv/status_confidence", 20);
    operational_mode_pub = this->create_publisher<std_msgs::msg::String>(
        "/wamv/operational_mode", 10);

    thruster_health_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/wamv/thruster_health", 10);

    usv_state_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/wamv/usv_state", 20);

    environmental_assistance_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/wamv/environmental_assistance", 10);

    planning_status_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/wamv/planning_status", 10);

    // harbor_zones_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        // "/wamv/harbor_zones", 1); // Low frequency for static data

    // initialize
    for(unsigned int i=0; i < WAMV_NU; i++) acados_out.u0[i] = 0.0;
    for(unsigned int i=0; i < WAMV_NX; i++) acados_in.x0[i] = 0.0;
    start_time = rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds();
    is_start = false;
    solver_param.Tp_pre = 0;
    solver_param.Ts_pre = 0;

    Q_cov << 1e-3, 1e-3, 1e-3, 
            1e-2, 1e-2, 1e-2, 
            0.01, 0.01, 0.001;
    noise_Q= Q_cov.asDiagonal();
    R_cov << 1e-4, 1e-4, 1e-4, 
            1e-3, 1e-3, 1e-3, 
            0.1, 0.1, 0.1;
    noise_R = R_cov.asDiagonal();
    R_imu_cov << 1.95e-4, 1.96e-6, 7.27, 7.27;
    noise_R_imu = R_imu_cov.asDiagonal();
    
    esti_x << 0,0,0,0,0,0,0,0,0;
    esti_P = P0;
    M_values << 180, 180, 446;
    M = M_values.asDiagonal();
    invM = M.inverse();

    // Initialize fault diagnosis parameters
    this->declare_parameter<int>("window_size", 20);
    this->declare_parameter<double>("learning_rate", 0.01);
    this->declare_parameter<double>("lambda", 0.001);
    this->declare_parameter<double>("wx_threshold", 15.0);     // Increased
    this->declare_parameter<double>("wy_threshold", 15.0);     // Increased
    this->declare_parameter<double>("wpsi_threshold", 5.0);    // Reduced - more sensitive to yaw
    this->declare_parameter<int>("detection_count_threshold", 1); // Lower for faster response
    this->declare_parameter<double>("detect_threshold", 0.60); // Reduced for better sensitivity

    this->get_parameter("window_size", window_size);
    this->get_parameter("wx_threshold", wx_threshold);
    this->get_parameter("wy_threshold", wy_threshold);
    this->get_parameter("wpsi_threshold", wpsi_threshold);
    this->get_parameter("detection_count_threshold", detection_count_threshold);
    
    // Initialize fault diagnosis publishers
    fault_diagnosis_pub = this->create_publisher<std_msgs::msg::String>(
        "/wamv/fault_diagnosis", 10);
    fault_features_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/wamv/fault_features", 10);

    // Add parameter for calibration
    this->declare_parameter<bool>("calibration_enabled", true);
    this->get_parameter("calibration_enabled", calibration_enabled);
    
    // Add publisher for the coefficient
    wpsi_coefficient_pub = this->create_publisher<std_msgs::msg::Float64>(
        "/wamv/wpsi_coefficient", 10);

    fault_confidence_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/wamv/fault_confidences", 10);

    initializeOperationalMode();
    initializeHarborZones(); // From previous fast planning code
    
    // Initialize calibration data with reserved capacity
    calibration_data.reserve(1000);
    
    // Initialize fault diagnosis model
    // initializeFaultDiagnosis();
    
    // Initialize previous thruster commands
    prev_Tp = 0.0;
    prev_Ts = 0.0;

    // Initialize confidences
    fault_confidences.resize(3, 0.0); // Initialize with 3 zeros (one for each fault type)

    mission_completed = false;
    arrival_time = 0.0;

    // Initialize environmental assistance
    environmental_assistance.current_forces = Vector3d::Zero();
    environmental_assistance.predicted_forces = Vector3d::Zero();
    environmental_assistance.assistance_capability = 0.0;
    environmental_assistance.is_reliable = false;
    environmental_assistance.surge_assistance_factor = 0.0;  // Start with no assistance
    environmental_assistance.sway_assistance_factor = 0.0;
    environmental_assistance.yaw_assistance_factor = 0.0;

    // Initialize adaptive weights
    adaptive_weights.use_environmental_assistance = false;
    adaptive_weights.environmental_weight_factor = 1.0;
    adaptive_weights.thruster_penalty_factor = 1.0;
    adaptive_weights.fault_compensation_gain = 0.0;

    initializeTrendPrediction();
}

// subscribe pos and vel
void WAMV_MPC::states_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    is_start = true;

    // Get linear position x, y, z
    local_pos.x = msg->pose.pose.position.x;
    local_pos.y = msg->pose.pose.position.y;
    local_pos.z = msg->pose.pose.position.z;

    // Get linear velocity u, v, w
    local_pos.u = msg->twist.twist.linear.x;
    local_pos.v = msg->twist.twist.linear.y;
    local_pos.w = msg->twist.twist.linear.z;

    // Get angular velocity p, q, r
    local_pos.p = msg->twist.twist.angular.x;
    local_pos.q = msg->twist.twist.angular.y;
    local_pos.r = msg->twist.twist.angular.z;

    // Convert quaternion to roll, pitch, yaw
    tf2::Quaternion tf_quaternion(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    // Normalize the quaternion
    tf_quaternion.normalize();

    // Extract roll, pitch, and yaw
    tf2::Matrix3x3(tf_quaternion).getRPY(local_pos.phi, local_pos.theta, local_pos.psi);
    // Convert velocity to body frame
    // v_inertial << local_pos.u, local_pos.v, local_pos.r;
    // R_ib << cos(local_pos.psi), -sin(local_pos.psi), 0,
    //         sin(local_pos.psi), cos(local_pos.psi), 0,
    //         0, 0, 1;
    // v_body = R_ib.inverse() * v_inertial;
    // odom_acc = local_pos.u-
    odom_data_available = true;
}

void WAMV_MPC::imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    // Raw IMU data
    double p_raw = msg->angular_velocity.x;
    double q_raw = msg->angular_velocity.y;
    double r_raw = msg->angular_velocity.z;
    double ax_raw = msg->linear_acceleration.x;
    double ay_raw = msg->linear_acceleration.y;
    double az_raw = msg->linear_acceleration.z;

    tf2::Quaternion tf_quaternion(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    tf_quaternion.normalize();
    double phi_raw, theta_raw, psi_raw;
    tf2::Matrix3x3(tf_quaternion).getRPY(phi_raw, theta_raw, psi_raw);

    // Initialize smoothed values on first message
    if (first_imu) {
        imu_filter.p_smoothed = p_raw;
        imu_filter.q_smoothed = q_raw;
        imu_filter.r_smoothed = r_raw;
        imu_filter.x_smoothed = ax_raw;
        imu_filter.y_smoothed = ay_raw;
        imu_filter.z_smoothed = az_raw;
        imu_filter.phi_smoothed = phi_raw;
        imu_filter.theta_smoothed = theta_raw;
        imu_filter.psi_smoothed = psi_raw;
        first_imu = false;
    } else {
        // Apply EMA: smoothed = alpha * raw + (1 - alpha) * previous_smoothed
        imu_filter.p_smoothed = alpha * p_raw + (1.0 - alpha) * imu_filter.p_smoothed;
        imu_filter.q_smoothed = alpha * q_raw + (1.0 - alpha) * imu_filter.q_smoothed;
        imu_filter.r_smoothed = alpha * r_raw + (1.0 - alpha) * imu_filter.r_smoothed;
        imu_filter.x_smoothed = alpha * ax_raw + (1.0 - alpha) * imu_filter.x_smoothed;
        imu_filter.y_smoothed = alpha * ay_raw + (1.0 - alpha) * imu_filter.y_smoothed;
        imu_filter.z_smoothed = alpha * az_raw + (1.0 - alpha) * imu_filter.z_smoothed;
        imu_filter.phi_smoothed = alpha * phi_raw + (1.0 - alpha) * imu_filter.phi_smoothed;
        imu_filter.theta_smoothed = alpha * theta_raw + (1.0 - alpha) * imu_filter.theta_smoothed;
        imu_filter.psi_smoothed = alpha * psi_raw + (1.0 - alpha) * imu_filter.psi_smoothed;
    }
    // Assign smoothed values to EKF inputs
    imu_pos.p = imu_filter.p_smoothed;
    imu_pos.q = imu_filter.q_smoothed;
    imu_pos.r = imu_filter.r_smoothed;
    imu_acc.x = imu_filter.x_smoothed;
    imu_acc.y = imu_filter.y_smoothed;
    imu_acc.z = imu_filter.z_smoothed;
    imu_pos.phi = imu_filter.phi_smoothed;
    imu_pos.theta = imu_filter.theta_smoothed;
    imu_pos.psi = imu_filter.psi_smoothed;

    if (!std::isnan(imu_pos.psi) && !std::isnan(imu_pos.r) && 
        !std::isnan(imu_acc.x) && !std::isnan(imu_acc.y)) {
        imu_data_available = true;
    }
}

// read trajectory data
int WAMV_MPC::readDataFromFile(const char* fileName, std::vector<std::vector<double>> &data)
{
	std::ifstream file(fileName);
	std::string line;
	int number_of_lines = 0;

	if (file.is_open())
	{
        std::cout<<"file is open"<<std::endl;
		while(getline(file, line)){
			number_of_lines++;
			std::istringstream linestream( line );
			std::vector<double> linedata;
			double number;

			while( linestream >> number ){
				linedata.push_back( number );
			}
			data.push_back( linedata );
		}

		file.close();
	}
	else
	{
        std::cout<<"file not open"<<std::endl;
		return 0;
	}

	return number_of_lines;
}
void WAMV_MPC::ref_cb(int line_to_read)
{
    if (WAMV_N+line_to_read+1 <= number_of_steps)  // All ref points within the file
    {
        for (unsigned int i = 0; i <= WAMV_N; i++)  // Fill all horizon with file data
        {
            for (unsigned int j = 0; j <= WAMV_NY; j++)
            {
                acados_in.yref[i][j] = trajectory[i+line_to_read][j];
            }
        }
    }
    else if(line_to_read < number_of_steps)    // Part of ref points within the file
    {
        for (int i = 0; i < number_of_steps-line_to_read; i++)    // Fill part of horizon with file data
        {
            
            for (unsigned int j = 0; j <= WAMV_NY; j++)
            {
                acados_in.yref[i][j] = trajectory[i+line_to_read][j];
            }
            
        }

        for (unsigned int i = number_of_steps-line_to_read; i <= WAMV_N; i++)  // Fill the rest horizon with the last point
        {
            
            for (unsigned int j = 0; j <= WAMV_NY; j++)
            {
                acados_in.yref[i][j] = trajectory[number_of_steps-1][j];
            }
            
        }
    }
    else    // none of ref points within the file
    {
        for (unsigned int i = 0; i <= WAMV_N; i++)  // Fill all horizon with the last point
        {
            
            for (unsigned int j = 0; j <= WAMV_NY; j++)
            {
                acados_in.yref[i][j] = trajectory[number_of_steps-1][j];
            }
            
        }
    }
    
}

void WAMV_MPC::solve()
{
    // identify turning direction
    if (pre_yaw >= 0 && local_pos.psi >=0)
    {
        yaw_diff = local_pos.psi - pre_yaw;
    }
    else if (pre_yaw >= 0 && local_pos.psi <0)
    {
        if (2*M_PI+local_pos.psi-pre_yaw >= pre_yaw+abs(local_pos.psi))
        {
            yaw_diff = -(pre_yaw + abs(local_pos.psi));
        }
        else
        {
            yaw_diff = 2 * M_PI + local_pos.psi - pre_yaw;
        }
    }
    else if (pre_yaw < 0 && local_pos.psi >= 0)
    {
        if (2*M_PI-local_pos.psi+pre_yaw >= abs(pre_yaw)+local_pos.psi)
        {
            yaw_diff = abs(pre_yaw)+local_pos.psi;
        }
        else
        {
            yaw_diff = -(2*M_PI-local_pos.psi+pre_yaw);
        }
    }
    else
    {
        yaw_diff = local_pos.psi - pre_yaw;
    }

    yaw_sum = yaw_sum + yaw_diff;
    pre_yaw = local_pos.psi;

    // set initial states
    acados_in.x0[x] = local_pos.x;
    acados_in.x0[y] = local_pos.y;
    acados_in.x0[psi] = yaw_sum;
    acados_in.x0[u] = local_pos.u;
    acados_in.x0[v] = local_pos.v;
    acados_in.x0[r] = local_pos.r;
    ocp_nlp_constraints_model_set(mpc_capsule->nlp_config,mpc_capsule->nlp_dims,mpc_capsule->nlp_in, 0, "lbx", acados_in.x0);
    ocp_nlp_constraints_model_set(mpc_capsule->nlp_config,mpc_capsule->nlp_dims,mpc_capsule->nlp_in, 0, "ubx", acados_in.x0);

    // Update environmental assistance before MPC solve
    updateEnvironmentalAssistance();
    updateValidationData();           // New: collect validation data
    fillMPCHorizonWithPrediction();   // New: fill MPC horizon
    adaptMPCWeights();

    // set parameters
    // double u_prev[2] = {solver_param.Tp_pre, solver_param.Ts_pre};
    double health_Tp = 1.0;  // Default: healthy
    double health_Ts = 1.0;  // Default: healthy
    
    if (iteration_count >= fault_trigger) {
        switch (FAULT_TYPE_TO_SIMULATE) {
            case LEFT_THRUSTER_FAULT_SIM:
                health_Tp = 1.0 - thruster_degrade_percentage;
                break;
            case RIGHT_THRUSTER_FAULT_SIM:
                health_Ts = 1.0 - thruster_degrade_percentage;
                break;
        }
    }

    double params[7] = {
        solver_param.Tp_pre, 
        solver_param.Ts_pre,
        esti_x[6] * environmental_assistance.surge_assistance_factor,
        esti_x[7] * environmental_assistance.sway_assistance_factor,
        esti_x[8] * environmental_assistance.yaw_assistance_factor,   // w_psi - environmental moment in yaw
        health_Tp,
        health_Ts
    };
    for (int i = 0; i <= WAMV_N; i++) {
        for (int j = 0; j < 7; j++) {  // CHANGE: from 2 to 5
            acados_param[i][j] = params[j];
        }
        wamv_acados_update_params(mpc_capsule, i, acados_param[i], WAMV_NP);
    }

    // change into form of (-pi, pi)
    if(sin(acados_in.yref[0][2]) >= 0)
    {
        yaw_ref = fmod(acados_in.yref[0][2],M_PI);
    }
    else{
        yaw_ref = -M_PI + fmod(acados_in.yref[0][2],M_PI);
    }

    // set reference
    // Update operational mode and trajectory generation
    updateOperationalMode();
    
    // Use enhanced reference callback instead of original ref_cb
    ref_cb_enhanced(line_number);
    line_number++;
    for (unsigned int i = 0; i <= WAMV_N; i++){
        ocp_nlp_cost_model_set(mpc_capsule->nlp_config, mpc_capsule->nlp_dims, mpc_capsule->nlp_in, i, "yref", acados_in.yref[i]);
    }

    // Solve OCP
    acados_status = wamv_acados_solve(mpc_capsule);

    if (acados_status != 0){
        RCLCPP_INFO(this->get_logger(), "acados returned status: %d", acados_status);
    }

    acados_out.status = acados_status;
    acados_out.kkt_res = (double)mpc_capsule->nlp_out->inf_norm_res;

    // ocp_nlp_get(mpc_capsule->nlp_config, mpc_capsule->nlp_solver, "time_tot", &acados_out.cpu_time);
    ocp_nlp_get(mpc_capsule->nlp_solver, "time_tot", &acados_out.cpu_time);

    ocp_nlp_out_get(mpc_capsule->nlp_config, mpc_capsule->nlp_dims, mpc_capsule->nlp_out, 0, "u", (void *)acados_out.u0);

    // acados_out.u0[0] = 200;
    // acados_out.u0[1] = 200;
    
    publish_cin(acados_out.u0[0], acados_out.u0[1]);
    
}

void WAMV_MPC::publish_cin(double Tp_mpc, double Ts_mpc)
{
    std::string fault_status;
    std::string fault_color;
    std::string mode_color;
    std::string mode_name;
    
    // Apply fault at the fault trigger point
    if (iteration_count < fault_trigger) {
        // Normal operation before fault trigger
        Tp.data = Tp_mpc;
        Ts.data = Ts_mpc;

        fault_status = "NORMAL";
        fault_color = "\033[32m"; // Green
        
        // Reset simulation tracking
        fault_simulation_active = false;
        simulated_fault_type = NO_FAULT;
    } else {
        // Apply the selected fault simulation
        switch (FAULT_TYPE_TO_SIMULATE) {
            case LEFT_THRUSTER_FAULT_SIM:
                fault_detected = true;
                Tp.data = Tp_mpc*(1-thruster_degrade_percentage);     // Port thruster fails
                Ts.data = Ts_mpc;  // Starboard thruster normal
                
                // Track the simulation state
                fault_simulation_active = true;
                simulated_fault_type = LEFT_THRUST_FAILURE;

                fault_status = "LEFT_THRUST_FAILURE";
                fault_color = "\033[31m"; // Red
                
                // RCLCPP_INFO(this->get_logger(), "Simulating port thruster force failure at iteration %zu", iteration_count);
                break;
                
            case RIGHT_THRUSTER_FAULT_SIM:
                fault_detected = true;
                Tp.data = Tp_mpc;  // Port thruster normal
                Ts.data = Ts_mpc*(1-thruster_degrade_percentage);     // Starboard thruster fails
                
                // Track the simulation state
                fault_simulation_active = true;
                simulated_fault_type = RIGHT_THRUST_FAILURE;

                fault_status = "RIGHT_THRUST_FAILURE";
                fault_color = "\033[31m"; // Red
                
                // RCLCPP_INFO(this->get_logger(), "Simulating starboard thruster force failure at iteration %zu", iteration_count);
                break;
                
            default:
                fault_detected = false;
                Tp.data = Tp_mpc;  // Normal operation
                Ts.data = Ts_mpc;

                fault_status = "NORMAL";
                fault_color = "\033[32m"; // Green
                
                fault_simulation_active = false;
                simulated_fault_type = NO_FAULT;
                break;
        }
    }
    iteration_count++;

    switch (current_mode) {
        case FOLLOW_PRESET_TRAJECTORY:
            mode_name = "PRESET_TRAJ";
            mode_color = "\033[36m"; // Cyan
            break;
        case STATION_KEEPING:
            mode_name = "STATION_KEEP";
            mode_color = "\033[33m"; // Yellow
            break;
        case ADAPTIVE_ASSISTED_RETURN:
            mode_name = "ADAPTIVE_RETURN";
            mode_color = "\033[35m"; // Magenta
            break;
        default:
            mode_name = "UNKNOWN";
            mode_color = "\033[37m"; // White
    }
    
    // Send actual values to thrusters
    left_thrust_cmd_pub->publish(Tp);
    right_thrust_cmd_pub->publish(Ts);

    // Update control inputs message with actual values (not commanded values)
    control_inputs.header.stamp = rclcpp::Clock().now();
    control_inputs.twist.linear.y = Tp.data;  // Use actual Tp.data
    control_inputs.twist.angular.y = Ts.data; // Use actual Ts.data
    control_inputs_pub->publish(control_inputs);

    // publish reference states
    tf2::Quaternion quat;
    quat.setRPY(0, 0, yaw_ref);
    geometry_msgs::msg::Quaternion quat_msg;
    tf2::convert(quat, quat_msg);
    ref_pose.pose.pose.position.x = acados_in.yref[0][0];
    ref_pose.pose.pose.position.y = acados_in.yref[0][1];
    ref_pose.pose.pose.orientation.x = quat_msg.x;
    ref_pose.pose.pose.orientation.y = quat_msg.y;
    ref_pose.pose.pose.orientation.z = quat_msg.z;
    ref_pose.pose.pose.orientation.w = quat_msg.w;
    
    ref_pose.header.stamp = rclcpp::Clock().now();
    ref_pose.header.frame_id = "odom_frame";
    ref_pose.child_frame_id = "base_link";
    ref_pose_pub->publish(ref_pose);

    // publish error states
    tf2::Quaternion quat_error;
    yaw_error = yaw_sum - acados_in.yref[0][2];
    quat_error.setRPY(0, 0, yaw_error);
    geometry_msgs::msg::Quaternion quat_error_msg;
    tf2::convert(quat_error, quat_error_msg);
    error_pose.pose.pose.position.x = acados_in.x0[0] - acados_in.yref[0][0];
    error_pose.pose.pose.position.y = acados_in.x0[1] - acados_in.yref[0][1];
    error_pose.pose.pose.orientation.x = quat_error_msg.x;
    error_pose.pose.pose.orientation.y = quat_error_msg.y;
    error_pose.pose.pose.orientation.z = quat_error_msg.z;
    error_pose.pose.pose.orientation.w = quat_error_msg.w;
    error_pose.header.stamp = rclcpp::Clock().now();
    error_pose.header.frame_id = "odom_frame";
    error_pose.child_frame_id = "base_link";

    error_pose_pub->publish(error_pose);


    solver_param.Tp_pre = acados_out.u0[0];
    solver_param.Ts_pre = acados_out.u0[1];

    double current_time = rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds();
    double z[4];
    ocp_nlp_out_get(mpc_capsule->nlp_config, mpc_capsule->nlp_dims, mpc_capsule->nlp_out, 0, "z", z);
    
    // Calculate calibrated w_psi for display
    // double calibrated_wpsi = getCalibrated_wpsi();
    // Add this at the end of publish_cin() function, before the closing brace

    // Publish fault diagnosis message
    std_msgs::msg::String fault_msg;
    if (iteration_count < fault_trigger) {
        fault_msg.data = "Fault: NO_FAULT";
    } else {
        switch (FAULT_TYPE_TO_SIMULATE) {
            case LEFT_THRUSTER_FAULT_SIM:
                fault_msg.data = "Fault: LEFT_THRUST_FAILURE";
                break;
            case RIGHT_THRUSTER_FAULT_SIM:
                fault_msg.data = "Fault: RIGHT_THRUST_FAILURE";
                break;
            default:
                fault_msg.data = "Fault: NO_FAULT";
        }
    }
    fault_diagnosis_pub->publish(fault_msg);

    // Publish operational mode
    std_msgs::msg::String mode_msg;
    switch (current_mode) {
        case FOLLOW_PRESET_TRAJECTORY:
            mode_msg.data = "FOLLOW_PRESET_TRAJECTORY";
            break;
        case STATION_KEEPING:
            mode_msg.data = "STATION_KEEPING";
            break;
        case ADAPTIVE_ASSISTED_RETURN:
            mode_msg.data = "ADAPTIVE_ASSISTED_RETURN";
            break;
        default:
            mode_msg.data = "UNKNOWN";
    }
    operational_mode_pub->publish(mode_msg);

    // Publish thruster health
    std_msgs::msg::Float64MultiArray thruster_health_msg;
    thruster_health_msg.data.resize(4);
    if (iteration_count >= fault_trigger) {
        switch (FAULT_TYPE_TO_SIMULATE) {
            case LEFT_THRUSTER_FAULT_SIM:
                thruster_health_msg.data[0] = (1.0 - thruster_degrade_percentage) * 100.0; // Left health %
                thruster_health_msg.data[1] = 100.0; // Right health %
                break;
            case RIGHT_THRUSTER_FAULT_SIM:
                thruster_health_msg.data[0] = 100.0; // Left health %
                thruster_health_msg.data[1] = (1.0 - thruster_degrade_percentage) * 100.0; // Right health %
                break;
            default:
                thruster_health_msg.data[0] = 100.0; // Left health %
                thruster_health_msg.data[1] = 100.0; // Right health %
        }
    } else {
        thruster_health_msg.data[0] = 100.0; // Left health %
        thruster_health_msg.data[1] = 100.0; // Right health %
    }
    thruster_health_msg.data[2] = Tp_mpc; // Commanded left thrust
    thruster_health_msg.data[3] = Ts_mpc; // Commanded right thrust
    thruster_health_pub->publish(thruster_health_msg);

    // Publish environmental assistance status
    std_msgs::msg::Float64MultiArray env_assist_msg;
    env_assist_msg.data.resize(6);
    env_assist_msg.data[0] = environmental_assistance.surge_assistance_factor;
    env_assist_msg.data[1] = environmental_assistance.sway_assistance_factor;
    env_assist_msg.data[2] = environmental_assistance.yaw_assistance_factor;
    env_assist_msg.data[3] = esti_x[6]; // Current w_x
    env_assist_msg.data[4] = esti_x[7]; // Current w_y
    env_assist_msg.data[5] = esti_x[8]; // Current w_psi
    environmental_assistance_pub->publish(env_assist_msg);

    // Publish planning status (only if in adaptive mode and plan is valid)
    if (current_mode == ADAPTIVE_ASSISTED_RETURN && current_plan.is_valid) {
        std_msgs::msg::Float64MultiArray planning_msg;
        planning_msg.data.resize(6);
        planning_msg.data[0] = current_plan.selected_harbor_zone;
        planning_msg.data[1] = current_plan.target_point.x();
        planning_msg.data[2] = current_plan.target_point.y();
        planning_msg.data[3] = current_plan.path_distance;
        planning_msg.data[4] = current_plan.feasibility_score;
        planning_msg.data[5] = current_plan.obstacle_free ? 1.0 : 0.0;
        planning_status_pub->publish(planning_msg);
    }

    // Publish enhanced USV state for map visualization
    geometry_msgs::msg::PoseStamped usv_state_msg;
    usv_state_msg.header.stamp = rclcpp::Clock().now();
    usv_state_msg.header.frame_id = "odom_frame";
    usv_state_msg.pose.position.x = local_pos.x;
    usv_state_msg.pose.position.y = local_pos.y;
    usv_state_msg.pose.position.z = 0.0;

    // Convert continuous yaw to quaternion
    tf2::Quaternion usv_quat;
    usv_quat.setRPY(0, 0, local_pos.psi);
    geometry_msgs::msg::Quaternion usv_quat_msg;
    tf2::convert(usv_quat, usv_quat_msg);
    usv_state_msg.pose.orientation = usv_quat_msg;
    usv_state_pub->publish(usv_state_msg);

    if(cout_counter > 2){
        std::cout << "---------------------------------------------------------------------------------------------------------------------" << std::endl;
        // ENHANCED: Add operational mode status line
        std::cout << mode_color << "OPERATIONAL MODE: " << mode_name 
                << " | Iteration: " << iteration_count 
                << " | Fault Trigger: " << fault_trigger;
        if (trajectory_generation_active) {
            std::cout << " | Generated Traj: " << generated_line_number 
                    << "/" << generated_trajectory.size();
        }
        if (current_mode == ADAPTIVE_ASSISTED_RETURN && current_plan.is_valid) {
            std::cout << " | Target Zone: " << current_plan.selected_harbor_zone
                    << " | Target: (" << std::fixed << std::setprecision(1) 
                    << current_plan.target_point.x() << ", " << current_plan.target_point.y() << ")";
        }
        std::cout << "\033[0m" << std::endl;
        std::cout << "ref_x:    " << acados_in.yref[0][0] << "\tref_y:   " << acados_in.yref[0][1] << "\tref_yaw:    " << acados_in.yref[0][2] << std::endl;
        std::cout << "error_x:  " << error_pose.pose.pose.position.x << "  error_y:  " << error_pose.pose.pose.position.y << "  error_psi:  " << yaw_error << std::endl;
        std::cout << "pos_x:  " << local_pos.x << "  pos_y:  " << local_pos.y << "  psi:  " << yaw_sum << std::endl;
        std::cout << "ekf pos_x:  " << esti_x[0] << "  pos_y:  " << esti_x[1] << "  psi:  " << esti_x[2] << std::endl;
        std::cout << "vel_x:  " << local_pos.u << "  vel_y:  " << local_pos.v << "  vel_r:  " << local_pos.r << std::endl;
        std::cout << "ekf vel_x:  " << esti_x[3] << "  vel_y:  " << esti_x[4] << "  vel_r:  " << esti_x[5] << std::endl;
        std::cout << "ekf w_x:  " << esti_x[6] << "  w_y:  " << esti_x[7] << "  w_psi:  " << esti_x[8] << std::endl;
        // std::cout << "calibrated w_psi: " << calibrated_wpsi << " (raw: " << esti_x[8] << ", expected: " << (Ts.data - Tp.data) * wpsi_coefficient << ")" << std::endl;
        std::cout << "ekf acc_x:  " << ekf_acc.x << "  acc_y:  " << ekf_acc.y << "  acc_psi:  " << ekf_acc.psi << std::endl;
        std::cout << "Tp:  " << acados_out.u0[0] << "  Ts:  " << acados_out.u0[1] << std::endl;
        std::cout << "solve_time: "<< acados_out.cpu_time << "\tkkt_res: " << acados_out.kkt_res << "\tacados_status: " << acados_out.status << std::endl;
        std::cout << "relative_time: " << std::fixed << (current_time - start_time) << std::endl;
        // NEW: Environmental prediction validation output
        if (pred_validation.validation_ready) {
            // Overall metrics first
            std::cout << "PREDICTION RMSE (overall): 0.5s=" << std::fixed << std::setprecision(2) 
                     << pred_validation.rmse_0_5s_overall << " | 1s=" << pred_validation.rmse_1s_overall 
                     << " | 2s=" << pred_validation.rmse_2s_overall << " | samples=" << pred_validation.validation_samples << std::endl;
            
            // Component-wise breakdown
            std::cout << "PREDICTION RMSE (wx):      0.5s=" << pred_validation.rmse_0_5s_wx 
                     << " | 1s=" << pred_validation.rmse_1s_wx 
                     << " | 2s=" << pred_validation.rmse_2s_wx << std::endl;
            std::cout << "PREDICTION RMSE (wy):      0.5s=" << pred_validation.rmse_0_5s_wy 
                     << " | 1s=" << pred_validation.rmse_1s_wy 
                     << " | 2s=" << pred_validation.rmse_2s_wy << std::endl;
            std::cout << "PREDICTION RMSE (wpsi):    0.5s=" << pred_validation.rmse_0_5s_wpsi 
                     << " | 1s=" << pred_validation.rmse_1s_wpsi 
                     << " | 2s=" << pred_validation.rmse_2s_wpsi << std::endl;
            
            // Show current environmental state vs predictions (still in body frame for clarity)
            Vector3d current_env(esti_x[6], esti_x[7], esti_x[8]);
            Vector3d pred_1s = predictWithDecay(1.0);
            Vector3d pred_2s = predictWithDecay(2.0);
            
            std::cout << "ENV FORCES (body): current=(" << std::setprecision(1) << current_env.x() 
                     << "," << current_env.y() << "," << current_env.z() << ")" << std::endl;
            std::cout << "                   pred_1s=(" << pred_1s.x() << "," << pred_1s.y() 
                     << "," << pred_1s.z() << ") | pred_2s=(" << pred_2s.x() 
                     << "," << pred_2s.y() << "," << pred_2s.z() << ")" << std::endl;
        } else {
            std::cout << "COMPONENT-WISE PREDICTION: warming up (" << pred_validation.predicted_0_5s_body.size() 
                     << "/40 samples needed)" << std::endl;
        }
        std::cout << fault_color << "FAULT STATUS: " << fault_status;
        if (fault_detected) {
            std::cout << " (Thruster degrad with: " <<  thruster_degrade_percentage * 100.0 << "%)";
        }
        std::cout << "\033[0m" << std::endl; // Reset color
        std::cout << fault_color << "FAULT STATUS: " << fault_status;
        // if (fault_detected) {
        //     std::cout << " (Confidence: " << std::fixed << std::setprecision(2) << fault_detection_confidence * 100.0 << "%)";
        // }
        // std::cout << "\033[0m" << std::endl;
        
        // ENHANCED: Add planning status for adaptive mode
        if (current_mode == ADAPTIVE_ASSISTED_RETURN && current_plan.is_valid) {
            std::cout << "\033[35m" << "PLANNING STATUS: Distance=" << std::fixed << std::setprecision(1) 
                    << current_plan.path_distance << "m | EnvAlign=" << std::setprecision(2) 
                    << current_plan.environmental_alignment << " | Score=" << std::setprecision(1) 
                    << current_plan.feasibility_score << " | ObstacleFree=" 
                    << (current_plan.obstacle_free ? "YES" : "NO") << "\033[0m" << std::endl;
        }
        std::cout << "---------------------------------------------------------------------------------------------------------------------" << std::endl;
        cout_counter = 0;
    }
    else{
        cout_counter++;
    }
}

void WAMV_MPC::EKF()
{
    // Calculate actual applied forces considering degradation
    Vector2d actual_thrust_commands;
    if (iteration_count >= fault_trigger) {
        switch (FAULT_TYPE_TO_SIMULATE) {
            case LEFT_THRUSTER_FAULT_SIM:
                actual_thrust_commands << solver_param.Tp_pre * (1.0 - thruster_degrade_percentage), 
                                         solver_param.Ts_pre;
                break;
            case RIGHT_THRUSTER_FAULT_SIM:
                actual_thrust_commands << solver_param.Tp_pre, 
                                         solver_param.Ts_pre * (1.0 - thruster_degrade_percentage);
                break;
            default:
                actual_thrust_commands << solver_param.Tp_pre, solver_param.Ts_pre;
        }adaptMPCWeights();
    } else {
        actual_thrust_commands << solver_param.Tp_pre, solver_param.Ts_pre;
    }
    pre_ekf_pos.u = esti_x[3];
    pre_ekf_pos.v = esti_x[4];
    pre_ekf_pos.r = esti_x[5];
    // get input u and measuremnet y
    meas_u = actual_thrust_commands;
    tau << meas_u[0] + meas_u[1], 0, -B/2*meas_u[0]+B/2*meas_u[1];
    
    // if two fixed direction thrusters
    tau << meas_u[0] + meas_u[1], 0, -B/2*meas_u[0]+B/2*meas_u[1];
    
            // meas_y << local_pos.x, local_pos.y, local_pos.psi,
    //         local_pos.u, local_pos.v, local_pos.r,
    //         tau(0),tau(1),tau(2);
    // Define Jacobian matrices of system dynamics and measurement model
    Matrix<double,9,9> F;     // Jacobian of system dynamics
    Matrix<double,9,9> H;     // Jacobian of measurement model

    // Define Kalman gain matrix
    Matrix<double,9,9> Kal;

    // Define prediction and update steps
    Matrix<double,9,1> x_pred;     // predicted state
    Matrix<double,9,9> P_pred;    // predicted covariance
    Matrix<double,9,1> y_pred;     // predicted measurement
    Matrix<double,9,1> y_err;      // measurement error

    // Prediction step: estimate state and covariance at time k+1|k
    F = compute_jacobian_F(esti_x, tau);             // compute Jacobian of system dynamics at current state and input
    x_pred = RK4(esti_x, tau);                       // predict state at time k+1|k
    P_pred = F * esti_P * F.transpose() + noise_Q;      // predict covariance at time k+1|k
    
    ekf_acc.x = (x_pred[3] - pre_ekf_pos.u)/dt;
    ekf_acc.y = (x_pred[4] - pre_ekf_pos.v)/dt;
    ekf_acc.psi = (x_pred[5] - pre_ekf_pos.r)/dt;

    // Update step: correct state and covariance using measurement at time k+1
    if (imu_data_available || odom_data_available) {
        int num_measurements = (imu_data_available ? 4 : 0) + (odom_data_available ? 9 : 0);
        VectorXd meas_y_full(num_measurements);
        VectorXd y_pred_full(num_measurements);
        MatrixXd H_full(num_measurements, 9);
        MatrixXd R_full(num_measurements, num_measurements);

        int idx = 0;
        if (imu_data_available) {
            meas_y_full.segment(idx, 4) << imu_pos.psi, imu_pos.r, imu_acc.x, imu_acc.y;
            H_full.block(idx, 0, 4, 9) = compute_jacobian_H_imu(x_pred);
            R_full.block(idx, idx, 4, 4) = noise_R_imu;
            idx += 4;
            imu_data_available = false;
        }
        if (odom_data_available) {
            meas_y_full.segment(idx, 9) << local_pos.x, local_pos.y, local_pos.psi,
                                          local_pos.u, local_pos.v, local_pos.r,
                                          tau(0), tau(1), tau(2);
            H_full.block(idx, 0, 9, 9) = compute_jacobian_H(x_pred);
            R_full.block(idx, idx, 9, 9) = noise_R;
            idx += 9;
            odom_data_available = false;
        }

        if (num_measurements == 13) {
            y_pred_full << h_imu(x_pred), h(x_pred);
        } else if (num_measurements == 4) {
            y_pred_full = h_imu(x_pred);
        } else {
            y_pred_full = h(x_pred);
        }
        y_err = meas_y_full - y_pred_full;
            MatrixXd S = H_full * P_pred * H_full.transpose() + R_full;
            Kal = P_pred * H_full.transpose() * S.inverse();
            esti_x = x_pred + Kal * y_err;
            esti_P = (MatrixXd::Identity(9, 9) - Kal * H_full) * P_pred * 
                     (MatrixXd::Identity(9, 9) - Kal * H_full).transpose() + 
                     Kal * R_full * Kal.transpose();
        } else {
            esti_x = x_pred;
            esti_P = P_pred;
    }

    // publish ekf states
    tf2::Quaternion quat_ekf;
    quat_ekf.setRPY(0, 0, esti_x[2]);
    geometry_msgs::msg::Quaternion quat_ekf_msg;
    tf2::convert(quat_ekf, quat_ekf_msg);
    ekf_pose.pose.pose.position.x = esti_x[0];
    ekf_pose.pose.pose.position.y = esti_x[1];
    ekf_pose.pose.pose.orientation.x = quat_ekf_msg.x;
    ekf_pose.pose.pose.orientation.y = quat_ekf_msg.y;
    ekf_pose.pose.pose.orientation.z = quat_ekf_msg.z;
    ekf_pose.pose.pose.orientation.w = quat_ekf_msg.w;
    ekf_pose.twist.twist.linear.x = esti_x[3];
    ekf_pose.twist.twist.linear.y = esti_x[4];
    ekf_pose.twist.twist.angular.z = esti_x[5];
    ekf_pose.header.stamp = rclcpp::Clock().now();
    ekf_pose.header.frame_id = "odom_frame";
    ekf_pose.child_frame_id = "base_link";

    ekf_pose_pub->publish(ekf_pose);

    disturbance.header.stamp = rclcpp::Clock().now();
    disturbance.twist.linear.x = esti_x[6];
    disturbance.twist.linear.y = esti_x[7];
    disturbance.twist.angular.z = esti_x[8];
    disturbance_pub->publish(disturbance);
    
    // H = compute_jacobian_H(x_pred);                         // compute Jacobian of measurement model at predicted state
    // y_pred = h(x_pred);                                     // predict measurement at time k+1
    // y_err = meas_y - y_pred;                                // compute measurement error
    // Kal = P_pred * H.transpose() * (H * P_pred * H.transpose() + noise_R).inverse();    // compute Kalman gain
    // esti_x = x_pred + Kal * y_err;                          // correct state estimate
    // esti_P = (MatrixXd::Identity(n, n) - Kal * H) * P_pred * (MatrixXd::Identity(n, n) - Kal * H).transpose() + Kal*noise_R*Kal.transpose(); // correct covariance estimate
    
}

MatrixXd WAMV_MPC::RK4(MatrixXd x, MatrixXd u)
{
    Matrix<double,9,1> k1;
    Matrix<double,9,1> k2;
    Matrix<double,9,1> k3;
    Matrix<double,9,1> k4;

    k1 = f(x, u) * dt;
    k2 = f(x+k1/2, u) * dt;
    k3 = f(x+k2/2, u) * dt;
    k4 = f(x+k3, u) * dt;

    return x + (k1+2*k2+2*k3+k4)/6;
}

// Define system dynamics function
MatrixXd WAMV_MPC::f(MatrixXd x, MatrixXd u)
{
    // Define system dynamics
    Matrix<double,9,1> xdot;

    xdot << cos(x(2))*x(3) - sin(x(2))*x(4),
            sin(x(2))*x(3) + cos(x(2))*x(4),
            x(5),
            invM(0,0)*(u(0) + mass*x(4)*x(5) + xu*x(3) + xuu*abs(x(3))*x(3) + x(6)),
            invM(1,1)*(u(1) - mass*x(3)*x(5) + yv*x(4) + yvv*abs(x(4))*x(4) + x(7)),
            invM(2,2)*(u(2) + nr*x(5) + nrr*abs(x(5))*x(5) + x(8)),
            0,0,0;
            
    return xdot; // dt is the time step
}

// Define measurement model function (Z = Hx, Z: measurement vector [x,xdot,tau]; X: state vector [x,xdot,disturbance])
MatrixXd WAMV_MPC::h(MatrixXd x)
{
    // Define measurement model
    Matrix<double,9,1> y;

    y << x(0),x(1),x(2),
        x(3),x(4),x(5),
        M(0,0)*ekf_acc.x - mass*x(4)*x(5) - xu*x(3) - xuu*abs(x(3))*x(3) - x(6),
        M(1,1)*ekf_acc.y + mass*x(3)*x(5) - yv*x(4) - yvv*abs(x(4))*x(4) - x(7),
        M(2,2)*ekf_acc.psi - nr*x(5) - nrr*abs(x(5))*x(5) - x(8);

    return y;
}

MatrixXd WAMV_MPC::h_imu(MatrixXd x) {
    Matrix<double, 4, 1> y;
    y << x(2),
         x(5),
         invM(0,0) * (tau(0) + mass * x(4) * x(5) + xu * x(3) + xuu * abs(x(3)) * x(3) + x(6)),
         invM(1,1) * (tau(1) - mass * x(3) * x(5) + yv * x(4) + yvv * abs(x(4)) * x(4) + x(7));
    return y;
}

// Define function to compute Jacobian of system dynamics at current state and input
MatrixXd WAMV_MPC::compute_jacobian_F(MatrixXd x, MatrixXd u)
{
    // Define Jacobian of system dynamics
    Matrix<double,9,9> F;
    double d = 1e-6;                    // finite difference step size
    VectorXd f0 = RK4(x, u);
    for (int i = 0; i < n; i++){
        VectorXd x1 = x;
        x1(i) += d;
        VectorXd f1 = RK4(x1, u);
        F.col(i) = (f1-f0)/d;
    }
    return F;
}

// Define function to compute Jacobian of measurement model at predicted state
MatrixXd WAMV_MPC::compute_jacobian_H(MatrixXd x)
{
    // Define Jacobian of measurement model
    Matrix<double,9,9> H;
    double d = 1e-6;                    // finite difference step size
    VectorXd f0 = h(x);
    for (int i = 0; i < n; i++){
        VectorXd x1 = x;
        x1(i) += d;
        VectorXd f1 = h(x1);
        H.col(i) = (f1-f0)/d;
    }
    return H;
}

MatrixXd WAMV_MPC::compute_jacobian_H_imu(MatrixXd x) {
    Matrix<double, 4, 9> H;
    double d = 1e-6;
    VectorXd f0 = h_imu(x);
    for (int i = 0; i < 9; i++) {
        VectorXd x1 = x;
        x1(i) += d;
        VectorXd f1 = h_imu(x1);
        H.col(i) = (f1 - f0) / d;
    }
    return H;
}


void WAMV_MPC::initializeHarborZones() {
    // Initialize harbor zones with corrected coordinates
    harbor_zones.clear();
    harbor_zones.resize(3);
    
    // Harbor Zone 1: [-580, 258], [-575, 240], [-600, 236], [-600, 248]
    harbor_zones[0].vertices = {
        Vector2d(-580, 258), Vector2d(-575, 240), 
        Vector2d(-600, 236), Vector2d(-600, 248)
    };
    
    // Harbor Zone 2: [-575, 222], [-575, 208], [-595, 208], [-595, 220]
    harbor_zones[1].vertices = {
        Vector2d(-575, 222), Vector2d(-575, 208),
        Vector2d(-595, 208), Vector2d(-595, 220)
    };
    
    // Harbor Zone 3: [-573, 192], [-593, 191], [-593, 183], [-584, 184]
    harbor_zones[2].vertices = {
        Vector2d(-573, 192), Vector2d(-593, 191),
        Vector2d(-593, 183), Vector2d(-584, 184)
    };
    
    // Calculate centers for each zone
    for (auto& zone : harbor_zones) {
        Vector2d center_sum(0, 0);
        for (const auto& vertex : zone.vertices) {
            center_sum += vertex;
        }
        zone.center = center_sum / zone.vertices.size();
    }
    
    // Initialize dock areas (obstacles to avoid)
    dock_areas.clear();
    dock_areas.resize(2);
    
    // Dock 1: [-575, 240], [-575, 222], [-595, 220], [-600, 236]
    dock_areas[0] = {
        Vector2d(-575, 240), Vector2d(-575, 222),
        Vector2d(-595, 220), Vector2d(-600, 236)
    };
    
    // Dock 2: [-575, 208], [-573, 192], [-593, 191], [-595, 208]
    dock_areas[1] = {
        Vector2d(-575, 208), Vector2d(-573, 192),
        Vector2d(-593, 191), Vector2d(-595, 208)
    };
    
    // Initialize planning variables
    last_replan_time = 0.0;
    current_plan.is_valid = false;
    
    RCLCPP_INFO(this->get_logger(), "Harbor zones initialized successfully");
}

bool WAMV_MPC::pointInPolygon(const Vector2d& point, const std::vector<Vector2d>& polygon) {
    bool inside = false;
    int n = polygon.size();
    
    for (int i = 0, j = n - 1; i < n; j = i++) {
        if (((polygon[i].y() > point.y()) != (polygon[j].y() > point.y())) &&
            (point.x() < (polygon[j].x() - polygon[i].x()) * (point.y() - polygon[i].y()) / 
             (polygon[j].y() - polygon[i].y()) + polygon[i].x())) {
            inside = !inside;
        }
    }
    return inside;
}

bool WAMV_MPC::lineIntersectsPolygon(const Vector2d& start, const Vector2d& end, 
                                    const std::vector<Vector2d>& polygon) {
    // Check if line segment intersects with any edge of the polygon
    int n = polygon.size();
    
    for (int i = 0; i < n; i++) {
        Vector2d p1 = polygon[i];
        Vector2d p2 = polygon[(i + 1) % n];
        
        // Line segment intersection check using cross products
        Vector2d dir1 = end - start;
        Vector2d dir2 = p2 - p1;
        Vector2d diff = start - p1;
        
        double cross = dir1.x() * dir2.y() - dir1.y() * dir2.x();
        
        if (std::abs(cross) > 1e-6) {  // Lines are not parallel
            double t = (diff.x() * dir2.y() - diff.y() * dir2.x()) / cross;
            double u = (diff.x() * dir1.y() - diff.y() * dir1.x()) / cross;
            
            if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
                return true;  // Intersection found
            }
        }
    }
    return false;
}

bool WAMV_MPC::isAboveBoundaryLine(const Vector2d& point) {
    // Upper boundary line: [-580, 258], [-600, 248]
    // Avoid area: y > line
    double m = (248 - 258) / (-600 - (-580));  // slope = 0.5
    double b = 258 - m * (-580);               // y-intercept
    double line_y = m * point.x() + b;
    return point.y() > line_y;
}

bool WAMV_MPC::isBelowBoundaryLine(const Vector2d& point) {
    // Lower boundary line: [-584, 184], [-593, 183]
    // Avoid area: y < line
    double m = (183 - 184) / (-593 - (-584));  // slope = 1/9
    double b = 184 - m * (-584);               // y-intercept
    double line_y = m * point.x() + b;
    return point.y() < line_y;
}

bool WAMV_MPC::isPathObstacleFree(const Vector2d& start, const Vector2d& end) {
    // Check intersection with dock areas
    for (const auto& dock : dock_areas) {
        if (lineIntersectsPolygon(start, end, dock)) {
            return false;
        }
    }
    
    // Check boundary line violations
    // Sample points along the path
    int num_samples = 10;
    for (int i = 0; i <= num_samples; i++) {
        double t = static_cast<double>(i) / num_samples;
        Vector2d sample_point = start + t * (end - start);
        
        if (isAboveBoundaryLine(sample_point) || isBelowBoundaryLine(sample_point)) {
            return false;
        }
    }
    
    return true;
}

double WAMV_MPC::calculateEnvironmentalAlignment(const Vector2d& path_direction) {
    // Get current environmental forces in body frame
    Vector3d env_forces_body(esti_x[6], esti_x[7], esti_x[8]);
    
    // Transform to inertial frame using current heading
    double current_psi = local_pos.psi;  // or yaw_sum for continuous
    Matrix2d R_body_to_inertial;
    R_body_to_inertial << cos(current_psi), -sin(current_psi),
                          sin(current_psi),  cos(current_psi);
    
    Vector2d env_force_body_2d(env_forces_body.x(), env_forces_body.y());
    Vector2d env_force_inertial = R_body_to_inertial * env_force_body_2d;
    
    // Handle zero force case
    if (env_force_inertial.norm() < 0.1) {
        return 0.0;  // Neutral alignment
    }
    
    // Now both vectors are in inertial frame
    Vector2d normalized_path = path_direction.normalized();
    Vector2d normalized_env = env_force_inertial.normalized();
    
    double alignment = normalized_path.dot(normalized_env);
    
    // Yaw moment assistance (w_psi is already scalar, no frame transformation needed)
    double yaw_assistance = 0.0;
    if (std::abs(env_forces_body.z()) > 0.1) {
        double current_heading = local_pos.psi;
        double desired_heading = atan2(path_direction.y(), path_direction.x());
        double heading_error = desired_heading - current_heading;
        
        // Normalize heading error to [-pi, pi]
        while (heading_error > M_PI) heading_error -= 2.0 * M_PI;
        while (heading_error < -M_PI) heading_error += 2.0 * M_PI;
        
        // Check if environmental yaw moment helps reduce heading error
        if (heading_error * env_forces_body.z() > 0) {
            yaw_assistance = 0.2;  // Bonus for yaw assistance
        }
    }
    
    return std::max(-1.0, std::min(1.0, alignment + yaw_assistance));
}

void WAMV_MPC::fastPlanning() {
    // Reset current plan
    current_plan.is_valid = false;
    current_plan.feasibility_score = -999.0;
    current_plan.selected_harbor_zone = -1;
    
    Vector2d current_position(local_pos.x, local_pos.y);
    double current_heading = local_pos.psi;
    
    // Check if we're already very close to any harbor zone
    for (size_t zone_idx = 0; zone_idx < harbor_zones.size(); zone_idx++) {
        const HarborZone& zone = harbor_zones[zone_idx];
        double distance = (zone.center - current_position).norm();
        
        // If very close to a zone, just target that zone
        if (distance < 20.0) {
            current_plan.selected_harbor_zone = zone_idx;
            current_plan.target_point = zone.center;
            current_plan.path_distance = distance;
            current_plan.obstacle_free = true;
            current_plan.feasibility_score = 200.0; // High score for close target
            current_plan.is_valid = true;
            
            RCLCPP_INFO(this->get_logger(), 
                       "Close proximity planning: Zone %zu, Distance %.1f m", 
                       zone_idx, distance);
            return;
        }
    }
    
    // Original planning logic for farther distances
    // Evaluate each harbor zone
    for (int zone_idx = 0; zone_idx < 3; zone_idx++) {
        const HarborZone& zone = harbor_zones[zone_idx];
        
        // Calculate path to zone center
        Vector2d path_vector = zone.center - current_position;
        double distance = path_vector.norm();
        
        // Skip if zone is too close (already inside)
        if (distance < 5.0) {
            continue;
        }
        
        Vector2d path_direction = path_vector.normalized();
        double desired_heading = atan2(path_direction.y(), path_direction.x());
        double heading_change = std::abs(desired_heading - current_heading);
        
        // Normalize heading change to [0, pi]
        if (heading_change > M_PI) {
            heading_change = 2.0 * M_PI - heading_change;
        }
        
        // Check if path is obstacle-free
        bool obstacle_free = isPathObstacleFree(current_position, zone.center);
        
        // Calculate environmental alignment
        double env_alignment = calculateEnvironmentalAlignment(path_direction);
        
        // Calculate feasibility score
        double score = 0.0;
        
        // Distance penalty (closer is better)
        score += 1000.0 / (distance + 10.0);  // Max ~100 points
        
        // Environmental assistance bonus
        score += env_alignment * 50.0;  // ±50 points
        
        // Heading change penalty
        score -= heading_change * 30.0 / M_PI;  // 0-30 point penalty
        
        // Obstacle bonus/penalty
        if (obstacle_free) {
            score += 100.0;  // Major bonus for clear path
        } else {
            score -= 200.0;  // Major penalty for blocked path
            continue;  // Skip blocked paths
        }
        
        // Zone preference (middle zones might be better)
        if (zone_idx == 1) score += 10.0;  // Slight preference for middle zone
        
        // Update current_plan if this is the best option so far
        if (score > current_plan.feasibility_score) {
            current_plan.selected_harbor_zone = zone_idx;
            current_plan.target_point = zone.center;
            current_plan.path_distance = distance;
            current_plan.required_heading_change = heading_change;
            current_plan.environmental_alignment = env_alignment;
            current_plan.obstacle_free = obstacle_free;
            current_plan.feasibility_score = score;
            current_plan.is_valid = true;
        }
    }
    
    // Log planning result
    if (current_plan.is_valid) {
        RCLCPP_INFO(this->get_logger(), 
                   "Planning: Zone %d, Distance %.1f, EnvAlign %.2f, Score %.1f",
                   current_plan.selected_harbor_zone, current_plan.path_distance, 
                   current_plan.environmental_alignment, current_plan.feasibility_score);
    } else {
        RCLCPP_WARN(this->get_logger(), "No feasible path to any harbor zone found!");
    }
}

bool WAMV_MPC::needsReplanning() {
    static int previous_fault_type = NO_FAULT;
    static Vector3d previous_env_direction(0, 0, 0);
    
    double current_time = rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds() - start_time;
    
    // Always replan if no valid plan exists
    if (!current_plan.is_valid) {
        return true;
    }
    
    // Check minimum replan interval
    if (current_time - last_replan_time < MIN_REPLAN_INTERVAL) {
        return false;
    }
    
    // Check for fault status change
    if (current_fault_type != previous_fault_type) {
        previous_fault_type = current_fault_type;
        RCLCPP_INFO(this->get_logger(), "Replanning due to fault status change");
        return true;
    }
    
    // Check for significant environmental force change
    Vector3d current_env_force(esti_x[6], esti_x[7], esti_x[8]);
    if (current_env_force.norm() > 1.0 && previous_env_direction.norm() > 1.0) {
        Vector3d current_dir = current_env_force.normalized();
        Vector3d prev_dir = previous_env_direction.normalized();
        double angle_change = acos(std::max(-1.0, std::min(1.0, current_dir.dot(prev_dir))));
        
        if (angle_change > M_PI / 6) {  // 30 degrees
            previous_env_direction = current_env_force;
            RCLCPP_INFO(this->get_logger(), "Replanning due to environmental force change");
            return true;
        }
    } else {
        previous_env_direction = current_env_force;
    }
    
    // Check if approaching target (within 20m)
    Vector2d current_pos(local_pos.x, local_pos.y);
    double distance_to_target = (current_plan.target_point - current_pos).norm();
    if (distance_to_target < 20.0) {
        RCLCPP_INFO(this->get_logger(), "Replanning due to target proximity");
        return true;
    }
    
    return false;
}

void WAMV_MPC::updatePlanningAndReference() {
    // Check if replanning is needed
    if (needsReplanning()) {
        fastPlanning();  // Now void function - updates current_plan directly
        last_replan_time = rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds() - start_time;
        
        if (current_plan.is_valid) {
            // Update MPC reference trajectory to target point
            // This will be used in your existing ref_cb function
            RCLCPP_INFO(this->get_logger(), 
                       "New plan: Target (%.1f, %.1f), Zone %d", 
                       current_plan.target_point.x(), current_plan.target_point.y(),
                       current_plan.selected_harbor_zone);
        }
    }
}

void WAMV_MPC::initializeOperationalMode() {
    current_mode = FOLLOW_PRESET_TRAJECTORY;
    trajectory_generation_active = false;
    generated_line_number = 0;
    generated_trajectory.clear();
    
    RCLCPP_INFO(this->get_logger(), "Operational mode initialized: FOLLOW_PRESET_TRAJECTORY");
}

void WAMV_MPC::generateStationKeepingTrajectory() {
    // Clear existing generated trajectory
    generated_trajectory.clear();
    
    // Get current position as station keeping target
    double target_x = local_pos.x;
    double target_y = local_pos.y;
    double target_psi = local_pos.psi;
    
    // Generate stationary trajectory (same format as your .txt file)
    // Format: [x, y, psi, u, v, r, Tp, Ts] - 8 columns
    int num_points = 500;  // 25 seconds at 20Hz (same as your script generates)
    
    for (int i = 0; i < num_points; i++) {
        std::vector<double> waypoint(8);
        waypoint[0] = target_x;     // x - keep current position
        waypoint[1] = target_y;     // y - keep current position  
        waypoint[2] = target_psi;   // psi - keep current heading
        waypoint[3] = 0.0;          // u - zero forward velocity
        waypoint[4] = 0.0;          // v - zero lateral velocity
        waypoint[5] = 0.0;          // r - zero angular velocity
        waypoint[6] = 0.0;          // Tp - let MPC determine
        waypoint[7] = 0.0;          // Ts - let MPC determine
        
        generated_trajectory.push_back(waypoint);
    }
    
    generated_line_number = 0;
    trajectory_generation_active = true;
    
    RCLCPP_INFO(this->get_logger(), 
                "Generated station keeping trajectory at (%.2f, %.2f, %.2f)", 
                target_x, target_y, target_psi);
}

void WAMV_MPC::generateAdaptiveReturnTrajectory() {
    // Clear existing generated trajectory
    generated_trajectory.clear();
    
    if (!current_plan.is_valid) {
        RCLCPP_WARN(this->get_logger(), "No valid plan available for trajectory generation");
        return;
    }
    
    // Current position and target
    Vector2d current_pos(local_pos.x, local_pos.y);
    Vector2d target_pos = current_plan.target_point;
    // double current_psi = local_pos.psi;
    
    // Calculate trajectory parameters
    double total_distance = (target_pos - current_pos).norm();
    double target_heading_bounded = atan2(target_pos.y() - current_pos.y(), 
                                         target_pos.x() - current_pos.x());
    
    // CRITICAL: Convert to continuous form to match yaw_sum
    double target_heading_continuous = convertToContinuousPsi(target_heading_bounded, yaw_sum);
    
    RCLCPP_INFO(this->get_logger(), "TRAJ_GEN: target_heading_bounded=%.3f, yaw_sum=%.3f, target_heading_continuous=%.3f", 
               target_heading_bounded, yaw_sum, target_heading_continuous);
    
    // Generate trajectory with same structure as your mission_traj.py
    double sample_time = 0.05;  // 20Hz to match your script
    double cruise_speed = 1.5;  // m/s - conservative speed for fault condition
    double approach_speed = 0.8; // m/s - slower for final approach
    
    // Estimate total time needed
    // double estimated_time = total_distance / cruise_speed + 10.0; // +10s buffer
    // int num_points = static_cast<int>(estimated_time / sample_time);
    
    // Phase 1: Heading adjustment (if needed)
    double heading_error = target_heading_continuous - yaw_sum;  // Now both are continuous!
    // No need for angle wrapping since both are continuous
    
    double heading_adjust_time = std::abs(heading_error) / 0.3; // 0.3 rad/s turn rate
    int heading_adjust_points = static_cast<int>(heading_adjust_time / sample_time);
    
    // Phase 2: Approach to target
    double approach_time = total_distance / cruise_speed;
    int approach_points = static_cast<int>(approach_time / sample_time);
    
    // Phase 3: Final positioning (last 20m at slow speed)
    double final_approach_distance = std::min(20.0, total_distance * 0.3);
    double final_approach_time = final_approach_distance / approach_speed;
    int final_points = static_cast<int>(final_approach_time / sample_time);
    
    // DEBUG: Print all phase information
    RCLCPP_INFO(this->get_logger(), "TRAJ_GEN: Phases - adjust:%d, approach:%d, final:%d, total:%d", 
               heading_adjust_points, approach_points, final_points, 
               heading_adjust_points + approach_points + final_points + 20);
    
    // Generate trajectory points
    Vector2d current_trajectory_pos = current_pos;
    double current_trajectory_psi = yaw_sum;  // Start from current continuous psi
    
    // Phase 1: Heading adjustment
    for (int i = 0; i < heading_adjust_points; i++) {
        std::vector<double> waypoint(8);
        
        double t = static_cast<double>(i) / std::max(heading_adjust_points, 1);
        double smooth_factor = 3 * t * t - 2 * t * t * t; // Smooth S-curve
        
        // Gradually adjust heading - CONTINUOUS
        current_trajectory_psi = yaw_sum + heading_error * smooth_factor;
        
        // Move forward during heading adjustment
        double slow_forward = std::max(0.3, cruise_speed * 0.3 * smooth_factor);
        current_trajectory_pos += Vector2d(slow_forward * cos(current_trajectory_psi) * sample_time,
                                          slow_forward * sin(current_trajectory_psi) * sample_time);
        
        waypoint[0] = current_trajectory_pos.x();
        waypoint[1] = current_trajectory_pos.y();
        waypoint[2] = current_trajectory_psi;  // CONTINUOUS PSI
        waypoint[3] = slow_forward;  // u
        waypoint[4] = 0.0;           // v  
        waypoint[5] = heading_error / std::max(heading_adjust_time, 0.1); // r
        waypoint[6] = 0.0;           // Tp
        waypoint[7] = 0.0;           // Ts
        
        generated_trajectory.push_back(waypoint);
    }
    
    // Phase 2: Main approach
    Vector2d direction = (target_pos - current_trajectory_pos).normalized();
    
    for (int i = 0; i < approach_points; i++) {
        std::vector<double> waypoint(8);
        
        double progress = static_cast<double>(i) / std::max(approach_points, 1);
        double current_speed = std::max(0.5, cruise_speed);
        
        // Slow down as we approach target
        if (progress > 0.7) {
            double slowdown_factor = 1.0 - (progress - 0.7) / 0.3 * 0.6;
            current_speed = std::max(0.3, cruise_speed * slowdown_factor);
        }
        
        current_trajectory_pos += direction * current_speed * sample_time;
        
        waypoint[0] = current_trajectory_pos.x();
        waypoint[1] = current_trajectory_pos.y(); 
        waypoint[2] = target_heading_continuous;  // CONTINUOUS PSI
        waypoint[3] = current_speed;  // u
        waypoint[4] = 0.0;           // v
        waypoint[5] = 0.0;           // r
        waypoint[6] = 0.0;           // Tp
        waypoint[7] = 0.0;           // Ts
        
        generated_trajectory.push_back(waypoint);
    }
    
    // Phase 3: Final approach and positioning
    for (int i = 0; i < final_points + 20; i++) {
        std::vector<double> waypoint(8);
        
        if (i < final_points) {
            // Still approaching
            double remaining_dist = (target_pos - current_trajectory_pos).norm();
            if (remaining_dist > 0.5) {
                Vector2d final_direction = (target_pos - current_trajectory_pos).normalized();
                current_trajectory_pos += final_direction * approach_speed * sample_time;
            } else {
                current_trajectory_pos = target_pos;
            }
            
            waypoint[0] = current_trajectory_pos.x();
            waypoint[1] = current_trajectory_pos.y();
            waypoint[2] = target_heading_continuous;  // CONTINUOUS PSI
            waypoint[3] = std::max(0.2, approach_speed * 0.5); // Minimum speed
            waypoint[4] = 0.0;
            waypoint[5] = 0.0;
        } else {
            // Final hold at target
            waypoint[0] = target_pos.x();
            waypoint[1] = target_pos.y();
            waypoint[2] = target_heading_continuous;  // CONTINUOUS PSI
            waypoint[3] = 0.1; // Small forward velocity for control authority
            waypoint[4] = 0.0;
            waypoint[5] = 0.0;
        }
        
        waypoint[6] = 0.0; // Tp
        waypoint[7] = 0.0; // Ts
        
        generated_trajectory.push_back(waypoint);
    }
    
    generated_line_number = 0;
    trajectory_generation_active = true;
    
    // DEBUG: Print first few trajectory points
    if (!generated_trajectory.empty()) {
        for (int i = 0; i < std::min(3, (int)generated_trajectory.size()); i++) {
            const auto& pt = generated_trajectory[i];
            RCLCPP_INFO(this->get_logger(), "TRAJ_GEN: Point[%d]: pos(%.2f,%.2f), psi=%.3f, u=%.3f, v=%.3f, r=%.3f", 
                       i, pt[0], pt[1], pt[2], pt[3], pt[4], pt[5]);
        }
    }
    
    RCLCPP_INFO(this->get_logger(), 
                "Generated adaptive return trajectory: %zu points, target (%.2f, %.2f), continuous_psi=%.3f",
                generated_trajectory.size(), target_pos.x(), target_pos.y(), target_heading_continuous);
}

void WAMV_MPC::updateOperationalMode() {
    OperationalMode previous_mode = current_mode;
    
    // Check if mission is already completed
    if (mission_completed) {
        current_mode = STATION_KEEPING;
        return;
    }
    
    // Mode switching logic based on fault status and iteration count
    if (iteration_count < fault_trigger) {
        // Before fault trigger - always follow preset trajectory
        current_mode = FOLLOW_PRESET_TRAJECTORY;
        trajectory_generation_active = false;
    } else {
        // After fault trigger - check if we've arrived at harbor zone
        if (current_mode == ADAPTIVE_ASSISTED_RETURN && hasArrivedAtHarborZone()) {
            // Mission completed! Switch to station keeping
            mission_completed = true;
            current_mode = STATION_KEEPING;
            arrival_time = rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds() - start_time;
            
            // Log successful arrival
            RCLCPP_INFO(this->get_logger(), "🎉 SUCCESS: USV has arrived at harbor zone %d!", current_plan.selected_harbor_zone);
            RCLCPP_INFO(this->get_logger(), "📍 Final position: (%.2f, %.2f)", local_pos.x, local_pos.y);
            RCLCPP_INFO(this->get_logger(), "⏱️  Mission completion time: %.1f seconds", arrival_time);
            RCLCPP_INFO(this->get_logger(), "🔄 Switching to STATION_KEEPING mode");
            
            // Generate station keeping trajectory at current position
            generateStationKeepingTrajectory();
            return;
        }
        
        // Continue with adaptive return mode
        current_mode = ADAPTIVE_ASSISTED_RETURN;
        
        // Enable fast planning when switching to adaptive mode
        if (previous_mode != ADAPTIVE_ASSISTED_RETURN) {
            // First time switching to adaptive mode
            fastPlanning(); // Generate initial plan
            if (current_plan.is_valid) {
                generateAdaptiveReturnTrajectory();
                RCLCPP_INFO(this->get_logger(), "Switched to ADAPTIVE_ASSISTED_RETURN mode");
            } else {
                // Fallback to station keeping if no valid plan
                current_mode = STATION_KEEPING;
                generateStationKeepingTrajectory();
                RCLCPP_INFO(this->get_logger(), "No valid plan - switched to STATION_KEEPING mode");
            }
        }
    }
    
    // Handle mode-specific updates (only if not completed)
    if (current_mode == ADAPTIVE_ASSISTED_RETURN && !mission_completed) {
        // Update planning and regenerate trajectory if needed
        updatePlanningAndReference();
        
        // Regenerate trajectory if plan changed
        static int last_selected_zone = -1;
        if (current_plan.is_valid && current_plan.selected_harbor_zone != last_selected_zone) {
            generateAdaptiveReturnTrajectory();
            last_selected_zone = current_plan.selected_harbor_zone;
        }
    }
}

// In ref_cb_enhanced, add safety checks:
void WAMV_MPC::ref_cb_enhanced(int line_to_read) {
    switch (current_mode) {
        case FOLLOW_PRESET_TRAJECTORY:
            ref_cb(line_to_read);
            break;
            
        case STATION_KEEPING:
        case ADAPTIVE_ASSISTED_RETURN:
            if (trajectory_generation_active && !generated_trajectory.empty()) {
                
                // SAFETY CHECK: Ensure we have valid trajectory data
                if (generated_line_number < 0 || generated_line_number >= static_cast<int>(generated_trajectory.size())) {
                    RCLCPP_WARN(this->get_logger(), "Invalid trajectory line number %d, resetting to 0", generated_line_number);
                    generated_line_number = 0;
                }
                
                // SAFETY CHECK: Ensure trajectory points have correct size
                if (generated_trajectory[0].size() < 8) {
                    RCLCPP_ERROR(this->get_logger(), "Generated trajectory points have wrong size: %zu", generated_trajectory[0].size());
                    // Fall back to station keeping
                    for (unsigned int i = 0; i <= WAMV_N; i++) {
                        acados_in.yref[i][0] = local_pos.x;
                        acados_in.yref[i][1] = local_pos.y;
                        acados_in.yref[i][2] = yaw_sum;
                        acados_in.yref[i][3] = 0.0; // u
                        acados_in.yref[i][4] = 0.0; // v
                        acados_in.yref[i][5] = 0.0; // r
                        acados_in.yref[i][6] = 0.0; // Tp
                        acados_in.yref[i][7] = 0.0; // Ts
                    }
                    return;
                }
                
                // Fill MPC horizon with generated trajectory
                for (unsigned int i = 0; i <= WAMV_N; i++) {
                    int traj_index = generated_line_number + i;
                    
                    if (traj_index < static_cast<int>(generated_trajectory.size())) {
                        // Copy all 8 values safely
                        for (unsigned int j = 0; j < WAMV_NY && j < generated_trajectory[traj_index].size(); j++) {
                            acados_in.yref[i][j] = generated_trajectory[traj_index][j];
                        }
                    } else {
                        // Use last trajectory point
                        const auto& last_point = generated_trajectory.back();
                        for (unsigned int j = 0; j < WAMV_NY && j < last_point.size(); j++) {
                            acados_in.yref[i][j] = last_point[j];
                        }
                    }
                }
                
                // Advance trajectory position
                generated_line_number++;
                if (generated_line_number >= static_cast<int>(generated_trajectory.size())) {
                    generated_line_number = static_cast<int>(generated_trajectory.size()) - 1;
                }
                
            } else {
                // This fallback should work - it's essentially station keeping
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                     "No trajectory available, using station keeping");
                for (unsigned int i = 0; i <= WAMV_N; i++) {
                    acados_in.yref[i][0] = local_pos.x;
                    acados_in.yref[i][1] = local_pos.y;
                    acados_in.yref[i][2] = yaw_sum;
                    acados_in.yref[i][3] = 0.0;
                    acados_in.yref[i][4] = 0.0;
                    acados_in.yref[i][5] = 0.0;
                    acados_in.yref[i][6] = 0.0;
                    acados_in.yref[i][7] = 0.0;
                }
            }
            break;
            
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown operational mode: %d", current_mode);
            ref_cb(line_to_read);
            break;
    }
}

double WAMV_MPC::convertToContinuousPsi(double target_heading_bounded, double current_continuous_psi) {
    // Find the equivalent continuous heading closest to current_continuous_psi
    
    // Calculate how many full rotations the vehicle has made
    double full_rotations = floor(current_continuous_psi / (2.0 * M_PI));
    
    // Start with the bounded heading in the same "rotation level"
    double continuous_target = target_heading_bounded + full_rotations * 2.0 * M_PI;
    
    // Check if we should be in the next or previous rotation level
    double error1 = fabs(continuous_target - current_continuous_psi);
    double error2 = fabs(continuous_target + 2.0 * M_PI - current_continuous_psi);
    double error3 = fabs(continuous_target - 2.0 * M_PI - current_continuous_psi);
    
    if (error2 < error1 && error2 < error3) {
        continuous_target += 2.0 * M_PI;
    } else if (error3 < error1 && error3 < error2) {
        continuous_target -= 2.0 * M_PI;
    }
    
    return continuous_target;
}

// check if USV has arrived at harbor zone:
bool WAMV_MPC::hasArrivedAtHarborZone() {
    Vector2d current_pos(local_pos.x, local_pos.y);
    
    // SUCCESS CRITERIA: USV is inside ANY harbor zone, regardless of which one was planned
    for (size_t zone_idx = 0; zone_idx < harbor_zones.size(); zone_idx++) {
        if (pointInPolygon(current_pos, harbor_zones[zone_idx].vertices)) {
            RCLCPP_INFO(this->get_logger(), 
                       "SUCCESS: USV arrived at harbor zone %zu (planned target was zone %d)!", 
                       zone_idx, current_plan.selected_harbor_zone);
            return true;
        }
    }
    
    // Fallback: Check if close to ANY zone center
    for (size_t zone_idx = 0; zone_idx < harbor_zones.size(); zone_idx++) {
        double distance_to_zone = (harbor_zones[zone_idx].center - current_pos).norm();
        if (distance_to_zone <= ARRIVAL_DISTANCE_THRESHOLD) {
            RCLCPP_INFO(this->get_logger(), 
                       "SUCCESS: USV within %.1fm of harbor zone %zu!", 
                       ARRIVAL_DISTANCE_THRESHOLD, zone_idx);
            return true;
        }
    }
    
    return false;
}

void WAMV_MPC::updateEnvironmentalAssistance()
{
    // Update current environmental forces from EKF
    environmental_assistance.current_forces = Vector3d(esti_x[6], esti_x[7], esti_x[8]);
    
    // Simple reliability check
    double force_magnitude = environmental_assistance.current_forces.norm();
    environmental_assistance.is_reliable = (force_magnitude < 100.0) && (force_magnitude > 0.5);
    environmental_assistance.assistance_capability = std::min(1.0, force_magnitude / 30.0);
    
    // FAULT-ONLY assistance (your key innovation)
    if (iteration_count >= fault_trigger && environmental_assistance.is_reliable) {
        // After fault - enable environmental assistance
        environmental_assistance.surge_assistance_factor = 0.6;
        environmental_assistance.sway_assistance_factor = 0.6;
        environmental_assistance.yaw_assistance_factor = 0.4;
    } else {
        // Normal operation - NO environmental assistance
        environmental_assistance.surge_assistance_factor = 0.0;
        environmental_assistance.sway_assistance_factor = 0.0;
        environmental_assistance.yaw_assistance_factor = 0.0;
    }
}

void WAMV_MPC::adaptMPCWeights() {
    if (iteration_count >= fault_trigger && adaptive_weights.use_environmental_assistance) {
        // Increase psi weight during fault conditions
        double adaptive_psi_weight = 300.0;  // Increased from original 150
        double adaptive_u_weight = 5.0;      // Slightly increase velocity weights
        double adaptive_v_weight = 5.0;
        
        // Update cost weights in ACADOS
        double new_W_x[6] = {80, 10, adaptive_psi_weight, adaptive_u_weight, adaptive_v_weight, 5};
        
        // Apply new weights to all horizon points
        for (int i = 0; i <= WAMV_N; i++) {
            ocp_nlp_cost_model_set(mpc_capsule->nlp_config, mpc_capsule->nlp_dims, 
                                  mpc_capsule->nlp_in, i, "W", new_W_x);
        }
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Adaptive weights active: psi_weight=%.1f", adaptive_psi_weight);
    }
}

void WAMV_MPC::initializeTrendPrediction() {
    prev_forces_initialized = false;
    prev_env_forces.setZero();
    prediction_dt = 0.05;  // 20Hz control rate
    
    // Clear validation data
    pred_validation = PredictionValidation();
    
    RCLCPP_INFO(this->get_logger(), "Trend-aware environmental prediction initialized");
}

Vector3d WAMV_MPC::predictWithDecay(double prediction_time_seconds) {
    Vector3d current_forces_body(esti_x[6], esti_x[7], esti_x[8]);
    double current_psi = local_pos.psi;
    
    // Transform current body frame forces to inertial frame
    Matrix2d R_body_to_inertial;
    R_body_to_inertial << cos(current_psi), -sin(current_psi),
                          sin(current_psi),  cos(current_psi);
    
    Vector2d forces_xy_body(current_forces_body.x(), current_forces_body.y());
    Vector2d forces_xy_inertial = R_body_to_inertial * forces_xy_body;
    
    // Apply exponential decay in inertial frame (environmental forces are consistent in inertial frame)
    double decay_factor = exp(-prediction_time_seconds / 4.0);  // 4-second decay constant
    Vector2d predicted_xy_inertial = forces_xy_inertial * decay_factor;
    double predicted_wpsi = current_forces_body.z() * decay_factor;  // w_psi is scalar (yaw moment)
    
    // Predict USV heading at future time
    double predicted_psi = current_psi + local_pos.r * prediction_time_seconds;
    
    // Transform predicted inertial forces back to body frame at predicted heading
    Matrix2d R_inertial_to_body;
    R_inertial_to_body << cos(predicted_psi), sin(predicted_psi),
                         -sin(predicted_psi), cos(predicted_psi);
    
    Vector2d predicted_xy_body = R_inertial_to_body * predicted_xy_inertial;
    
    return Vector3d(predicted_xy_body.x(), predicted_xy_body.y(), predicted_wpsi);
}

void WAMV_MPC::updateValidationData() {
    Vector3d current_forces(esti_x[6], esti_x[7], esti_x[8]);
    double current_time = rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds() - start_time;
    double current_heading = local_pos.psi;
    
    // Store current predictions in body frame
    pred_validation.predicted_0_5s_body.push_back(predictWithDecay(0.5));
    pred_validation.predicted_1s_body.push_back(predictWithDecay(1.0));
    pred_validation.predicted_2s_body.push_back(predictWithDecay(2.0));
    
    // Store actual forces and corresponding heading
    pred_validation.actual_forces_body.push_back(current_forces);
    pred_validation.actual_headings.push_back(current_heading);
    pred_validation.timestamps.push_back(current_time);
    
    // Store heading when prediction was made (for frame correction)
    pred_validation.prediction_headings.push_back(current_heading);
    
    // Maintain buffer size
    if (pred_validation.predicted_0_5s_body.size() > VALIDATION_HISTORY_SIZE) {
        pred_validation.predicted_0_5s_body.pop_front();
        pred_validation.predicted_1s_body.pop_front();
        pred_validation.predicted_2s_body.pop_front();
        pred_validation.actual_forces_body.pop_front();
        pred_validation.prediction_headings.pop_front();
        pred_validation.actual_headings.pop_front();
        pred_validation.timestamps.pop_front();
    }
    
    // Enable validation after collecting enough data
    if (pred_validation.predicted_0_5s_body.size() >= 40 && !pred_validation.validation_ready) {
        pred_validation.validation_ready = true;
        RCLCPP_INFO(this->get_logger(), "Frame-aware prediction validation ready");
    }
    
    // Compute metrics every 10 samples
    if (pred_validation.validation_ready && pred_validation.predicted_0_5s_body.size() % 10 == 0) {
        computePredictionMetrics();
    }
}

void WAMV_MPC::computePredictionMetrics() {
    if (!pred_validation.validation_ready) return;
    
    size_t buffer_size = pred_validation.predicted_0_5s_body.size();
    
    // Validation indices (predictions made N steps ago vs current actual)
    int idx_0_5s = buffer_size - 10;  // 0.5 seconds ago
    int idx_1s = buffer_size - 20;    // 1 second ago  
    int idx_2s = buffer_size - 40;    // 2 seconds ago
    
    if (idx_2s >= 0) {  // Ensure sufficient data
        Vector3d actual_now_body = pred_validation.actual_forces_body.back();
        double actual_heading_now = pred_validation.actual_headings.back();
        
        // Transform current actual forces to inertial frame
        Vector3d actual_now_inertial = transformBodyToInertial(actual_now_body, actual_heading_now);
        
        // Process 0.5s prediction
        Vector3d pred_0_5s_body = pred_validation.predicted_0_5s_body[idx_0_5s];
        double pred_0_5s_heading = pred_validation.prediction_headings[idx_0_5s];
        Vector3d pred_0_5s_inertial = transformBodyToInertial(pred_0_5s_body, pred_0_5s_heading);
        
        // Process 1s prediction
        Vector3d pred_1s_body = pred_validation.predicted_1s_body[idx_1s];
        double pred_1s_heading = pred_validation.prediction_headings[idx_1s];
        Vector3d pred_1s_inertial = transformBodyToInertial(pred_1s_body, pred_1s_heading);
        
        // Process 2s prediction
        Vector3d pred_2s_body = pred_validation.predicted_2s_body[idx_2s];
        double pred_2s_heading = pred_validation.prediction_headings[idx_2s];
        Vector3d pred_2s_inertial = transformBodyToInertial(pred_2s_body, pred_2s_heading);
        
        // Calculate component-wise errors in inertial frame
        Vector3d error_0_5s = pred_0_5s_inertial - actual_now_inertial;
        Vector3d error_1s = pred_1s_inertial - actual_now_inertial;
        Vector3d error_2s = pred_2s_inertial - actual_now_inertial;
        
        // Component-wise absolute errors (instantaneous)
        double abs_error_0_5s_wx = std::abs(error_0_5s.x());
        double abs_error_0_5s_wy = std::abs(error_0_5s.y());
        double abs_error_0_5s_wpsi = std::abs(error_0_5s.z());
        
        double abs_error_1s_wx = std::abs(error_1s.x());
        double abs_error_1s_wy = std::abs(error_1s.y());
        double abs_error_1s_wpsi = std::abs(error_1s.z());
        
        double abs_error_2s_wx = std::abs(error_2s.x());
        double abs_error_2s_wy = std::abs(error_2s.y());
        double abs_error_2s_wpsi = std::abs(error_2s.z());
        
        // Overall RMSE (vector norm)
        double rmse_0_5s_overall_new = error_0_5s.norm();
        double rmse_1s_overall_new = error_1s.norm();
        double rmse_2s_overall_new = error_2s.norm();
        
        // Exponential moving average update
        double alpha = 0.15;  // Smoothing factor
        
        if (pred_validation.validation_samples == 0) {
            // Initialize on first sample
            pred_validation.rmse_0_5s_wx = abs_error_0_5s_wx;
            pred_validation.rmse_0_5s_wy = abs_error_0_5s_wy;
            pred_validation.rmse_0_5s_wpsi = abs_error_0_5s_wpsi;
            
            pred_validation.rmse_1s_wx = abs_error_1s_wx;
            pred_validation.rmse_1s_wy = abs_error_1s_wy;
            pred_validation.rmse_1s_wpsi = abs_error_1s_wpsi;
            
            pred_validation.rmse_2s_wx = abs_error_2s_wx;
            pred_validation.rmse_2s_wy = abs_error_2s_wy;
            pred_validation.rmse_2s_wpsi = abs_error_2s_wpsi;
            
            pred_validation.rmse_0_5s_overall = rmse_0_5s_overall_new;
            pred_validation.rmse_1s_overall = rmse_1s_overall_new;
            pred_validation.rmse_2s_overall = rmse_2s_overall_new;
            
            // Initialize MAE (same as RMSE for absolute errors)
            pred_validation.mae_0_5s_wx = abs_error_0_5s_wx;
            pred_validation.mae_0_5s_wy = abs_error_0_5s_wy;
            pred_validation.mae_0_5s_wpsi = abs_error_0_5s_wpsi;
            
            pred_validation.mae_1s_wx = abs_error_1s_wx;
            pred_validation.mae_1s_wy = abs_error_1s_wy;
            pred_validation.mae_1s_wpsi = abs_error_1s_wpsi;
            
            pred_validation.mae_2s_wx = abs_error_2s_wx;
            pred_validation.mae_2s_wy = abs_error_2s_wy;
            pred_validation.mae_2s_wpsi = abs_error_2s_wpsi;
        } else {
            // Running average update for component-wise RMSE
            pred_validation.rmse_0_5s_wx = alpha * abs_error_0_5s_wx + (1.0 - alpha) * pred_validation.rmse_0_5s_wx;
            pred_validation.rmse_0_5s_wy = alpha * abs_error_0_5s_wy + (1.0 - alpha) * pred_validation.rmse_0_5s_wy;
            pred_validation.rmse_0_5s_wpsi = alpha * abs_error_0_5s_wpsi + (1.0 - alpha) * pred_validation.rmse_0_5s_wpsi;
            
            pred_validation.rmse_1s_wx = alpha * abs_error_1s_wx + (1.0 - alpha) * pred_validation.rmse_1s_wx;
            pred_validation.rmse_1s_wy = alpha * abs_error_1s_wy + (1.0 - alpha) * pred_validation.rmse_1s_wy;
            pred_validation.rmse_1s_wpsi = alpha * abs_error_1s_wpsi + (1.0 - alpha) * pred_validation.rmse_1s_wpsi;
            
            pred_validation.rmse_2s_wx = alpha * abs_error_2s_wx + (1.0 - alpha) * pred_validation.rmse_2s_wx;
            pred_validation.rmse_2s_wy = alpha * abs_error_2s_wy + (1.0 - alpha) * pred_validation.rmse_2s_wy;
            pred_validation.rmse_2s_wpsi = alpha * abs_error_2s_wpsi + (1.0 - alpha) * pred_validation.rmse_2s_wpsi;
            
            // Overall RMSE update
            pred_validation.rmse_0_5s_overall = alpha * rmse_0_5s_overall_new + (1.0 - alpha) * pred_validation.rmse_0_5s_overall;
            pred_validation.rmse_1s_overall = alpha * rmse_1s_overall_new + (1.0 - alpha) * pred_validation.rmse_1s_overall;
            pred_validation.rmse_2s_overall = alpha * rmse_2s_overall_new + (1.0 - alpha) * pred_validation.rmse_2s_overall;
            
            // MAE update (for component-wise, MAE = RMSE since we use absolute values)
            pred_validation.mae_0_5s_wx = pred_validation.rmse_0_5s_wx;
            pred_validation.mae_0_5s_wy = pred_validation.rmse_0_5s_wy;
            pred_validation.mae_0_5s_wpsi = pred_validation.rmse_0_5s_wpsi;
            
            pred_validation.mae_1s_wx = pred_validation.rmse_1s_wx;
            pred_validation.mae_1s_wy = pred_validation.rmse_1s_wy;
            pred_validation.mae_1s_wpsi = pred_validation.rmse_1s_wpsi;
            
            pred_validation.mae_2s_wx = pred_validation.rmse_2s_wx;
            pred_validation.mae_2s_wy = pred_validation.rmse_2s_wy;
            pred_validation.mae_2s_wpsi = pred_validation.rmse_2s_wpsi;
        }
        
        pred_validation.validation_samples++;
        
        // Log detailed component analysis occasionally
        if (pred_validation.validation_samples % 200 == 0) {
            RCLCPP_INFO(this->get_logger(), 
                       "Component Analysis - 1s prediction: wx_err=%.2f, wy_err=%.2f, wpsi_err=%.2f", 
                       pred_validation.rmse_1s_wx, pred_validation.rmse_1s_wy, pred_validation.rmse_1s_wpsi);
                       
            // Identify worst performing component
            std::string worst_component = "wx";
            double worst_error = pred_validation.rmse_1s_wx;
            
            if (pred_validation.rmse_1s_wy > worst_error) {
                worst_component = "wy";
                worst_error = pred_validation.rmse_1s_wy;
            }
            
            if (pred_validation.rmse_1s_wpsi > worst_error) {
                worst_component = "wpsi";
                worst_error = pred_validation.rmse_1s_wpsi;
            }
            
            RCLCPP_INFO(this->get_logger(), 
                       "Worst prediction component: %s (error=%.2f)", 
                       worst_component.c_str(), worst_error);
        }
    }
}

void WAMV_MPC::fillMPCHorizonWithPrediction() {
    // Fill MPC prediction horizon with trend-decay predictions
    for (int i = 0; i <= WAMV_N; i++) {
        double prediction_time = i * 0.05;  // 50ms MPC timestep
        Vector3d predicted_forces = predictWithDecay(prediction_time);
        
        // Apply environmental assistance factors
        acados_param[i][2] = predicted_forces.x() * environmental_assistance.surge_assistance_factor;
        acados_param[i][3] = predicted_forces.y() * environmental_assistance.sway_assistance_factor;
        acados_param[i][4] = predicted_forces.z() * environmental_assistance.yaw_assistance_factor;
    }
}

Vector3d WAMV_MPC::transformBodyToInertial(const Vector3d& forces_body, double heading) {
    Matrix2d R_body_to_inertial;
    R_body_to_inertial << cos(heading), -sin(heading),
                          sin(heading),  cos(heading);
    
    Vector2d forces_xy_body(forces_body.x(), forces_body.y());
    Vector2d forces_xy_inertial = R_body_to_inertial * forces_xy_body;
    
    return Vector3d(forces_xy_inertial.x(), forces_xy_inertial.y(), forces_body.z());
}