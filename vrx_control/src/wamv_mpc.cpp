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
    
    left_thrust_angle_pub = this->create_publisher<std_msgs::msg::Float64>(
        "/wamv/thrusters/left/pos", 20);
    left_thrust_cmd_pub = this->create_publisher<std_msgs::msg::Float64>(
        "/wamv/thrusters/left/thrust", 20);
    right_thrust_angle_pub = this->create_publisher<std_msgs::msg::Float64>(
        "/wamv/thrusters/right/pos", 20);
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

    // initialize
    for(unsigned int i=0; i < WAMV_NU; i++) acados_out.u0[i] = 0.0;
    for(unsigned int i=0; i < WAMV_NX; i++) acados_in.x0[i] = 0.0;
    start_time = rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds();
    is_start = false;
    solver_param.Tp_pre = 0;
    solver_param.Ts_pre = 0;
    solver_param.delta_p_pre = 0;
    solver_param.delta_s_pre = 0;
    // pre_pos.u = 0;
    // pre_pos.v = 0;
    // pre_pos.r = 0;

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
    
    // Initialize calibration data with reserved capacity
    calibration_data.reserve(1000);
    
    // Initialize fault diagnosis model
    initializeFaultDiagnosis();
    
    // Initialize previous thruster commands
    prev_Tp = 0.0;
    prev_Ts = 0.0;
    prev_delta_p = 0.0;
    prev_delta_s = 0.0;

    // Initialize confidences
    fault_confidences.resize(3, 0.0); // Initialize with 3 zeros (one for each fault type)
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

    // set parameters
    double u_prev[4] = {solver_param.Tp_pre, solver_param.Ts_pre, solver_param.delta_p_pre, solver_param.delta_s_pre};
    for (int i = 0; i <= WAMV_N; i++) {
        acados_param[i][0] = u_prev[0];  // Tp_prev
        acados_param[i][1] = u_prev[1];  // Ts_prev
        acados_param[i][2] = u_prev[2];  // delta_p_prev
        acados_param[i][3] = u_prev[3];  // delta_s_prev
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
    ref_cb(line_number); 
    line_number++;
    for (unsigned int i = 0; i <= WAMV_N; i++){
        ocp_nlp_cost_model_set(mpc_capsule->nlp_config, mpc_capsule->nlp_dims, mpc_capsule->nlp_in, i, "yref", acados_in.yref[i]);
    }

    // Solve OCP
    // acados_status = wamv_acados_solve(mpc_capsule);

    // if (acados_status != 0){
    //     RCLCPP_INFO(this->get_logger(), "acados returned status: %d", acados_status);
    // }

    // acados_out.status = acados_status;
    // acados_out.kkt_res = (double)mpc_capsule->nlp_out->inf_norm_res;

    // // ocp_nlp_get(mpc_capsule->nlp_config, mpc_capsule->nlp_solver, "time_tot", &acados_out.cpu_time);
    // ocp_nlp_get(mpc_capsule->nlp_solver, "time_tot", &acados_out.cpu_time);

    // ocp_nlp_out_get(mpc_capsule->nlp_config, mpc_capsule->nlp_dims, mpc_capsule->nlp_out, 0, "u", (void *)acados_out.u0);

    acados_out.u0[0] = 200;
    acados_out.u0[1] = 100;
    acados_out.u0[2] = 0;
    acados_out.u0[3] = 0;
    // if(testfd_counter < 300){
    //     publish_cin(acados_out.u0[0], acados_out.u0[1], acados_out.u0[2], acados_out.u0[3]);
    //     testfd_counter++;
    // }
    // else{
    //     publish_cin(0, 0, acados_out.u0[2], acados_out.u0[3]);
    // }
    publish_cin(acados_out.u0[0], acados_out.u0[1], acados_out.u0[2], acados_out.u0[3]);
    
    solver_param.Tp_pre = acados_out.u0[0];
    solver_param.Ts_pre = acados_out.u0[1];
    solver_param.delta_p_pre = acados_out.u0[2];
    solver_param.delta_s_pre = acados_out.u0[3];

    double current_time = rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds();
    double z[4];
    ocp_nlp_out_get(mpc_capsule->nlp_config, mpc_capsule->nlp_dims, mpc_capsule->nlp_out, 0, "z", z);
    std::string fault_status;
    std::string fault_color;
    
    if (fault_detected) {
        switch (current_fault_type) {
            case NO_FAULT:
                fault_status = "NO_FAULT";
                fault_color = "\033[32m"; // Green
                break;
            case LEFT_THRUST_FAILURE:
                fault_status = "LEFT_THRUST_FAILURE";
                fault_color = "\033[31m"; // Red
                break;
            case RIGHT_THRUST_FAILURE:
                fault_status = "RIGHT_THRUST_FAILURE";
                fault_color = "\033[31m"; // Red
                break;
            default:
                fault_status = "UNKNOWN_FAULT";
                fault_color = "\033[35m"; // Magenta
        }
    } else {
        fault_status = "NORMAL";
        fault_color = "\033[32m"; // Green
    }
    // Calculate calibrated w_psi for display
    double calibrated_wpsi = getCalibrated_wpsi();

    if(cout_counter > 2){
        std::cout << "---------------------------------------------------------------------------------------------------------------------" << std::endl;
        std::cout << "ref_x:    " << acados_in.yref[0][0] << "\tref_y:   " << acados_in.yref[0][1] << "\tref_yaw:    " << acados_in.yref[0][2] << std::endl;
        std::cout << "error_x:  " << error_pose.pose.pose.position.x << "  error_y:  " << error_pose.pose.pose.position.y << "  error_psi:  " << yaw_error << std::endl;
        std::cout << "pos_x:  " << local_pos.x << "  pos_y:  " << local_pos.y << "  psi:  " << yaw_sum << std::endl;
        std::cout << "ekf pos_x:  " << esti_x[0] << "  pos_y:  " << esti_x[1] << "  psi:  " << esti_x[2] << std::endl;
        std::cout << "vel_x:  " << local_pos.u << "  vel_y:  " << local_pos.v << "  vel_r:  " << local_pos.r << std::endl;
        std::cout << "ekf vel_x:  " << esti_x[3] << "  vel_y:  " << esti_x[4] << "  vel_r:  " << esti_x[5] << std::endl;
        std::cout << "ekf w_x:  " << esti_x[6] << "  w_y:  " << esti_x[7] << "  w_psi:  " << esti_x[8] << std::endl;
        std::cout << "calibrated w_psi: " << calibrated_wpsi << " (raw: " << esti_x[8] << ", expected: " << (Ts.data - Tp.data) * wpsi_coefficient << ")" << std::endl;
        std::cout << "ekf acc_x:  " << ekf_acc.x << "  acc_y:  " << ekf_acc.y << "  acc_psi:  " << ekf_acc.psi << std::endl;
        std::cout << "Tp:  " << acados_out.u0[0] << "  Ts:  " << acados_out.u0[1] << "  delta_p:  " << acados_out.u0[2] << "  delta_s:  " << acados_out.u0[3] << std::endl;
        std::cout << "solve_time: "<< acados_out.cpu_time << "\tkkt_res: " << acados_out.kkt_res << "\tacados_status: " << acados_out.status << std::endl;
        std::cout << "relative_time: " << std::fixed << (current_time - start_time) << std::endl;
        std::cout << "Confidences NO_FAULT:  " << fault_confidences[0] << "  LEFT_THRUST_FAILURE:  " << fault_confidences[1] << "  RIGHT_THRUST_FAILURE:  " << fault_confidences[2] << std::endl;
        std::cout << "Confidences LEFT_ANGLE_FAILURE:  " << fault_confidences[3] << "  RIGHT_ANGLE_FAILURE:  " << fault_confidences[4] << std::endl;
        std::cout << fault_color << "FAULT STATUS: " << fault_status;
        if (fault_detected) {
            std::cout << " (Confidence: " << std::fixed << std::setprecision(2) << fault_detection_confidence * 100.0 << "%)";
        }
        std::cout << "\033[0m" << std::endl; // Reset color
        std::cout << "---------------------------------------------------------------------------------------------------------------------" << std::endl;
        cout_counter = 0;
    }
    else{
        cout_counter++;
    }
}

void WAMV_MPC::publish_cin(double Tp_mpc, double Ts_mpc, double delta_p_mpc, double delta_s_mpc)
{
    // Use enum to track the fault type
    enum FaultSimulationType {
        NO_FAULT_SIM = 0,
        LEFT_THRUSTER_FAULT_SIM = 1,
        RIGHT_THRUSTER_FAULT_SIM = 2,
        BOTH_THRUSTERS_FAULT_SIM = 3
    };
    
    // Define the fault type to simulate - change this to simulate different faults
    static const int FAULT_TYPE_TO_SIMULATE = NO_FAULT_SIM;  // Change as needed
    
    // Apply fault at the fault trigger point
    if (iteration_count < fault_trigger) {
        // Normal operation before fault trigger
        Tp.data = Tp_mpc;
        Ts.data = Ts_mpc;
        
        // Reset simulation tracking
        fault_simulation_active = false;
        simulated_fault_type = NO_FAULT;
    } else {
        // Apply the selected fault simulation
        switch (FAULT_TYPE_TO_SIMULATE) {
            case LEFT_THRUSTER_FAULT_SIM:
                Tp.data = 0.0;     // Port thruster fails (no force)
                Ts.data = Ts_mpc;  // Starboard thruster normal
                
                // Track the simulation state
                fault_simulation_active = true;
                simulated_fault_type = LEFT_THRUST_FAILURE;
                
                RCLCPP_INFO(this->get_logger(), "Simulating port thruster force failure at iteration %zu", iteration_count);
                break;
                
            case RIGHT_THRUSTER_FAULT_SIM:
                Tp.data = Tp_mpc;  // Port thruster normal
                Ts.data = 0.0;     // Starboard thruster fails (no force)
                
                // Track the simulation state
                fault_simulation_active = true;
                simulated_fault_type = RIGHT_THRUST_FAILURE;
                
                RCLCPP_INFO(this->get_logger(), "Simulating starboard thruster force failure at iteration %zu", iteration_count);
                break;
                
            case BOTH_THRUSTERS_FAULT_SIM:
                Tp.data = 0.0;     // Port thruster fails
                Ts.data = 0.0;     // Starboard thruster fails
                
                // Not tracking this case specifically
                fault_simulation_active = true;
                simulated_fault_type = LEFT_THRUST_FAILURE; // Arbitrary choice
                
                RCLCPP_INFO(this->get_logger(), "Simulating both thrusters force failure at iteration %zu", iteration_count);
                break;
                
            default:
                Tp.data = Tp_mpc;  // Normal operation
                Ts.data = Ts_mpc;
                
                fault_simulation_active = false;
                simulated_fault_type = NO_FAULT;
                break;
        }
    }
    iteration_count++;
    
    // Send actual values to thrusters
    left_thrust_cmd_pub->publish(Tp);
    right_thrust_cmd_pub->publish(Ts);
   
    delta_p.data = delta_p_mpc;
    left_thrust_angle_pub->publish(delta_p);

    delta_s.data = delta_s_mpc;
    right_thrust_angle_pub->publish(delta_s);

    // Update control inputs message with actual values (not commanded values)
    control_inputs.header.stamp = rclcpp::Clock().now();
    control_inputs.twist.linear.x = delta_p_mpc;
    control_inputs.twist.linear.y = Tp.data;  // Use actual Tp.data
    control_inputs.twist.angular.x = delta_s_mpc;
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

    // // publish ekf states
    // tf2::Quaternion quat_ekf;
    // quat_ekf.setRPY(0, 0, esti_x[2]);
    // geometry_msgs::msg::Quaternion quat_ekf_msg;
    // tf2::convert(quat_ekf, quat_ekf_msg);
    // ekf_pose.pose.pose.position.x = esti_x[0];
    // ekf_pose.pose.pose.position.y = esti_x[1];
    // ekf_pose.pose.pose.orientation.x = quat_ekf_msg.x;
    // ekf_pose.pose.pose.orientation.y = quat_ekf_msg.y;
    // ekf_pose.pose.pose.orientation.z = quat_ekf_msg.z;
    // ekf_pose.pose.pose.orientation.w = quat_ekf_msg.w;
    // ekf_pose.twist.twist.linear.x = esti_x[3];
    // ekf_pose.twist.twist.linear.y = esti_x[4];
    // ekf_pose.twist.twist.angular.z = esti_x[5];
    // ekf_pose.header.stamp = rclcpp::Clock().now();
    // ekf_pose.header.frame_id = "odom_frame";
    // ekf_pose.child_frame_id = "base_link";

    // ekf_pose_pub->publish(ekf_pose);


}

void WAMV_MPC::EKF()
{
    pre_ekf_pos.u = esti_x[3];
    pre_ekf_pos.v = esti_x[4];
    pre_ekf_pos.r = esti_x[5];
    // get input u and measuremnet y
    meas_u << solver_param.Tp_pre, solver_param.Ts_pre, solver_param.delta_p_pre, solver_param.delta_s_pre;
    // tau << meas_u[0] * cos(meas_u[2]) + meas_u[1] * cos(meas_u[3]),
    //         meas_u[0] * sin(meas_u[2]) + meas_u[1] * sin(meas_u[3]),
    //         -LCG * meas_u[0] * meas_u[2] - B/2 * meas_u[0] * sin(meas_u[2]) - LCG * meas_u[1] * cos(meas_u[3]) + B/2 * meas_u[1] * sin(meas_u[3]);
    // if two fixed direction thrusters
    tau << meas_u[0] + meas_u[1], 0, -LCG*meas_u[0]+LCG*meas_u[1];
    
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

void WAMV_MPC::initializeFaultDiagnosis() {
    // Print initial configuration for debugging
    std::cout << "\033[1;32m" << "Initializing Fault Diagnosis System" << "\033[0m" << std::endl;
    
    // Initialize fault detection parameters
    detection_counter = 0;
    fault_detected = false;
    current_fault_type = NO_FAULT;
    fault_detection_confidence = 0.0;
    
    // Initialize the online logistic regression model
    fault_model.feature_dim = 12; // Expanded feature set including turning points and thrust stability
    fault_model.weights = MatrixXd::Zero(fault_model.feature_dim, 5); // 5 classes (no fault + 4 fault types)
    fault_model.bias = 0.0;
    fault_model.learning_rate = 0.01;
    fault_model.lambda = 0.001;
    fault_model.buffer_size = window_size;
    fault_model.detect_threshold = 0.65;
    
    // Initialize the disturbance buffer
    dist_buffer.clear();
    for (int i = 0; i < window_size; i++) {
        dist_buffer.push_back(Vector3d::Zero());
    }
    
    // Initialize command history
    command_history.clear();
    
    // Try to load a pre-trained model if available
    try {
        loadFaultModel("fault_model.csv");
        RCLCPP_INFO(this->get_logger(), "Loaded pre-trained fault diagnosis model");
    } catch (...) {
        RCLCPP_INFO(this->get_logger(), "No pre-trained model found, starting with a new model");
        
        // Initialize weights to help with early detection of turning points
        // For LEFT_THRUST_FAILURE: positive change in wpsi
        fault_model.weights(5, LEFT_THRUST_FAILURE) = 2.0;  // Positive weight for wpsi change
        fault_model.weights(8, LEFT_THRUST_FAILURE) = 1.5;  // Strong weight for wpsi turning point
        
        // For RIGHT_THRUST_FAILURE: negative change in wpsi
        fault_model.weights(5, RIGHT_THRUST_FAILURE) = -2.0; // Negative weight for wpsi change
        fault_model.weights(8, RIGHT_THRUST_FAILURE) = 1.5;  // Strong weight for wpsi turning point
        
        // For both thrust failures: thrust commands unchanged during disturbance change
        if (fault_model.feature_dim >= 10) {
            fault_model.weights(9, LEFT_THRUST_FAILURE) = 1.0;  // Thrust unchanged
            fault_model.weights(9, RIGHT_THRUST_FAILURE) = 1.0; // Thrust unchanged
        }
    }
}

// Update the fault model with new disturbance information
void WAMV_MPC::updateFaultModel() {
    // Add the current disturbance to the buffer
    Vector3d current_dist(esti_x[6], esti_x[7], esti_x[8]);
    dist_buffer.push_back(current_dist);
    if (dist_buffer.size() > static_cast<size_t>(window_size)) {
        dist_buffer.pop_front();
    }
    
    // Track command history for detecting unchanged commands
    Vector4d current_command(Tp.data, Ts.data, delta_p.data, delta_s.data);
    command_history.push_back(current_command);
    if (command_history.size() > 50) {
        command_history.pop_front();
    }
    
    // Skip fault detection during warmup
    if (iteration_count < warmup_iterations) {
        warmup_completed = false;
        return;
    } else if (!warmup_completed) {
        warmup_completed = true;
        RCLCPP_INFO(this->get_logger(), "Warmup completed, fault detection active");
    }
    
    // Extract features
    VectorXd features(fault_model.feature_dim);
    extractFeatures(features);
    
    // Publish features for debugging
    auto feature_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
    feature_msg->data.resize(fault_model.feature_dim);
    for (int i = 0; i < fault_model.feature_dim; i++) {
        feature_msg->data[i] = features[i];
    }
    fault_features_pub->publish(*feature_msg);
    
    // Use model-based fault detection
    int detected_fault;
    bool is_fault = detectFault(features, detected_fault, fault_confidences);
    
    // State machine for fault status with faster confirmation
    static int same_fault_counter = 0;
    static int no_fault_counter = 0;
    static double first_detection_time = 0.0;
    
    if (is_fault) {
        // Record time of first detection
        if (same_fault_counter == 0) {
            first_detection_time = rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds() - start_time;
        }
        
        // Check if it's the same fault as before
        if (detected_fault == current_fault_type) {
            same_fault_counter++;
        } else {
            same_fault_counter = 1;
            current_fault_type = detected_fault;
        }
        
        // ULTRA-FAST: only need a single detection for thrust failures!
        // This dramatically reduces detection delay
        int required_detections = 1; // Requires only a single detection
        
        // After enough consistent detections, confirm the fault
        if (same_fault_counter >= required_detections) {
            if (!fault_detected || current_fault_type != detected_fault) {
                fault_detected = true;
                current_fault_type = detected_fault;
                fault_detection_confidence = fault_confidences[detected_fault];
                
                double detection_time = rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds() - start_time;
                
                // Report fault with detection timing info
                RCLCPP_INFO(this->get_logger(), 
                          "FAULT CONFIRMED - Type: %d, First detection: %.2fs, Confirmed: %.2fs, Delay: %.2fs",
                          current_fault_type, first_detection_time, detection_time, 
                          detection_time - first_detection_time);
                
                publishFaultDiagnosis(current_fault_type, fault_confidences);
                
                // Train the model with the detected fault
                logisticRegressionUpdate(features, current_fault_type);
            }
        }
        
        no_fault_counter = 0;
    } else {
        // No fault detected
        same_fault_counter = 0;
        no_fault_counter++;
        
        // Need more consecutive "no fault" detections to clear a fault (more conservative)
        if (no_fault_counter >= detection_count_threshold * 3) {
            if (fault_detected) {
                fault_detected = false;
                current_fault_type = NO_FAULT;
                fault_confidences[NO_FAULT] = 0.9;
                publishFaultDiagnosis(NO_FAULT, fault_confidences);
                
                // Train the model with NO_FAULT example
                logisticRegressionUpdate(features, NO_FAULT);
            }
        }
    }
    
    // Periodically train on current data regardless of fault status 
    // This helps the model learn normal operation patterns too
    static int training_counter = 0;
    if (++training_counter % 20 == 0) {
        // Use the current fault status as the label
        int training_label = fault_detected ? current_fault_type : NO_FAULT;
        logisticRegressionUpdate(features, training_label);
    }
    
    // Occasionally save the model
    static int save_counter = 0;
    if (++save_counter % 5000 == 0) {
        saveFaultModel("fault_model.csv");
        RCLCPP_INFO(this->get_logger(), "Saved fault model to fault_model.csv");
    }
    
    // Store current commands for next iteration
    prev_Tp = Tp.data;
    prev_Ts = Ts.data;
    prev_delta_p = delta_p.data;
    prev_delta_s = delta_s.data;
    
    // Run the calibration routine (throttled internally)
    if (calibration_enabled && !fault_detected) {
        calibrateDisturbanceModel();
    }
}

// Extract features from the disturbance buffer
void WAMV_MPC::extractFeatures(VectorXd& features) {
    // Calculate statistics on the disturbance buffer
    Vector3d mean = Vector3d::Zero();
    Vector3d variance = Vector3d::Zero();
    
    // Calculate mean
    for (const auto& dist : dist_buffer) {
        mean += dist;
    }
    mean /= dist_buffer.size();
    
    // Calculate variance
    for (const auto& dist : dist_buffer) {
        variance[0] += (dist[0] - mean[0]) * (dist[0] - mean[0]);
        variance[1] += (dist[1] - mean[1]) * (dist[1] - mean[1]);
        variance[2] += (dist[2] - mean[2]) * (dist[2] - mean[2]);
    }
    variance /= dist_buffer.size();
    
    // Focus more on the most recent changes - use smaller sections
    // This gives even greater emphasis to very recent changes
    size_t quarter_size = dist_buffer.size() / 4;
    size_t very_recent_start = dist_buffer.size() - quarter_size;
    size_t recent_start = dist_buffer.size() - quarter_size * 2;
    
    Vector3d very_recent_mean = Vector3d::Zero();
    Vector3d recent_mean = Vector3d::Zero();
    Vector3d older_mean = Vector3d::Zero();
    
    // Calculate means for each section
    for (size_t i = 0; i < quarter_size && i < dist_buffer.size(); i++) {
        if (i + very_recent_start < dist_buffer.size()) {
            very_recent_mean += dist_buffer[i + very_recent_start];
        }
        if (i + recent_start < very_recent_start) {
            recent_mean += dist_buffer[i + recent_start];
        }
        older_mean += dist_buffer[i];
    }
    very_recent_mean /= std::min(quarter_size, dist_buffer.size() - very_recent_start);
    recent_mean /= std::min(quarter_size, very_recent_start - recent_start);
    older_mean /= std::min(quarter_size, dist_buffer.size());
    
    // Calculate rate of change between very_recent and recent
    double wpsi_rate = 0.0;
    if (quarter_size > 0) {
        wpsi_rate = (very_recent_mean[2] - recent_mean[2]) / quarter_size;
    }
    
    // Calculate acceleration (change in rate)
    double wpsi_accel = 0.0;
    double older_wpsi_rate = 0.0;
    if (quarter_size > 0) {
        older_wpsi_rate = (recent_mean[2] - older_mean[2]) / quarter_size;
        wpsi_accel = wpsi_rate - older_wpsi_rate;
    }
    
    // Enhanced criteria for pattern change detection with heightened sensitivity
    bool significant_pattern_change = false;
    
    // More sensitive detection using both rate and acceleration
    double very_recent_vs_recent = std::abs(very_recent_mean[2] - recent_mean[2]);
    double recent_vs_older = std::abs(recent_mean[2] - older_mean[2]);
    
    // Detect even smaller changes in very recent data
    if (very_recent_vs_recent > wpsi_threshold * 0.4 ||  // Lower threshold for faster detection
        std::abs(wpsi_accel) > 0.1 ||                   // Detect changes in acceleration
        very_recent_vs_recent > recent_vs_older * 1.3) { // Look for any change in pattern
        significant_pattern_change = true;
    }
    
    // Check if we're in a steady state (consistent pattern)
    bool steady_state = std::abs(wpsi_accel) < 0.05 && 
                       very_recent_vs_recent < wpsi_threshold * 0.2 &&
                       variance[2] < 0.8;
    
    // Determine if thrust commands have changed - only look at last 3 commands
    static double prev_Tp_sum = 0, prev_Ts_sum = 0;
    double current_Tp_sum = 0, current_Ts_sum = 0;
    
    // Use fewer commands for faster response
    size_t cmd_window = std::min(size_t(3), command_history.size());
    for (size_t i = 0; i < cmd_window; i++) {
        current_Tp_sum += command_history[command_history.size() - 1 - i][0];
        current_Ts_sum += command_history[command_history.size() - 1 - i][1];
    }
    current_Tp_sum /= cmd_window;
    current_Ts_sum /= cmd_window;
    
    // Calculate change in thrust commands
    double Tp_change = std::abs(current_Tp_sum - prev_Tp_sum);
    double Ts_change = std::abs(current_Ts_sum - prev_Ts_sum);
    bool thrust_unchanged = (Tp_change < 15.0) && (Ts_change < 15.0); // Slightly more lenient
    
    // Update for next time
    prev_Tp_sum = current_Tp_sum;
    prev_Ts_sum = current_Ts_sum;
    
    // Check for immediate spikes in wpsi as early warning signs
    bool wpsi_spike = false;
    if (dist_buffer.size() >= 3) {
        double latest_wpsi = dist_buffer.back()[2];
        double prev_wpsi = dist_buffer[dist_buffer.size()-2][2];
        double rate_change = std::abs(latest_wpsi - prev_wpsi);
        
        if (rate_change > wpsi_threshold * 0.3) {
            wpsi_spike = true;
        }
    }
    
    // Assemble the feature vector with improved pattern change detection
    features[0] = esti_x[6];  // Current w_x
    features[1] = esti_x[7];  // Current w_y
    features[2] = esti_x[8];  // Current w_psi
    features[3] = very_recent_mean[2]; // Very recent mean of wpsi
    features[4] = recent_mean[2];     // Recent mean of wpsi
    features[5] = older_mean[2];      // Older mean of wpsi
    features[6] = significant_pattern_change ? 1.0 : 0.0;  // Pattern change in w_psi
    features[7] = wpsi_accel;         // Acceleration in wpsi
    features[8] = steady_state ? 1.0 : 0.0;  // Steady pattern indicator
    
    // Add features related to the relationship between disturbance and thrust
    if (fault_model.feature_dim >= 12) {
        features[9] = thrust_unchanged ? 1.0 : 0.0;  // Whether thrust commands are stable
        features[10] = wpsi_spike ? 1.0 : 0.0;       // Early warning indicator
        features[11] = Ts.data - Tp.data;            // Thrust differential
    }
    
    // Debug output occasionally
    static int debug_counter = 0;
    if (debug_counter++ % 50 == 0) {
        RCLCPP_INFO(this->get_logger(), 
            "Pattern: very_recent_wpsi=%.3f, recent_wpsi=%.3f, wpsi_accel=%.3f, "
            "pattern_change=%d, steady=%d, thrust_unchanged=%d, spike=%d",
            very_recent_mean[2], recent_mean[2], wpsi_accel,
            significant_pattern_change, steady_state, thrust_unchanged, wpsi_spike);
    }
}

// Detect faults based on extracted features
bool WAMV_MPC::detectFault(const VectorXd& features, int& fault_type, std::vector<double>& fault_confidences) {
    // Skip detection during warmup period when EKF is still stabilizing
    if (iteration_count < warmup_iterations) {
        fault_type = NO_FAULT;
        fault_confidences.resize(3, 0.0);
        fault_confidences[NO_FAULT] = 0.9;
        fault_confidences[LEFT_THRUST_FAILURE] = 0.05;
        fault_confidences[RIGHT_THRUST_FAILURE] = 0.05;
        return false;
    }

    // Initialize history buffers for tracking trend
    static std::deque<double> wpsi_history;
    static std::deque<double> wx_history;
    static std::deque<double> wy_history;
    
    // Add current values to history
    double wx = esti_x[6];
    double wy = esti_x[7];
    double wpsi = esti_x[8];
    
    wx_history.push_back(wx);
    wy_history.push_back(wy);
    wpsi_history.push_back(wpsi);
    
    // Keep history to a reasonable size
    const size_t MAX_HISTORY = 10;
    if (wx_history.size() > MAX_HISTORY) {
        wx_history.pop_front();
        wy_history.pop_front();
        wpsi_history.pop_front();
    }
    
    // Calculate recent trends (for last 3 points)
    double wpsi_trend1 = 0;
    double wpsi_trend2 = 0;
    
    // Need at least 4 points for two sequential trends
    if (wpsi_history.size() >= 4) {
        // First trend (previous 3 points)
        wpsi_trend1 = wpsi_history[wpsi_history.size() - 2] - wpsi_history[wpsi_history.size() - 4];
        
        // Second trend (most recent 3 points)
        wpsi_trend2 = wpsi_history[wpsi_history.size() - 1] - wpsi_history[wpsi_history.size() - 3];
    }
    
    // Check for trend reversals (sign change in consecutive trends)
    bool wpsi_turning_point = (wpsi_trend1 * wpsi_trend2 < 0) && 
                             (std::abs(wpsi_trend1) > 0.1) && 
                             (std::abs(wpsi_trend2) > 0.1);
    
    // Calculate current rate of change in wpsi
    double current_wpsi_slope = 0;
    if (wpsi_history.size() >= 3) {
        current_wpsi_slope = wpsi_history.back() - wpsi_history[wpsi_history.size() - 3];
    }
    
    // Log buffer contents for debugging
    std::string wpsi_buffer_str = "wpsi_buffer: ";
    for (double val : wpsi_history) {
        wpsi_buffer_str += std::to_string(val) + " ";
    }
    RCLCPP_INFO(this->get_logger(), "%s", wpsi_buffer_str.c_str());
    
    // Log turning point and trend information
    RCLCPP_INFO(this->get_logger(), 
        "Trends: wpsi_trend1=%.3f, wpsi_trend2=%.3f, slope=%.3f, turning_point=%s",
        wpsi_trend1, wpsi_trend2, current_wpsi_slope, 
        wpsi_turning_point ? "YES" : "NO");
    
    // Initialize fault confidences to default values
    fault_confidences.resize(3, 0.0);
    fault_confidences[NO_FAULT] = 0.8;
    fault_confidences[LEFT_THRUST_FAILURE] = 0.1;
    fault_confidences[RIGHT_THRUST_FAILURE] = 0.1;
    
    // ===== SIMPLE TREND-BASED FAULT DETECTION =====
    
    // Also check for abrupt change in slope direction
    bool significant_slope_change = false;
    static double prev_slope = 0;
    
    if (std::abs(current_wpsi_slope) > 0.1) {
        // If previous slope had opposite sign and significant magnitude
        if (prev_slope * current_wpsi_slope < 0 && std::abs(prev_slope) > 0.1) {
            significant_slope_change = true;
            RCLCPP_INFO(this->get_logger(), "Significant slope change detected: %.3f -> %.3f",
                       prev_slope, current_wpsi_slope);
        }
        // Update previous slope
        prev_slope = current_wpsi_slope;
    }
    
    // Raw fault detection based on turning point or significant slope change
    bool raw_fault_detected = wpsi_turning_point || significant_slope_change;
    int raw_fault_type = NO_FAULT;
    
    if (raw_fault_detected) {
        // Analyze current configuration
        bool is_forward_motion = (std::abs(delta_p.data) < 0.2 && std::abs(delta_s.data) < 0.2);
        bool is_right_turn = (delta_p.data > 1.0 && delta_s.data > 1.0);
        bool is_left_turn = (delta_p.data < -1.0 && delta_s.data < -1.0);
        
        // Determine fault type based on trend direction after the turning point
        if (is_forward_motion) {
            // For forward motion: slope direction after turning point indicates fault type
            raw_fault_type = (current_wpsi_slope > 0) ? LEFT_THRUST_FAILURE : RIGHT_THRUST_FAILURE;
        }
        else if (is_right_turn) {
            // For right turn: change from negative to positive slope -> RIGHT thruster failure
            //                  change from positive to negative slope -> LEFT thruster failure
            raw_fault_type = (current_wpsi_slope > 0) ? RIGHT_THRUST_FAILURE : LEFT_THRUST_FAILURE;
        }
        else if (is_left_turn) {
            // For left turn: change from negative to positive slope -> LEFT thruster failure
            //                 change from positive to negative slope -> RIGHT thruster failure
            raw_fault_type = (current_wpsi_slope > 0) ? LEFT_THRUST_FAILURE : RIGHT_THRUST_FAILURE;
        }
        else {
            // For complex motion: use slope direction
            raw_fault_type = (current_wpsi_slope > 0) ? LEFT_THRUST_FAILURE : RIGHT_THRUST_FAILURE;
        }
        
        RCLCPP_INFO(this->get_logger(), 
            "FAULT DETECTED - Slope: %.3f, Raw fault type: %d",
            current_wpsi_slope, raw_fault_type);
    }
    
    // ===== FAULT STATE FILTERING =====
    // Tracking variables (static to persist between calls)
    static int consecutive_fault_detections = 0;
    static int consecutive_normal_detections = 0;
    static int last_detected_fault_type = NO_FAULT;
    static bool fault_state_active = false;
    static bool use_ml_confidences = false;  // NEW: Separate flag for confidence mode

    // Fault confirmation parameters
    const int FAULT_CONFIRMATION_COUNT = 1;  // Only need 1 detection to confirm
    const int NORMAL_CONFIRMATION_COUNT = 5; // Need 5 consecutive normals to clear

    // Update state counters
    if (raw_fault_detected) {
        if (raw_fault_type == last_detected_fault_type) {
            consecutive_fault_detections++;
        } else {
            consecutive_fault_detections = 1;
            last_detected_fault_type = raw_fault_type;
        }
        consecutive_normal_detections = 0;
        use_ml_confidences = true;  // Switch to ML mode when fault detected
    } else {
        // Only increment normal detections if we're not currently in a fault state
        // OR if the disturbance values have actually returned to normal levels
        bool truly_normal = (std::abs(esti_x[6]) < wx_threshold/2) && 
                        (std::abs(esti_x[7]) < wy_threshold/2) && 
                        (std::abs(esti_x[8]) < wpsi_threshold/2);
        
        if (!fault_state_active || truly_normal) {
            consecutive_normal_detections++;
        } else {
            // Reset normal counter if disturbances are still high during fault state
            consecutive_normal_detections = 0;
        }
        
        if (consecutive_normal_detections > 100) consecutive_normal_detections = 100; // Prevent overflow
        
        // IMPORTANT: Don't reset use_ml_confidences here!
        // Only reset it when fault is completely cleared (in the state machine below)
    }

    // ALWAYS calculate ML confidence values (regardless of fault state)
    VectorXd scores = VectorXd::Zero(3);
    for (int i = 0; i < 3; i++) {
        double logit = fault_model.weights.col(i).dot(features) + fault_model.bias;
        scores[i] = 1.0 / (1.0 + exp(-logit));
    }

    // Normalize to ensure sum = 1.0
    double sum = scores.sum();
    if (sum > 0) {
        fault_confidences[NO_FAULT] = scores[0] / sum;
        fault_confidences[LEFT_THRUST_FAILURE] = scores[1] / sum;
        fault_confidences[RIGHT_THRUST_FAILURE] = scores[2] / sum;
    } else {
        // Fallback values if ML model fails
        fault_confidences[NO_FAULT] = 0.8;
        fault_confidences[LEFT_THRUST_FAILURE] = 0.1;
        fault_confidences[RIGHT_THRUST_FAILURE] = 0.1;
    }

    // Decide which confidences to use based on our separate flag
    RCLCPP_INFO(this->get_logger(), 
        "BEFORE confidence decision: use_ml_confidences=%s, fault_state_active=%s, raw_fault_detected=%s", 
        use_ml_confidences ? "true" : "false", 
        fault_state_active ? "true" : "false",
        raw_fault_detected ? "true" : "false");

    if (!use_ml_confidences) {
        // Normal operation - use default values
        fault_confidences[NO_FAULT] = 0.8;           // 80%
        fault_confidences[LEFT_THRUST_FAILURE] = 0.1; // 10%
        fault_confidences[RIGHT_THRUST_FAILURE] = 0.1; // 10%
        
        RCLCPP_INFO(this->get_logger(), "SETTING DEFAULT confidences: 80/10/10");
    } else {
        // Use ML confidences, but ensure reasonable fault-specific values
        RCLCPP_INFO(this->get_logger(), 
            "KEEPING ML confidences: NO_FAULT=%.1f, LEFT=%.1f, RIGHT=%.1f", 
            fault_confidences[NO_FAULT]*100, fault_confidences[LEFT_THRUST_FAILURE]*100, 
            fault_confidences[RIGHT_THRUST_FAILURE]*100);
            
        if (fault_state_active && last_detected_fault_type == LEFT_THRUST_FAILURE) {
            if (fault_confidences[LEFT_THRUST_FAILURE] < 0.5) {
                fault_confidences[LEFT_THRUST_FAILURE] = 0.7;
                fault_confidences[RIGHT_THRUST_FAILURE] = 0.15;
                fault_confidences[NO_FAULT] = 0.15;
                RCLCPP_INFO(this->get_logger(), "BOOSTED LEFT fault confidence to 70/15/15");
            }
        } else if (fault_state_active && last_detected_fault_type == RIGHT_THRUST_FAILURE) {
            if (fault_confidences[RIGHT_THRUST_FAILURE] < 0.5) {
                fault_confidences[RIGHT_THRUST_FAILURE] = 0.7;
                fault_confidences[LEFT_THRUST_FAILURE] = 0.15;
                fault_confidences[NO_FAULT] = 0.15;
                RCLCPP_INFO(this->get_logger(), "BOOSTED RIGHT fault confidence to 15/70/15");
            }
        }
    }

    RCLCPP_INFO(this->get_logger(), 
        "FINAL confidences before state machine: NO_FAULT=%.1f, LEFT=%.1f, RIGHT=%.1f", 
        fault_confidences[NO_FAULT]*100, fault_confidences[LEFT_THRUST_FAILURE]*100, 
        fault_confidences[RIGHT_THRUST_FAILURE]*100);

    // State machine for fault status
    if (!fault_state_active) {
        // Currently in normal state
        if (consecutive_fault_detections >= FAULT_CONFIRMATION_COUNT) {
            // Confirm fault
            fault_state_active = true;
            fault_type = last_detected_fault_type;
            
            // Use ML confidences for fault state
            
            confidence_level.header.stamp = rclcpp::Clock().now();
            confidence_level.twist.linear.x = fault_confidences[LEFT_THRUST_FAILURE];
            confidence_level.twist.linear.y = fault_confidences[RIGHT_THRUST_FAILURE]; 
            confidence_level.twist.angular.x = fault_confidences[NO_FAULT];
            confidence_pub->publish(confidence_level);
            
            RCLCPP_INFO(this->get_logger(), 
                "FAULT CONFIRMED - Type: %d, Consecutive detections: %d",
                fault_type, consecutive_fault_detections);
                
            return true;
        } else {
            // Stay in normal state - but still publish confidence values
            fault_type = NO_FAULT;
            
            // Publish confidence values for normal operation
            auto conf_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
            conf_msg->data.resize(3);
            conf_msg->data[0] = fault_confidences[NO_FAULT];
            conf_msg->data[1] = fault_confidences[LEFT_THRUST_FAILURE];
            conf_msg->data[2] = fault_confidences[RIGHT_THRUST_FAILURE];
            fault_confidence_pub->publish(*conf_msg);
            
            return false;
        }
    } else {
        // Currently in fault state
        if (consecutive_normal_detections >= NORMAL_CONFIRMATION_COUNT) {
            // Clear fault after multiple normal readings
            fault_state_active = false;
            fault_type = NO_FAULT;
            use_ml_confidences = false;  // Switch back to default values
            
            // Don't set confidence values here - they'll be set above based on use_ml_confidences flag
            
            // Publish the cleared fault confidence
            auto conf_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
            conf_msg->data.resize(3);
            conf_msg->data[0] = fault_confidences[NO_FAULT];
            conf_msg->data[1] = fault_confidences[LEFT_THRUST_FAILURE];
            conf_msg->data[2] = fault_confidences[RIGHT_THRUST_FAILURE];
            fault_confidence_pub->publish(*conf_msg);
            
            RCLCPP_INFO(this->get_logger(), 
                "FAULT CLEARED - After %d consecutive normal readings",
                consecutive_normal_detections);
                
            return false;
        } else {
            // Continue reporting current fault
            fault_type = last_detected_fault_type;
            
            // Keep the ML-calculated confidences (don't override them)
            // The confidences were already calculated above using the ML model
            
            // Publish fault confidence
            auto conf_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
            conf_msg->data.resize(3);
            conf_msg->data[0] = fault_confidences[NO_FAULT];
            conf_msg->data[1] = fault_confidences[LEFT_THRUST_FAILURE];
            conf_msg->data[2] = fault_confidences[RIGHT_THRUST_FAILURE];
            fault_confidence_pub->publish(*conf_msg);
            
            return true;
        }
    }
}

// Update the logistic regression model
void WAMV_MPC::logisticRegressionUpdate(const VectorXd& features, int label) 
{
    // Store data in buffers
    fault_model.feature_buffer.push_back(features);
    fault_model.label_buffer.push_back(label);
    
    // Ensure buffer doesn't exceed max size
    if (fault_model.feature_buffer.size() > static_cast<size_t>(fault_model.buffer_size)) {
        fault_model.feature_buffer.pop_front();
        fault_model.label_buffer.pop_front();
    }
    
    // Skip if we don't have enough data
    if (fault_model.feature_buffer.size() < static_cast<size_t>(10)) {
        return;
    }
    
    // Implement stochastic gradient descent update for logistic regression
    // This is a simplified multi-class logistic regression using one-vs-all approach
    
    // Create one-hot encoded label
    VectorXd one_hot = VectorXd::Zero(5);
    one_hot[label] = 1.0;
    
    // Calculate predictions (softmax)
    VectorXd scores = VectorXd::Zero(5);
    for (int i = 0; i < 5; i++) {
        scores[i] = fault_model.weights.col(i).dot(features) + fault_model.bias;
    }
    
    // Apply softmax
    double max_score = scores.maxCoeff();
    scores = scores.array() - max_score; // For numerical stability
    scores = scores.array().exp();
    double sum = scores.sum();
    scores = scores / sum;
    
    // Calculate gradient and update weights
    for (int i = 0; i < 5; i++) {
        VectorXd gradient = features * (scores[i] - one_hot[i]);
        fault_model.weights.col(i) -= fault_model.learning_rate * 
                                      (gradient + fault_model.lambda * fault_model.weights.col(i));
    }
    
    // Periodically save the model
    static int update_count = 0;
    update_count++;
    if (update_count % 1000 == 0) {
        saveFaultModel("fault_model.csv");
    }
}

// Calculate statistics on the disturbance buffer
Vector3d WAMV_MPC::calculateDisturbanceStats(const std::deque<Vector3d>& buffer) 
{
    Vector3d mean = Vector3d::Zero();
    
    // Calculate mean
    for (const auto& dist : buffer) {
        mean += dist;
    }
    mean /= buffer.size();
    
    return mean;
}

// Publish fault diagnosis results
void WAMV_MPC::publishFaultDiagnosis(int fault_type, std::vector<double>& fault_confidences) 
{
    // Create a new message type for more detailed fault information
    auto fault_msg = std::make_unique<std_msgs::msg::String>();
    auto conf_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
    
    std::string fault_str;
    
    // Get calibrated w_psi
    double calibrated_wpsi = getCalibrated_wpsi();
    
    switch (fault_type) {
        case NO_FAULT:
            fault_str = "NO_FAULT";
            break;
        case LEFT_THRUST_FAILURE:
            fault_str = "LEFT_THRUST_FAILURE";
            break;
        case RIGHT_THRUST_FAILURE:
            fault_str = "RIGHT_THRUST_FAILURE";
            break;
        default:
            fault_str = "UNKNOWN_FAULT";
    }
    
    // Include all confidence values in the message
    fault_msg->data = "Fault: " + fault_str + " (Confidence: " + 
                   std::to_string(fault_confidences[fault_type] * 100.0) + "%)";
    
    // Add all confidence values to the array
    conf_msg->data.resize(3); // Only 3 fault types now
    conf_msg->data[0] = fault_confidences[NO_FAULT];
    conf_msg->data[1] = fault_confidences[LEFT_THRUST_FAILURE];
    conf_msg->data[2] = fault_confidences[RIGHT_THRUST_FAILURE];
    
    // Publish both messages
    fault_diagnosis_pub->publish(*fault_msg);
    fault_confidence_pub->publish(*conf_msg);
    
    RCLCPP_INFO(this->get_logger(), "FAULT DIAGNOSIS: %s", fault_str.c_str());
    RCLCPP_INFO(this->get_logger(), "Confidences - NO_FAULT: %.2f%%, LEFT: %.2f%%, RIGHT: %.2f%%",
               fault_confidences[NO_FAULT] * 100.0,
               fault_confidences[LEFT_THRUST_FAILURE] * 100.0,
               fault_confidences[RIGHT_THRUST_FAILURE] * 100.0);
               
    // Print additional debug info to console
    std::cout << "\033[1;36m" << "FAULT DIAGNOSIS: " << fault_str << "\033[0m" << std::endl;
    std::cout << "\033[1;36m" << "  Confidences - NO_FAULT: " << std::fixed << std::setprecision(2) 
              << fault_confidences[NO_FAULT] * 100.0 << "%, LEFT: " 
              << fault_confidences[LEFT_THRUST_FAILURE] * 100.0 << "%, RIGHT: "
              << fault_confidences[RIGHT_THRUST_FAILURE] * 100.0 << "%"
              << "\033[0m" << std::endl;
              
    // Print current disturbance values and thrusts with calibrated w_psi
    std::cout << "\033[1;36m" << "  Disturbances - w_x: " << esti_x[6] 
              << ", w_y: " << esti_x[7] << ", raw w_psi: " << esti_x[8] 
              << ", calibrated w_psi: " << calibrated_wpsi << "\033[0m" << std::endl;
              
    std::cout << "\033[1;36m" << "  Thrusts - Tp: " << Tp.data 
              << ", Ts: " << Ts.data << ", delta_p: " << delta_p.data 
              << ", delta_s: " << delta_s.data << "\033[0m" << std::endl;
}

// Save the fault model to a file
void WAMV_MPC::saveFaultModel(const std::string& filename) 
{
    std::ofstream file(filename);
    if (file.is_open()) {
        // Save weights
        for (int i = 0; i < fault_model.weights.rows(); i++) {
            for (int j = 0; j < fault_model.weights.cols(); j++) {
                file << fault_model.weights(i, j);
                if (j < fault_model.weights.cols() - 1) {
                    file << ",";
                }
            }
            file << std::endl;
        }
        file.close();
        RCLCPP_INFO(this->get_logger(), "Saved fault model to %s", filename.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to save fault model to %s", filename.c_str());
    }
}

// Load the fault model from a file
void WAMV_MPC::loadFaultModel(const std::string& filename) 
{
    std::ifstream file(filename);
    if (file.is_open()) {
        std::string line;
        int row = 0;
        
        while (std::getline(file, line) && row < fault_model.weights.rows()) {
            std::stringstream ss(line);
            std::string cell;
            int col = 0;
            
            while (std::getline(ss, cell, ',') && col < fault_model.weights.cols()) {
                fault_model.weights(row, col) = std::stod(cell);
                col++;
            }
            row++;
        }
        
        file.close();
        RCLCPP_INFO(this->get_logger(), "Loaded fault model from %s", filename.c_str());
    } else {
        RCLCPP_WARN(this->get_logger(), "Failed to load fault model from %s", filename.c_str());
        throw std::runtime_error("Failed to load fault model");
    }
}

// Implement the calibration function
void WAMV_MPC::calibrateDisturbanceModel()
{
    // Debug output to see current values
    static int debug_counter = 0;
    if (debug_counter++ % 100 == 0) {
        std::cout << "\033[1;34m" << "Calibration Debug - wpsi_coefficient: " << wpsi_coefficient
                 << ", raw_wpsi: " << esti_x[8]
                 << ", thrust_diff: " << (Ts.data - Tp.data)
                 << ", expected_wpsi: " << ((Ts.data - Tp.data) * wpsi_coefficient)
                 << ", calibrated_wpsi: " << getCalibrated_wpsi()
                 << "\033[0m" << std::endl;
    }
    
    // Skip calibration if disabled
    if (!calibration_enabled) {
        return;
    }
    
    // Only run calibration every 10 iterations
    calibration_counter++;
    if (calibration_counter % 10 != 0) {
        return;
    }
    
    // Only collect data during stable operation with no faults
    static std::deque<Vector4d> command_history;
    Vector4d current_command(Tp.data, Ts.data, delta_p.data, delta_s.data);
    command_history.push_back(current_command);
    if (command_history.size() > 50) {
        command_history.pop_front();
    }
    
    // Check if commands have been stable for a reasonable period
    bool commands_stable = true;
    if (command_history.size() > 20) {
        Vector4d first_cmd = command_history[command_history.size() - 20];
        for (size_t i = command_history.size() - 19; i < command_history.size(); i++) {
            if (std::abs(command_history[i][0] - first_cmd[0]) > 10.0 || 
                std::abs(command_history[i][1] - first_cmd[1]) > 10.0 ||
                std::abs(command_history[i][2] - first_cmd[2]) > 0.1 ||
                std::abs(command_history[i][3] - first_cmd[3]) > 0.1) {
                commands_stable = false;
                break;
            }
        }
    } else {
        commands_stable = false;
    }
    
    // Only collect data during stable operation with no faults and sufficient thrust
    if (!fault_detected && commands_stable && 
        Tp.data > 30.0 && Ts.data > 30.0 && 
        std::abs(esti_x[8]) > 1.0) {
        
        double thrust_diff = Ts.data - Tp.data;
        double current_wpsi = esti_x[8];
        
        // Add the data point to our calibration dataset
        calibration_data.push_back(std::make_pair(thrust_diff, current_wpsi));
        
        // Limit calibration dataset size
        if (calibration_data.size() > 1000) {
            calibration_data.erase(calibration_data.begin());
        }
        
        // Only update model if we have enough data points
        if (calibration_data.size() > 50) {
            // Perform linear regression to find the best relationship
            // between thrust_diff and wpsi using least squares method
            double sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_xx = 0.0;
            size_t n = calibration_data.size();
            
            for (const auto& point : calibration_data) {
                double x = point.first;   // thrust_diff
                double y = point.second;  // wpsi
                
                sum_x += x;
                sum_y += y;
                sum_xy += x * y;
                sum_xx += x * x;
            }
            
            // Calculate the slope (coefficient)
            if (std::abs(n * sum_xx - sum_x * sum_x) > 1e-6) {  // Avoid division by zero
                double new_coefficient = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
                
                // Smooth the update to avoid rapid changes
                double alpha = 0.1;  // Smoothing factor
                wpsi_coefficient = alpha * new_coefficient + (1.0 - alpha) * wpsi_coefficient;
                
                // Apply bounds to avoid unreasonable values
                wpsi_coefficient = std::max(-0.2, std::min(0.2, wpsi_coefficient));
                
                // Publish the updated coefficient
                auto msg = std::make_unique<std_msgs::msg::Float64>();
                msg->data = wpsi_coefficient;
                wpsi_coefficient_pub->publish(*msg);
                
                // Log the update occasionally
                static int log_counter = 0;
                if (log_counter++ % 50 == 0) {
                    RCLCPP_INFO(this->get_logger(), "Updated wpsi_coefficient: %.5f", wpsi_coefficient);
                }
            }
        }
    }
}

double WAMV_MPC::getCalibrated_wpsi() const {
    // Current w_psi value
    double raw_wpsi = esti_x[8];
    
    // Calculate thrust differential - use actual published values
    double thrust_diff = Ts.data - Tp.data;
    
    // Expected w_psi based on thrust differential
    double expected_wpsi = thrust_diff * wpsi_coefficient;
    
    // Calibrated value: actual minus expected
    double calibrated_wpsi = raw_wpsi - expected_wpsi;
    
    return calibrated_wpsi;
}

WAMV_MPC::ThrusterConfiguration WAMV_MPC::analyzeThrusterConfiguration(
    double tp, double ts, double delta_p, double delta_s) 
{
    ThrusterConfiguration config;
    config.type = ThrusterConfiguration::UNKNOWN;
    config.expected_wpsi = 0.0;
    config.turn_direction = 0.0;
    
    // Classification based on thruster angles
    bool is_forward_motion = (std::abs(delta_p) < 0.2 && std::abs(delta_s) < 0.2);
    bool is_right_turn = (delta_p > 1.0 && delta_s > 1.0);
    bool is_left_turn = (delta_p < -1.0 && delta_s < -1.0);
    
    // Calculate thrust differential
    double thrust_diff = ts - tp;
    
    // Calculate approximate forces and moments
    if (is_forward_motion) {
        // Forward motion classification
        config.type = ThrusterConfiguration::FORWARD;
        
        // In forward motion, yaw moment is primarily from thrust differential
        config.expected_wpsi = thrust_diff * wpsi_coefficient;
        
        // Set minimal turn direction indicator
        config.turn_direction = (thrust_diff > 0) ? 0.1 : -0.1;
        
        // For nearly identical thrusts, expect minimal yaw
        if (std::abs(thrust_diff) < 10.0) {
            config.expected_wpsi = 0.0;
            config.turn_direction = 0.0;
        }
    }
    else if (is_right_turn) {
        // Right turn classification (both thrusters angled right)
        config.type = ThrusterConfiguration::TURNING;
        config.turn_direction = 1.0;  // Right turn
        
        // For right turns, expect significant positive wpsi
        // This is just an approximate value - trend analysis is more important
        config.expected_wpsi = 15.0;  // Typical value for (200,200,1.57,1.57)
        
        // Scale with thrust magnitude
        double avg_thrust = (std::abs(tp) + std::abs(ts)) / 2.0;
        if (avg_thrust > 0) {
            config.expected_wpsi *= (avg_thrust / 200.0);
        }
    }
    else if (is_left_turn) {
        // Left turn classification (both thrusters angled left)
        config.type = ThrusterConfiguration::TURNING;
        config.turn_direction = -1.0;  // Left turn
        
        // For left turns, expect significant negative wpsi
        config.expected_wpsi = -15.0;  // Approximate value
        
        // Scale with thrust magnitude
        double avg_thrust = (std::abs(tp) + std::abs(ts)) / 2.0;
        if (avg_thrust > 0) {
            config.expected_wpsi *= (avg_thrust / 200.0);
        }
    }
    else {
        // Mixed or complex configuration
        double angle_diff = std::abs(delta_p - delta_s);
        bool similar_direction = angle_diff < 0.25;
        
        if (similar_direction) {
            // Thrusters pointing in similar direction - likely turning
            config.type = ThrusterConfiguration::TURNING;
            
            // Calculate average angle to determine turn direction
            double avg_angle = (delta_p + delta_s) / 2.0;
            config.turn_direction = (avg_angle > 0) ? 1.0 : -1.0;
            
            // Rough estimate of expected wpsi - less important for trend detection
            config.expected_wpsi = avg_angle * 5.0;
        }
        else {
            // Thrusters pointing in different directions - lateral or complex motion
            config.type = ThrusterConfiguration::COMPLEX;
            
            // Rough moment calculation for complex configurations
            double port_lateral = tp * std::sin(delta_p);
            double stbd_lateral = ts * std::sin(delta_s);
            double net_lateral = port_lateral + stbd_lateral;
            
            config.turn_direction = (net_lateral > 0) ? 0.5 : -0.5;
            config.expected_wpsi = net_lateral * 0.05;
        }
    }
    
    return config;
}