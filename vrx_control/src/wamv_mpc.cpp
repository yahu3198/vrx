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
    this->declare_parameter<int>("window_size", 50);
    this->declare_parameter<double>("learning_rate", 0.01);
    this->declare_parameter<double>("lambda", 0.001);
    this->declare_parameter<double>("wx_threshold", 15.0);     // Increased
    this->declare_parameter<double>("wy_threshold", 15.0);     // Increased
    this->declare_parameter<double>("wpsi_threshold", 5.0);    // Reduced - more sensitive to yaw
    this->declare_parameter<int>("detection_count_threshold", 2); // Lower for faster response
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
    
    // Initialize calibration data with reserved capacity
    calibration_data.reserve(1000);
    
    // Initialize fault diagnosis model
    initializeFaultDiagnosis();
    
    // Initialize previous thruster commands
    prev_Tp = 0.0;
    prev_Ts = 0.0;
    prev_delta_p = 0.0;
    prev_delta_s = 0.0;
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
    acados_out.u0[1] = 200;
    acados_out.u0[2] = 1.57;
    acados_out.u0[3] = 1.57;
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
            case LEFT_ANGLE_FAILURE:
                fault_status = "LEFT_ANGLE_FAILURE";
                fault_color = "\033[33m"; // Yellow
                break;
            case RIGHT_ANGLE_FAILURE:
                fault_status = "RIGHT_ANGLE_FAILURE";
                fault_color = "\033[33m"; // Yellow
                break;
            default:
                fault_status = "UNKNOWN_FAULT";
                fault_color = "\033[35m"; // Magenta
        }
    } else {
        fault_status = "NORMAL";
        fault_color = "\033[32m"; // Green
    }
    if(cout_counter > 2){
        std::cout << "---------------------------------------------------------------------------------------------------------------------" << std::endl;
        std::cout << "ref_x:    " << acados_in.yref[0][0] << "\tref_y:   " << acados_in.yref[0][1] << "\tref_yaw:    " << acados_in.yref[0][2] << std::endl;
        std::cout << "error_x:  " << error_pose.pose.pose.position.x << "  error_y:  " << error_pose.pose.pose.position.y << "  error_psi:  " << yaw_error << std::endl;
        std::cout << "pos_x:  " << local_pos.x << "  pos_y:  " << local_pos.y << "  psi:  " << yaw_sum << std::endl;
        std::cout << "ekf pos_x:  " << esti_x[0] << "  pos_y:  " << esti_x[1] << "  psi:  " << esti_x[2] << std::endl;
        std::cout << "vel_x:  " << local_pos.u << "  vel_y:  " << local_pos.v << "  vel_r:  " << local_pos.r << std::endl;
        std::cout << "ekf vel_x:  " << esti_x[3] << "  vel_y:  " << esti_x[4] << "  vel_r:  " << esti_x[5] << std::endl;
        std::cout << "ekf w_x:  " << esti_x[6] << "  w_y:  " << esti_x[7] << "  w_psi:  " << esti_x[8] << std::endl;
        std::cout << "ekf acc_x:  " << ekf_acc.x << "  acc_y:  " << ekf_acc.y << "  acc_psi:  " << ekf_acc.psi << std::endl;
        std::cout << "Tp:  " << acados_out.u0[0] << "  Ts:  " << acados_out.u0[1] << "  delta_p:  " << acados_out.u0[2] << "  delta_s:  " << acados_out.u0[3] << std::endl;
        std::cout << "z:  " << "  Tp_z:  " << z[0] << "  Ts_z:  " << z[1] << "  delta_p_z:  " << z[2] << "  delta_s_z:  " << z[3] << std::endl;
        std::cout << "solve_time: "<< acados_out.cpu_time << "\tkkt_res: " << acados_out.kkt_res << "\tacados_status: " << acados_out.status << std::endl;
        std::cout << "relative_time: " << std::fixed << (current_time - start_time) << std::endl;
        // Add the fault diagnosis status:
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
    // if (iteration_count % 20 == 0) {
        // RCLCPP_INFO(this->get_logger(), "Iteration: %zu, Tp_mpc: %f", iteration_count, Tp_mpc);
    // }

    // publish control inputs
    if (iteration_count < fault_trigger) {
        Tp.data = Tp_mpc;  // Normal operation
    } else {
        Tp.data = 0.0;     // Port thruster fails (no force)
        RCLCPP_INFO(this->get_logger(), "Simulating port thruster force failure at iteration %zu", iteration_count);
    }
    iteration_count++;
    left_thrust_cmd_pub->publish(Tp);

    Ts.data = Ts_mpc;
    right_thrust_cmd_pub->publish(Ts);
   
    delta_p.data = delta_p_mpc;
    left_thrust_angle_pub->publish(delta_p);

    delta_s.data = delta_s_mpc;
    right_thrust_angle_pub->publish(delta_s);

    control_inputs.header.stamp = rclcpp::Clock().now();
    control_inputs.twist.linear.x = delta_p_mpc;
    control_inputs.twist.linear.y = Tp_mpc;
    control_inputs.twist.angular.x = delta_s_mpc;
    control_inputs.twist.angular.y = Ts_mpc;
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
    tau << meas_u[0] * cos(meas_u[2]) + meas_u[1] * cos(meas_u[3]),
            meas_u[0] * sin(meas_u[2]) + meas_u[1] * sin(meas_u[3]),
            -LCG * meas_u[0] * meas_u[2] - B/2 * meas_u[0] * sin(meas_u[2]) - LCG * meas_u[1] * cos(meas_u[3]) + B/2 * meas_u[1] * sin(meas_u[3]);
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

void WAMV_MPC::initializeFaultDiagnosis() 
{
    // Print initial configuration for debugging
    std::cout << "\033[1;32m" << "Initializing Fault Diagnosis System" << "\033[0m" << std::endl;
    std::cout << "\033[1;32m" << "  window_size: " << window_size << "\033[0m" << std::endl;
    std::cout << "\033[1;32m" << "  wx_threshold: " << wx_threshold << "\033[0m" << std::endl;
    std::cout << "\033[1;32m" << "  wy_threshold: " << wy_threshold << "\033[0m" << std::endl;
    std::cout << "\033[1;32m" << "  wpsi_threshold: " << wpsi_threshold << "\033[0m" << std::endl;
    std::cout << "\033[1;32m" << "  detection_count_threshold: " << detection_count_threshold << "\033[0m" << std::endl;
    
    // Initialize fault detection parameters
    detection_counter = 0;
    fault_detected = false;
    current_fault_type = NO_FAULT;
    fault_detection_confidence = 0.0;
    
    // Initialize the online logistic regression model
    fault_model.feature_dim = 9; // Using 9 features for fault detection
    fault_model.weights = MatrixXd::Zero(fault_model.feature_dim, 5); // 5 classes (no fault + 4 fault types)
    fault_model.bias = 0.0;
    fault_model.learning_rate = 0.01;
    fault_model.lambda = 0.001;
    fault_model.buffer_size = window_size;
    fault_model.detect_threshold = 0.65;  // Slightly lower threshold for more sensitivity
    
    // Initialize the disturbance buffer
    dist_buffer.clear();
    for (int i = 0; i < window_size; i++) {
        dist_buffer.push_back(Vector3d::Zero());
    }
    
    // Try to load a pre-trained model if available
    try {
        loadFaultModel("fault_model.csv");
        RCLCPP_INFO(this->get_logger(), "Loaded pre-trained fault diagnosis model");
        std::cout << "\033[1;32m" << "Loaded pre-trained fault diagnosis model" << "\033[0m" << std::endl;
    } catch (...) {
        RCLCPP_INFO(this->get_logger(), "No pre-trained model found, starting with a new model");
        std::cout << "\033[1;32m" << "No pre-trained model found, starting with a new model" << "\033[0m" << std::endl;
        
        // Initialize weights with some basic patterns to help early detection
        // These will be refined with learning, but give a starting point
        
        // For LEFT_THRUST_FAILURE: positive wpsi (turning right)
        fault_model.weights(2, LEFT_THRUST_FAILURE) = 2.0;  // Strong positive weight for wpsi
        fault_model.weights(8, LEFT_THRUST_FAILURE) = 0.5;  // Positive slope

        // For RIGHT_THRUST_FAILURE: negative wpsi (turning left)
        fault_model.weights(2, RIGHT_THRUST_FAILURE) = -2.0; // Strong negative weight for wpsi
        fault_model.weights(8, RIGHT_THRUST_FAILURE) = -0.5; // Negative slope
        
        // For LEFT_ANGLE_FAILURE: moderate wpsi, significant wx
        fault_model.weights(0, LEFT_ANGLE_FAILURE) = 1.0;  // wx current value
        fault_model.weights(2, LEFT_ANGLE_FAILURE) = 0.5;  // wpsi current value
        
        // For RIGHT_ANGLE_FAILURE: opposite of left
        fault_model.weights(0, RIGHT_ANGLE_FAILURE) = -1.0; // wx current value
        fault_model.weights(2, RIGHT_ANGLE_FAILURE) = -0.5; // wpsi current value
    }
}

// Update the fault model with new disturbance information
void WAMV_MPC::updateFaultModel() 
{
    // Add the current disturbance to the buffer
    Vector3d current_dist(esti_x[6], esti_x[7], esti_x[8]);
    dist_buffer.push_back(current_dist);
    if (dist_buffer.size() > static_cast<size_t>(window_size)) {
        dist_buffer.pop_front();
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
    
    // Direct command check for rapid detection
    if (std::abs(Tp.data) < 5.0 && std::abs(prev_Tp) > 50.0) {
        // This is a definite LEFT thruster failure
        std::cout << "\033[1;31m" << "DIRECT LEFT THRUST FAILURE DETECTION: Tp dropped from " 
                  << prev_Tp << " to " << Tp.data << "\033[0m" << std::endl;
        fault_detected = true;
        current_fault_type = LEFT_THRUST_FAILURE;
        fault_detection_confidence = 0.95;
        publishFaultDiagnosis(LEFT_THRUST_FAILURE, 0.95);
        
        // Immediately update prev values and return
        prev_Tp = Tp.data;
        prev_Ts = Ts.data;
        prev_delta_p = delta_p.data;
        prev_delta_s = delta_s.data;
        return;
    }
    
    if (std::abs(Ts.data) < 5.0 && std::abs(prev_Ts) > 50.0) {
        // This is a definite RIGHT thruster failure
        std::cout << "\033[1;31m" << "DIRECT RIGHT THRUST FAILURE DETECTION: Ts dropped from " 
                  << prev_Ts << " to " << Ts.data << "\033[0m" << std::endl;
        fault_detected = true;
        current_fault_type = RIGHT_THRUST_FAILURE;
        fault_detection_confidence = 0.95;
        publishFaultDiagnosis(RIGHT_THRUST_FAILURE, 0.95);
        
        // Immediately update prev values and return
        prev_Tp = Tp.data;
        prev_Ts = Ts.data;
        prev_delta_p = delta_p.data;
        prev_delta_s = delta_s.data;
        return;
    }
    
    // Normal pattern-based detection
    int detected_fault;
    double confidence;
    bool is_fault = detectFault(features, detected_fault, confidence);
    
    // State machine for fault status
    static int same_fault_counter = 0;
    static int no_fault_counter = 0;
    
    if (is_fault) {
        // Check if it's the same fault as before
        if (detected_fault == current_fault_type) {
            same_fault_counter++;
        } else {
            same_fault_counter = 1;
            current_fault_type = detected_fault;
        }
        
        // After enough consistent detections, confirm the fault
        if (same_fault_counter >= detection_count_threshold) {
            if (!fault_detected || current_fault_type != detected_fault) {
                fault_detected = true;
                current_fault_type = detected_fault;
                fault_detection_confidence = confidence;
                publishFaultDiagnosis(current_fault_type, confidence);
            }
        }
        
        no_fault_counter = 0;
    } else {
        // No fault detected
        same_fault_counter = 0;
        no_fault_counter++;
        
        // Need more consecutive "no fault" detections to clear a fault
        if (no_fault_counter >= detection_count_threshold * 3) {
            if (fault_detected) {
                fault_detected = false;
                current_fault_type = NO_FAULT;
                publishFaultDiagnosis(NO_FAULT, 0.0);
            }
        }
    }
    
    // Store current commands for next iteration
    prev_Tp = Tp.data;
    prev_Ts = Ts.data;
    prev_delta_p = delta_p.data;
    prev_delta_s = delta_s.data;
}

// Extract features from the disturbance buffer
void WAMV_MPC::extractFeatures(VectorXd& features) 
{
    // Calculate statistics on the disturbance buffer
    Vector3d mean = Vector3d::Zero();
    Vector3d variance = Vector3d::Zero();
    Vector3d max_val = Vector3d::Zero();
    Vector3d min_val = Vector3d::Zero();
    
    // Initialize min/max values
    if (!dist_buffer.empty()) {
        min_val = max_val = dist_buffer.front();
    }
    
    // Calculate mean and find min/max
    for (const auto& dist : dist_buffer) {
        mean += dist;
        
        // Update max values
        if (dist[0] > max_val[0]) max_val[0] = dist[0];
        if (dist[1] > max_val[1]) max_val[1] = dist[1];
        if (dist[2] > max_val[2]) max_val[2] = dist[2];
        
        // Update min values
        if (dist[0] < min_val[0]) min_val[0] = dist[0];
        if (dist[1] < min_val[1]) min_val[1] = dist[1];
        if (dist[2] < min_val[2]) min_val[2] = dist[2];
    }
    mean /= dist_buffer.size();
    
    // Calculate variance
    for (const auto& dist : dist_buffer) {
        variance[0] += (dist[0] - mean[0]) * (dist[0] - mean[0]);
        variance[1] += (dist[1] - mean[1]) * (dist[1] - mean[1]);
        variance[2] += (dist[2] - mean[2]) * (dist[2] - mean[2]);
    }
    variance /= dist_buffer.size();
    
    // Calculate slope (trend) of disturbances over the window
    Vector3d slope = Vector3d::Zero();
    if (dist_buffer.size() >= 10) {
        // Take average of first 5 and last 5 elements to reduce noise
        Vector3d early_mean = Vector3d::Zero();
        Vector3d late_mean = Vector3d::Zero();
        
        for (size_t i = 0; i < 5; i++) {
            early_mean += dist_buffer[i];
        }
        early_mean /= 5;
        
        for (size_t i = dist_buffer.size() - 5; i < dist_buffer.size(); i++) {
            late_mean += dist_buffer[i];
        }
        late_mean /= 5;
        
        // Calculate rate of change
        slope = (late_mean - early_mean) / static_cast<double>(dist_buffer.size() - 5);
    }
    
    // Enhanced features vector
    features[0] = esti_x[6];  // Current w_x
    features[1] = esti_x[7];  // Current w_y
    features[2] = esti_x[8];  // Current w_psi
    
    // Mean of disturbances
    features[3] = mean[0];    // Mean of w_x
    features[4] = mean[1];    // Mean of w_y
    features[5] = mean[2];    // Mean of w_psi
    
    // Rate of change (slope)
    features[6] = slope[0];   // Trend of w_x
    features[7] = slope[1];   // Trend of w_y
    features[8] = slope[2];   // Trend of w_psi
}

// Detect faults based on extracted features
bool WAMV_MPC::detectFault(const VectorXd& features, int& fault_type, double& confidence) 
{
    // Get current disturbance values
    double wx = features[0];
    double wy = features[1];
    double wpsi = features[2];
    
    // Get trend values
    double wpsi_trend = features[8];
    
    // Print debug information periodically
    static int debug_counter = 0;
    if (debug_counter++ % 20 == 0) {
        std::cout << "\033[1;35m" << "PATTERN DEBUG - wx: " << wx
                  << ", wy: " << wy << ", wpsi: " << wpsi 
                  << ", wpsi_trend: " << wpsi_trend 
                  << ", Tp: " << Tp.data << ", Ts: " << Ts.data
                  << "\033[0m" << std::endl;
    }
    
    // Direct command-based detection (highest priority)
    if (std::abs(Tp.data) < 5.0 && std::abs(prev_Tp) > 50.0) {
        // This is a definite LEFT thruster failure
        std::cout << "\033[1;31m" << "DIRECT LEFT THRUST FAILURE DETECTION: Tp dropped from " 
                  << prev_Tp << " to " << Tp.data << "\033[0m" << std::endl;
        fault_type = LEFT_THRUST_FAILURE;
        confidence = 0.95;
        return true;
    }
    
    if (std::abs(Ts.data) < 5.0 && std::abs(prev_Ts) > 50.0) {
        // This is a definite RIGHT thruster failure
        std::cout << "\033[1;31m" << "DIRECT RIGHT THRUST FAILURE DETECTION: Ts dropped from " 
                  << prev_Ts << " to " << Ts.data << "\033[0m" << std::endl;
        fault_type = RIGHT_THRUST_FAILURE;
        confidence = 0.95;
        return true;
    }
    
    // Store historical wpsi values to detect significant changes
    static std::deque<double> wpsi_history;
    wpsi_history.push_back(wpsi);
    if (wpsi_history.size() > 30) { // 3 seconds at 10Hz
        wpsi_history.pop_front();
    }
    
    // Calculate the average of first 5 and last 5 values
    double early_avg = 0.0;
    double recent_avg = 0.0;
    double wpsi_change = 0.0;
    
    if (wpsi_history.size() >= 10) {
        for (int i = 0; i < 5; i++) {
            early_avg += wpsi_history[i];
        }
        early_avg /= 5.0;
        
        for (size_t i = wpsi_history.size() - 5; i < wpsi_history.size(); i++) {
            recent_avg += wpsi_history[i];
        }
        recent_avg /= 5.0;
        
        wpsi_change = recent_avg - early_avg;
    }
    
    // Track command history
    static std::deque<Vector4d> command_history;
    Vector4d current_command(Tp.data, Ts.data, delta_p.data, delta_s.data);
    command_history.push_back(current_command);
    if (command_history.size() > 50) { // 5 seconds at 10Hz
        command_history.pop_front();
    }
    
    // Check if commands have been stable (indicating normal operation)
    bool commands_stable = true;
    if (command_history.size() > 10) {
        Vector4d first_cmd = command_history.front();
        for (const auto& cmd : command_history) {
            if (std::abs(cmd[0] - first_cmd[0]) > 10.0 || 
                std::abs(cmd[1] - first_cmd[1]) > 10.0 ||
                std::abs(cmd[2] - first_cmd[2]) > 0.1 ||
                std::abs(cmd[3] - first_cmd[3]) > 0.1) {
                commands_stable = false;
                break;
            }
        }
    }
    
    // Clear detection thresholds
    bool might_be_fault = false;
    double left_thrust_score = 0.0;
    double right_thrust_score = 0.0;
    double left_angle_score = 0.0;
    double right_angle_score = 0.0;
    
    // IMPORTANT: Calculate expected disturbance from commands
    // In a normal operation, the yaw disturbance should be proportional to the differential thrust
    // For the WAM-V, this is a simplified model of expected disturbance
    double expected_wpsi = 0.0;
    
    // Only consider expected disturbance calculations for stable commands
    if (commands_stable && command_history.size() > 20) {
        // Calculate average thrust difference over recent history
        double avg_thrust_diff = 0.0;
        for (size_t i = command_history.size() - 10; i < command_history.size(); i++) {
            avg_thrust_diff += (command_history[i][1] - command_history[i][0]); // Ts - Tp
        }
        avg_thrust_diff /= 10.0;
        
        // Simple linear model: expected disturbance proportional to thrust difference
        // The coefficient should be tuned based on your specific vessel
        double wpsi_coefficient = 0.05; // This needs calibration
        expected_wpsi = avg_thrust_diff * wpsi_coefficient;
        
        // If commands are stable but disturbance differs significantly from expected,
        // that may indicate a fault
        double wpsi_deviation = wpsi - expected_wpsi;
        if (std::abs(wpsi_deviation) > wpsi_threshold * 2.0) {
            might_be_fault = true;
            
            // Determine which thruster is likely problematic based on the deviation
            if (wpsi_deviation > 0) { // Actual wpsi is more positive than expected
                left_thrust_score = 0.6;
            } else { // Actual wpsi is more negative than expected
                right_thrust_score = 0.6;
            }
        }
    }
    // For changing commands, we need to look at trends more than absolute values
    else {
        // Detection based on significant, sudden changes in wpsi
        if (std::abs(wpsi_change) > 5.0) {
            might_be_fault = true;
            
            if (wpsi_change > 0) { // Sudden increase in wpsi
                left_thrust_score = 0.6;
            } else { // Sudden decrease in wpsi
                right_thrust_score = 0.6;
            }
        }
    }
    
    // Add fault detection criteria based on commanded vs. actual thrust difference
    // If one thruster is commanded much higher than the other but the vessel isn't turning 
    // as expected, this indicates a fault
    if (std::abs(Tp.data - Ts.data) > 50.0) { // Significant thrust differential
        // Expected strong yaw rate
        double expected_turn_rate = 0.5; // Approximate, should be calibrated
        
        // If actual turn rate is much less than expected, suspect a fault
        if (std::abs(local_pos.r) < 0.2 * expected_turn_rate) {
            might_be_fault = true;
            
            // Determine which thruster is likely failing based on commands
            if (Tp.data > Ts.data && local_pos.r > -0.1) {
                right_thrust_score = 0.7; // Right thrust should cause left turn but isn't
            } 
            else if (Ts.data > Tp.data && local_pos.r < 0.1) {
                left_thrust_score = 0.7; // Left thrust should cause right turn but isn't
            }
        }
    }
    
    // Detection based on commanded thrust vs. actual disturbance
    // This works with asymmetric commands
    if (Tp.data > 50.0 && Ts.data > 50.0) { // Both thrusters commanded
        // If wpsi is changing rapidly in a direction inconsistent with commands,
        // it suggests a fault
        if (Tp.data >= Ts.data && wpsi_trend < -0.5) {
            // Left thruster should cause right turn (negative wpsi),
            // but if trend is strongly negative, suspect left thruster issue
            might_be_fault = true;
            left_thrust_score = 0.7;
        }
        else if (Ts.data >= Tp.data && wpsi_trend > 0.5) {
            // Right thruster should cause left turn (positive wpsi),
            // but if trend is strongly positive, suspect right thruster issue
            might_be_fault = true;
            right_thrust_score = 0.7;
        }
    }
    
    // Special case for zero-commanded thrust with significant disturbance
    if (Tp.data < 10.0 && Ts.data < 10.0 && std::abs(wpsi) > wpsi_threshold) {
        // If no thrust is commanded but significant disturbance exists,
        // this indicates unmodeled dynamics, not a fault
        might_be_fault = false;
    }
    
    // Add additional criteria based on thrust commands vs. resulting motion
    if (might_be_fault) {
        // LEFT thrust issue detection based on trend and commands
        if (wpsi_change > 3.0 || (wpsi_trend > 0.2 && Tp.data > 50.0)) {
            // Strong positive trend indicates left thruster weakening
            left_thrust_score = 0.7 + 0.3 * std::min(std::abs(wpsi_change / 5.0), 1.0);
            
            if (left_thrust_score > 0.7) {
                std::cout << "\033[1;33m" << "LEFT THRUST FAILURE DETECTED BY TREND: wpsi_change = " 
                          << wpsi_change << ", wpsi_trend = " << wpsi_trend << "\033[0m" << std::endl;
            }
        }
        
        // RIGHT thrust issue detection based on trend and commands
        if (wpsi_change < -3.0 || (wpsi_trend < -0.2 && Ts.data > 50.0)) {
            // Strong negative trend indicates right thruster weakening
            right_thrust_score = 0.7 + 0.3 * std::min(std::abs(wpsi_change / 5.0), 1.0);
            
            if (right_thrust_score > 0.7) {
                std::cout << "\033[1;33m" << "RIGHT THRUST FAILURE DETECTED BY TREND: wpsi_change = " 
                          << wpsi_change << ", wpsi_trend = " << wpsi_trend << "\033[0m" << std::endl;
            }
        }
        
        // Scale fault scores by command magnitude to avoid false positives during low thrust
        left_thrust_score *= std::min(1.0, Tp.data / 100.0);
        right_thrust_score *= std::min(1.0, Ts.data / 100.0);
        
        // Find the highest score
        double max_score = std::max({left_thrust_score, right_thrust_score, left_angle_score, right_angle_score});
        
        // Determine the most likely fault type
        if (max_score > fault_model.detect_threshold) {
            if (max_score == left_thrust_score) {
                fault_type = LEFT_THRUST_FAILURE;
                confidence = max_score;
                return true;
            } else if (max_score == right_thrust_score) {
                fault_type = RIGHT_THRUST_FAILURE;
                confidence = max_score;
                return true;
            } else if (max_score == left_angle_score) {
                fault_type = LEFT_ANGLE_FAILURE;
                confidence = max_score;
                return true;
            } else if (max_score == right_angle_score) {
                fault_type = RIGHT_ANGLE_FAILURE;
                confidence = max_score;
                return true;
            }
        }
    }
    
    // If we got here, no fault was detected
    fault_type = NO_FAULT;
    confidence = 0.0;
    return false;
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
void WAMV_MPC::publishFaultDiagnosis(int fault_type, double confidence) 
{
    auto message = std::make_unique<std_msgs::msg::String>();
    std::string fault_str;
    
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
        case LEFT_ANGLE_FAILURE:
            fault_str = "LEFT_ANGLE_FAILURE";
            break;
        case RIGHT_ANGLE_FAILURE:
            fault_str = "RIGHT_ANGLE_FAILURE";
            break;
        default:
            fault_str = "UNKNOWN_FAULT";
    }
    
    message->data = "Fault: " + fault_str + " (Confidence: " + 
                   std::to_string(confidence * 100.0) + "%)";
    fault_diagnosis_pub->publish(*message);
    
    // Add more detailed logging for debugging
    RCLCPP_INFO(this->get_logger(), "FAULT DIAGNOSIS: %s (Confidence: %.2f%%)", 
               fault_str.c_str(), confidence * 100.0);
               
    // Print additional debug info to console
    std::cout << "\033[1;36m" << "FAULT DIAGNOSIS: " << fault_str 
              << " (Confidence: " << std::fixed << std::setprecision(2) 
              << confidence * 100.0 << "%)" << "\033[0m" << std::endl;
              
    // Print current disturbance values and thrusts
    std::cout << "\033[1;36m" << "  Disturbances - w_x: " << esti_x[6] 
              << ", w_y: " << esti_x[7] << ", w_psi: " << esti_x[8] << "\033[0m" << std::endl;
              
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

