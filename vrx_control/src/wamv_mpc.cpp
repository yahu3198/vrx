#include <vrx_control/wamv_mpc.h>

WAMV_MPC::WAMV_MPC() 
: Node("wamv_mpc_node")
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

    acados_out.u0[0] = 0;
    acados_out.u0[1] = 0;
    acados_out.u0[2] = 0;
    acados_out.u0[3] = 0;
    publish_cin(acados_out.u0[0], acados_out.u0[1], acados_out.u0[2], acados_out.u0[3]);
    
    solver_param.Tp_pre = acados_out.u0[0];
    solver_param.Ts_pre = acados_out.u0[1];
    solver_param.delta_p_pre = acados_out.u0[2];
    solver_param.delta_s_pre = acados_out.u0[3];

    double current_time = rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds();
    double z[4];
        ocp_nlp_out_get(mpc_capsule->nlp_config, mpc_capsule->nlp_dims, mpc_capsule->nlp_out, 0, "z", z);
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
        std::cout << "---------------------------------------------------------------------------------------------------------------------" << std::endl;
        cout_counter = 0;
    }
    else{
        cout_counter++;
    }
}

void WAMV_MPC::publish_cin(double Tp_mpc, double Ts_mpc, double delta_p_mpc, double delta_s_mpc)
{

    // publish control inputs
    Tp.data = Tp_mpc;
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