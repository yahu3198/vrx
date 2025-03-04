#include <vrx_control/wamv_mpc.h>

WAMV_MPC::WAMV_MPC() 
: Node("wamv_mpc_node")
{
    // Declare parameters with default values
    this->declare_parameter<int>("read_wrench", 0);
    this->declare_parameter<bool>("compensate_d", false);
    this->declare_parameter<std::string>("ref_traj", "");
    // this->declare_parameter<double>("Tp_pre", 0.0);
    // this->declare_parameter<double>("Ts_pre", 0.0);
    // this->declare_parameter<double>("delta_p_pre", 0.0);
    // this->declare_parameter<double>("delta_s_pre", 0.0);

    // Get parameters
    this->get_parameter("read_wrench", READ_WRENCH);
    this->get_parameter("compensate_d", COMPENSATE_D);
    this->get_parameter("ref_traj", REF_TRAJ);
    // this->get_parameter("Tp_pre", solver_param.Tp_pre);
    // this->get_parameter("Ts_pre", solver_param.Ts_pre);
    // this->get_parameter("delta_p_pre", solver_param.delta_p_pre);
    // this->get_parameter("delta_s_pre", solver_param.delta_s_pre);
    
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
        10,
        std::bind(&WAMV_MPC::states_cb, this, std::placeholders::_1));
    
    left_thrust_angle_pub = this->create_publisher<std_msgs::msg::Float64>(
        "/wamv/thrusters/left/pos", 10);
    left_thrust_cmd_pub = this->create_publisher<std_msgs::msg::Float64>(
        "/wamv/thrusters/left/thrust", 10);
    right_thrust_angle_pub = this->create_publisher<std_msgs::msg::Float64>(
        "/wamv/thrusters/right/pos", 10);
    right_thrust_cmd_pub = this->create_publisher<std_msgs::msg::Float64>(
        "/wamv/thrusters/right/thrust", 10);
    ref_pose_pub = this->create_publisher<nav_msgs::msg::Odometry>(
        "/wamv/ref_pose", 10);
    error_pose_pub = this->create_publisher<nav_msgs::msg::Odometry>(
        "/wamv/error_pose", 10);
    control_inputs_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/wamv/control_inputs", 10);

    // initialize
    for(unsigned int i=0; i < WAMV_NU; i++) acados_out.u0[i] = 0.0;
    for(unsigned int i=0; i < WAMV_NX; i++) acados_in.x0[i] = 0.0;
    start_time = rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds();
    is_start = false;
    solver_param.Tp_pre = 0;
    solver_param.Ts_pre = 0;
    solver_param.delta_p_pre = 0;
    solver_param.delta_s_pre = 0;
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
    acados_status = wamv_acados_solve(mpc_capsule);

    if (acados_status != 0){
        RCLCPP_INFO(this->get_logger(), "acados returned status: %d", acados_status);
    }

    acados_out.status = acados_status;
    acados_out.kkt_res = (double)mpc_capsule->nlp_out->inf_norm_res;

    // ocp_nlp_get(mpc_capsule->nlp_config, mpc_capsule->nlp_solver, "time_tot", &acados_out.cpu_time);
    ocp_nlp_get(mpc_capsule->nlp_solver, "time_tot", &acados_out.cpu_time);

    ocp_nlp_out_get(mpc_capsule->nlp_config, mpc_capsule->nlp_dims, mpc_capsule->nlp_out, 0, "u", (void *)acados_out.u0);

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
        std::cout << "phi:  " << local_pos.phi << "  theta:  " << local_pos.theta << "  psi:  " << local_pos.psi << std::endl;
        std::cout << "vel_x:  " << local_pos.u << "  vel_y:  " << local_pos.v << "  vel_z:  " << local_pos.w << std::endl;
        std::cout << "vel_p:  " << local_pos.p << "  vel_q:  " << local_pos.q << "  vel_r:  " << local_pos.r << std::endl;
        std::cout << "Tp:  " << acados_out.u0[0] << "  Ts:  " << acados_out.u0[1] << "  delta_p:  " << acados_out.u0[2] << "  delta_s:  " << acados_out.u0[3] << std::endl;
        // std::cout << "Tp_cmd  " << Tp.data << "  Ts_cmd:  " << Ts.data << std::endl;
        std::cout << "z:    " << z << std::endl;
        std::cout << "solve_time: "<< acados_out.cpu_time << "\tkkt_res: " << acados_out.kkt_res << "\tacados_status: " << acados_out.status << std::endl;
        std::cout << "relative_time: " << std::fixed << (current_time - start_time) << std::endl;
        std::cout << "---------------------------------------------------------------------------------------------------------------------" << std::endl;
        cout_counter = 0;
        // double z[4];
        // ocp_nlp_out_get(mpc_capsule->nlp_config, mpc_capsule->nlp_dims, mpc_capsule->nlp_out, 0, "z", z);
        // RCLCPP_INFO(this->get_logger(), "z: [%f, %f, %f, %f]", z[0], z[1], z[2], z[3]);
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


}