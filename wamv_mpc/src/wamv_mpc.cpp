#include <wamv_mpc/wamv_mpc.h>

WAMV_MPC::WAMV_MPC(ros::NodeHandle& nh)
{
    // read parameter
    nh.getParam("/wamv_mpc_node/read_wrench",READ_WRENCH);
    nh.getParam("/wamv_mpc_node/compensate_d",COMPENSATE_D);
    nh.getParam("/wamv_mpc_node/ref_traj", REF_TRAJ);
    
    // Pre-load the trajectory
    const char * c = REF_TRAJ.c_str();
	number_of_steps = readDataFromFile(c, trajectory);
	if (number_of_steps == 0){
		ROS_WARN("Cannot load CasADi optimal trajectory!");
	}
	else{
		ROS_INFO_STREAM("Number of steps of selected trajectory: " << number_of_steps << std::endl);
	}

    // Initialize MPC
    int create_status = 1;
    create_status = wamv_acados_create(mpc_capsule);
    if (create_status != 0){
        ROS_INFO_STREAM("acados_create() returned status " << create_status << ". Exiting." << std::endl);
        exit(1);
    }

    // ros subsriber & publisher
    states_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 20, &WAMV_MPC::states_cb, this);
    left_thrust_angle_pub = nh.advertise<std_msgs::Float32>("/wamv/thrusters/left_thrust_angle", 20);
    left_thrust_cmd_pub = nh.advertise<std_msgs::Float32>("/wamv/thrusters/left_thrust_cmd", 20);
    right_thrust_angle_pub = nh.advertise<std_msgs::Float32>("/wamv/thrusters/right_thrust_angle", 20);
    right_thrust_cmd_pub = nh.advertise<std_msgs::Float32>("/wamv/thrusters/right_thrust_cmd", 20);
    ref_pose_pub = nh.advertise<nav_msgs::Odometry>("/wamv/ref_pose",20);
    error_pose_pub = nh.advertise<nav_msgs::Odometry>("/wamv/error_pose",20);
    pose_gt_pub = nh.advertise<nav_msgs::Odometry>("/wamv/pose_gt",20);
    control_inputs_pub = nh.advertise<geometry_msgs::TwistStamped>("/wamv/control_inputs", 20);

    // initialize
    for(unsigned int i=0; i < WAMV_NU; i++) acados_out.u0[i] = 0.0;
    for(unsigned int i=0; i < WAMV_NX; i++) acados_in.x0[i] = 0.0;
    is_start = false;
}

// subscribe pos and vel
void WAMV_MPC::states_cb(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    is_start = true;
    // get linear position x y z
    local_pos.x = msg->pose[17].position.x;
    local_pos.y = msg->pose[17].position.y;
    local_pos.z = msg->pose[17].position.z;

    // get linear vel u v w
    local_pos.u = msg->twist[17].linear.x;
    local_pos.v = msg->twist[17].linear.y;
    local_pos.w = msg->twist[17].linear.z;

    // get angular vel p q r
    local_pos.p = msg->twist[17].angular.x;
    local_pos.q = msg->twist[17].angular.y;
    local_pos.r = msg->twist[17].angular.z;

    // get angle phi, theta, psi
    tf::quaternionMsgToTF(msg->pose[17].orientation, tf_quaternion);
    // Normalize the quaternion
    if (tf_quaternion.length() != 0) {
        tf_quaternion.normalize();
    }
    // extract roll, pitch, and yaw
    tf::Matrix3x3(tf_quaternion).getRPY(local_euler.phi, local_euler.theta, local_euler.psi);
    
    v_inertial << local_pos.u, local_pos.v, local_pos.r;
    R_ib << cos(local_euler.psi), -sin(local_euler.psi), 0,
            sin(local_euler.psi), cos(local_euler.psi), 0,
            0, 0, 1;
    v_body = R_ib.inverse()*v_inertial;

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
        for (unsigned int i = 0; i < number_of_steps-line_to_read; i++)    // Fill part of horizon with file data
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
    if (pre_yaw >= 0 && local_euler.psi >=0)
    {
        yaw_diff = local_euler.psi - pre_yaw;
    }
    else if (pre_yaw >= 0 && local_euler.psi <0)
    {
        if (2*M_PI+local_euler.psi-pre_yaw >= pre_yaw+abs(local_euler.psi))
        {
            yaw_diff = -(pre_yaw + abs(local_euler.psi));
        }
        else
        {
            yaw_diff = 2 * M_PI + local_euler.psi - pre_yaw;
        }
    }
    else if (pre_yaw < 0 && local_euler.psi >= 0)
    {
        if (2*M_PI-local_euler.psi+pre_yaw >= abs(pre_yaw)+local_euler.psi)
        {
            yaw_diff = abs(pre_yaw)+local_euler.psi;
        }
        else
        {
            yaw_diff = -(2*M_PI-local_euler.psi+pre_yaw);
        }
    }
    else
    {
        yaw_diff = local_euler.psi - pre_yaw;
    }

    yaw_sum = yaw_sum + yaw_diff;
    pre_yaw = local_euler.psi;

    // set initial states
    acados_in.x0[x] = local_pos.x;
    acados_in.x0[y] = local_pos.y;
    acados_in.x0[psi] = yaw_sum;
    acados_in.x0[u] = v_body[0];
    acados_in.x0[v] = v_body[1];
    acados_in.x0[r] = v_body[2];
    ocp_nlp_constraints_model_set(mpc_capsule->nlp_config,mpc_capsule->nlp_dims,mpc_capsule->nlp_in, 0, "lbx", acados_in.x0);
    ocp_nlp_constraints_model_set(mpc_capsule->nlp_config,mpc_capsule->nlp_dims,mpc_capsule->nlp_in, 0, "ubx", acados_in.x0);

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
        ROS_INFO_STREAM("acados returned status " << acados_status << std::endl);
    }

    acados_out.status = acados_status;
    acados_out.kkt_res = (double)mpc_capsule->nlp_out->inf_norm_res;

    ocp_nlp_get(mpc_capsule->nlp_config, mpc_capsule->nlp_solver, "time_tot", &acados_out.cpu_time);

    ocp_nlp_out_get(mpc_capsule->nlp_config, mpc_capsule->nlp_dims, mpc_capsule->nlp_out, 0, "u", (void *)acados_out.u0);

    publish_cin(acados_out.u0[0], acados_out.u0[1], acados_out.u0[2], acados_out.u0[3]);

    // test control allocation
    // x
    // publish_cin(240, 240, 0,0);

    // y
    // publish_cin(240, 240, 1.57,1.57);

    // yaw
    // publish_cin(240, -100, -1.57, -1.57);

    if(cout_counter > 2){
        std::cout << "---------------------------------------------------------------------------------------------------------------------" << std::endl;
        std::cout << "ref_x:    " << acados_in.yref[0][0] << "\tref_y:   " << acados_in.yref[0][1] << "\tref_yaw:    " << acados_in.yref[0][2] << std::endl;
        std::cout << "error_x:  " << error_pose.pose.pose.position.x << "  error_y:  " << error_pose.pose.pose.position.y << "  error_psi:  " << yaw_error << std::endl;
        std::cout << "pos_x:  " << local_pos.x << "  pos_y:  " << local_pos.y << "  psi:  " << yaw_sum << std::endl;
        // std::cout << "phi:  " << local_euler.phi << "  theta:  " << local_euler.theta << "  psi:  " << yaw_sum << std::endl;
        std::cout << "vel_x:  " << local_pos.u << "  vel_y:  " << local_pos.v << "  vel_z:  " << local_pos.w << std::endl;
        std::cout << "vel_p:  " << local_pos.p << "  vel_q:  " << local_pos.q << "  vel_r:  " << local_pos.r << std::endl;
        std::cout << "Tp:  " << acados_out.u0[0] << "  Ts:  " << acados_out.u0[1] << "  delta_p:  " << acados_out.u0[2] << "  delta_s:  " << acados_out.u0[3] << std::endl;
        std::cout << "Tp_cmd  " << Tp_cmd << "  Ts_cmd:  " << Ts_cmd << std::endl;
        std::cout << "solve_time: "<< acados_out.cpu_time << "\tkkt_res: " << acados_out.kkt_res << "\tacados_status: " << acados_out.status << std::endl;
        std::cout << "ros_time:   " << std::fixed << ros::Time::now().toSec() << std::endl;
        std::cout << "---------------------------------------------------------------------------------------------------------------------" << std::endl;
        cout_counter = 0;
    }
    else{
        cout_counter++;
    }
}

void WAMV_MPC::publish_cin(double Tp, double Ts, double delta_p, double delta_s)
{
    Tp_cmd = 0.1;
    Ts_cmd = 0.1;
    if (Tp > 1.21)
    {
        Tp_cmd = thrustToCmd(Tp, 0.01, 59.82, 5.0, 0.38, 0.56, 0.28);
    }
    else if (Tp < 0.061)
    {
        Tp_cmd = thrustToCmd(Tp, -199.13, -0.09, 8.84, 5.34, 0.99, -0.57);
    }
    if (Ts > 1.21)
    {
        Ts_cmd = thrustToCmd(Ts, 0.01, 59.82, 5.0, 0.38, 0.56, 0.28);
    }
    else if (Ts < 0.061)
    {
        Ts_cmd = thrustToCmd(Tp, -199.13, -0.09, 8.84, 5.34, 0.99, -0.57);
    }
    left_thrust_angle.data = delta_p;
    left_thrust_cmd.data = Tp_cmd;
    right_thrust_angle.data = delta_s;
    right_thrust_cmd.data = Ts_cmd;

    // publish control inputs
    left_thrust_angle_pub.publish(left_thrust_angle);
    left_thrust_cmd_pub.publish(left_thrust_cmd);
    right_thrust_angle_pub.publish(right_thrust_angle);
    right_thrust_cmd_pub.publish(right_thrust_cmd);

    geometry_msgs::TwistStamped control_inputs_msg;
    control_inputs.header.stamp = ros::Time::now();
    control_inputs.twist.linear.x = delta_p;
    control_inputs.twist.linear.y = Tp_cmd;
    control_inputs.twist.angular.x = delta_s;
    control_inputs.twist.angular.y = Ts_cmd;
    control_inputs_pub.publish(control_inputs);

    // publish reference states
    tf2::Quaternion quat;
    quat.setRPY(0, 0, yaw_ref);
    geometry_msgs::Quaternion quat_msg;
    tf2::convert(quat, quat_msg);
    ref_pose.pose.pose.position.x = acados_in.yref[0][0];
    ref_pose.pose.pose.position.y = acados_in.yref[0][1];
    ref_pose.pose.pose.orientation.x = quat_msg.x;
    ref_pose.pose.pose.orientation.y = quat_msg.y;
    ref_pose.pose.pose.orientation.z = quat_msg.z;
    ref_pose.pose.pose.orientation.w = quat_msg.w;
    
    ref_pose.header.stamp = ros::Time::now();
    ref_pose.header.frame_id = "odom_frame";
    ref_pose.child_frame_id = "base_link";
    ref_pose_pub.publish(ref_pose);

    // publish error states
    tf2::Quaternion quat_error;
    yaw_error = yaw_sum - acados_in.yref[0][2];
    quat_error.setRPY(0, 0, yaw_error);
    geometry_msgs::Quaternion quat_error_msg;
    tf2::convert(quat_error, quat_error_msg);
    error_pose.pose.pose.position.x = acados_in.x0[0] - acados_in.yref[0][0];
    error_pose.pose.pose.position.y = acados_in.x0[1] - acados_in.yref[0][1];
    error_pose.pose.pose.orientation.x = quat_error_msg.x;
    error_pose.pose.pose.orientation.y = quat_error_msg.y;
    error_pose.pose.pose.orientation.z = quat_error_msg.z;
    error_pose.pose.pose.orientation.w = quat_error_msg.w;
    error_pose.header.stamp = ros::Time::now();
    error_pose.header.frame_id = "odom_frame";
    error_pose.child_frame_id = "base_link";

    error_pose_pub.publish(error_pose);

    // publish pose_gt
    tf2::Quaternion quat_gt;
    quat_gt.setRPY(local_euler.phi, local_euler.theta, local_euler.psi);
    geometry_msgs::Quaternion quat_gt_msg;
    tf2::convert(quat_gt, quat_gt_msg);
    pose_gt.pose.pose.position.x = local_pos.x;
    pose_gt.pose.pose.position.y = local_pos.y;
    pose_gt.pose.pose.orientation.x = quat_gt_msg.x;
    pose_gt.pose.pose.orientation.y = quat_gt_msg.y;
    pose_gt.pose.pose.orientation.z = quat_gt_msg.z;
    pose_gt.pose.pose.orientation.w = quat_gt_msg.w;
    
    pose_gt.header.stamp = ros::Time::now();
    pose_gt.header.frame_id = "odom_frame";
    pose_gt.child_frame_id = "base_link";
    pose_gt_pub.publish(pose_gt);

}

double WAMV_MPC::thrustToCmd(double glf_T, double glf_A, double glf_K, double glf_B, double glf_v, double glf_C, double glf_M)
{
    double term = (glf_K - glf_A) / (glf_T - glf_A);
    double exponent = std::pow(term, glf_v) - glf_C;
    // Check if the exponent is positive before applying log
    // if (exponent <= 0) 
    // {
    //     throw std::runtime_error("Invalid input: log argument must be positive.");
    // }

    double cmd = glf_M - (1.0 / glf_B) * std::log(exponent);
    return cmd;
}