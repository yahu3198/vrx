#include <wamv_mpc/wamv_mpc.h>

WAMV_MPC::WAMV_MPC(ros::NodeHandle& nh)
{
    // read parameter
    nh.getParam("/bluerov2_dob_node/read_wrench",READ_WRENCH);
    nh.getParam("/bluerov2_dob_node/compensate_d",COMPENSATE_D);
    nh.getParam("/bluerov2_dob_node/ref_traj", REF_TRAJ);
    // nh.getParam("/bluerov2_dob_node/applied_forcex", WRENCH_FX);
    // nh.getParam("/bluerov2_dob_node/applied_forcey", WRENCH_FY);
    // nh.getParam("/bluerov2_dob_node/applied_forcez", WRENCH_FZ);
    // nh.getParam("/bluerov2_dob_node/applied_torquez", WRENCH_TZ);
    // nh.getParam("/bluerov2_dob_node/disturbance_x", solver_param.disturbance_x);
    // nh.getParam("/bluerov2_dob_node/disturbance_y", solver_param.disturbance_y);
    // nh.getParam("/bluerov2_dob_node/disturbance_z", solver_param.disturbance_z);
    // nh.getParam("/bluerov2_dob_node/disturbance_phi", solver_param.disturbance_phi);
    // nh.getParam("/bluerov2_dob_node/disturbance_theta", solver_param.disturbance_theta);
    // nh.getParam("/bluerov2_dob_node/disturbance_psi", solver_param.disturbance_psi);
    
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

    // initialize
    is_start = false;
}

// quaternion to euler angle
WAMV_MPC::Euler WAMV_MPC::q2rpy(const geometry_msgs::Quaternion& quaternion)
{
    tf::Quaternion tf_quaternion;
    Euler euler;
    tf::quaternionMsgToTF(quaternion,tf_quaternion);
    tf::Matrix3x3(tf_quaternion).getRPY(euler.phi, euler.theta, euler.psi);
    return euler;
}

// euler angle to quaternion
geometry_msgs::Quaternion WAMV_MPC::rpy2q(const Euler& euler)
{
    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromRollPitchYaw(euler.phi, euler.theta, euler.psi);
    return quaternion;
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
    linear_vel_inertial.u = msg->twist[17].linear.x;
    linear_vel_inertial.v = msg->twist[17].linear.y;
    linear_vel_inertial.w = msg->twist[17].linear.z;

    // get angular vel p q r
    angular_vel_inertial.p = msg->twist[17].angular.x;
    angular_vel_inertial.q = msg->twist[17].angular.y;
    angular_vel_inertial.r = msg->twist[17].angular.z;

    // get angle phi, theta, psi
    tf::quaternionMsgToTF(msg->pose[17].orientation, tf_quaternion);
    // Normalize the quaternion
    if (tf_quaternion.length() != 0) {
        tf_quaternion.normalize();
    }
    // extract roll, pitch, and yaw
    tf::Matrix3x3(tf_quaternion).getRPY(local_euler.phi, local_euler.theta, local_euler.psi);
    
}

void WAMV_MPC::solve()
{
    if(cout_counter > 2){
        std::cout << "---------------------------------------------------------------------------------------------------------------------" << std::endl;
        std::cout << "pos_x:  " << local_pos.x << "  pos_y:  " << local_pos.y << "  pos_z:  " << local_pos.z << std::endl;
        std::cout << "phi:  " << local_euler.phi << "  theta:  " << local_euler.theta << "  psi:  " << local_euler.psi << std::endl;
        std::cout << "vel_x:  " << linear_vel_inertial.u << "  vel_y:  " << linear_vel_inertial.v << "  vel_z:  " << linear_vel_inertial.w << std::endl;
        std::cout << "vel_p:  " << angular_vel_inertial.p << "  vel_q:  " << angular_vel_inertial.q << "  vel_r:  " << angular_vel_inertial.r << std::endl;
        std::cout << "---------------------------------------------------------------------------------------------------------------------" << std::endl;
        cout_counter = 0;
    }
    else{
        cout_counter++;
    }
}