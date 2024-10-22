#include <ros/ros.h>

#include "wamv_mpc/wamv_mpc.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "wamv_mpc_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(20);

    ros::Time start_time = ros::Time::now();
    ros::Duration duration(50.0); // Set the desired duration to 25 seconds

    WAMV_MPC wm(nh);
    while(ros::ok()){
        ros::Time current_time = ros::Time::now();
        ros::Duration elapsed_time = current_time - start_time;

        // if (elapsed_time >= duration)
        // {
        //     ROS_INFO("Reached 25 seconds. Stopping the program.");
        //     break;
        // }
        
        if(wm.is_start==true)
        {
            wm.solve();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}