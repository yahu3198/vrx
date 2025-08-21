#include <rclcpp/rclcpp.hpp>
#include "vrx_control/wamv_mpc.h"

int main(int argc, char **argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create the ROS 2 node
    auto wm_node = std::make_shared<WAMV_MPC>();

    // Set loop rate 
    rclcpp::Rate loop_rate(100); 
    // Counter to throttle solve() to 20 Hz
    int iteration_count = 0;
    const int solve_frequency = 5; // 100 Hz / 20 Hz = 5 iterations
    // const int fault_diagnosis_frequency = 10; // 100 Hz / 10 Hz = 10 iterations

    // Main loop for ROS 2 node
    while (rclcpp::ok()) {
        // Call solve() if the condition is met
        if (wm_node->is_start == true) {
            wm_node->EKF();
            
            // // Run calibration at every iteration (it has its own throttling)
            // wm_node->calibrateDisturbanceModel();
            
            // // Run fault diagnosis at 10 Hz
            // if (iteration_count % fault_diagnosis_frequency == 0) {
            //     wm_node->updateFaultModel();
            // }
            
            // Run MPC at 20 Hz
            if (iteration_count % solve_frequency == 0) {
                wm_node->solve();
            }
        }

        // Increment counter
        iteration_count++;

        // Spin the node (to process callbacks if needed)
        rclcpp::spin_some(wm_node);

        // Sleep to maintain loop rate
        loop_rate.sleep();
    }

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
