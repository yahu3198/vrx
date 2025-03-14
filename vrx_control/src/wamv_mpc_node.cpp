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

    // Set start time and duration for the operation
    // rclcpp::Time start_time = rclcpp::Clock().now();
    // rclcpp::Duration duration(50.0); // Set the desired duration to 50 seconds

    // Delay for 5 seconds before starting the loop
    // rclcpp::sleep_for(std::chrono::seconds(5));

    // Main loop for ROS 2 node
    while (rclcpp::ok()) {
        // rclcpp::Time current_time = rclcpp::Clock().now();
        // rclcpp::Duration elapsed_time = current_time - start_time;

        // Optional: Check if elapsed time exceeds the desired duration
        // if (elapsed_time.seconds() >= duration.seconds()) {
        //     RCLCPP_INFO(wm_node->get_logger(), "Reached 50 seconds. Stopping the program.");
        //     break;
        // }

        // Call solve() if the condition is met
        if (wm_node->is_start == true) {
            wm_node->EKF();
            // wm_node->solve();
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
