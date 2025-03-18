#include "vrx_control/wamv_dashboard.h"
#include <QApplication>
#include <memory>

int main(int argc, char *argv[])
{
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    // Initialize Qt
    QApplication app(argc, argv);
    
    // Create dashboard node
    auto node = std::make_shared<WAMVDashboardNode>();
    
    // Create dashboard window with node
    auto dashboard = new WAMVDashboard(node);
    
    // Set dashboard in node
    node->setDashboard(dashboard);
    
    // Create executor for ROS2 callbacks
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    
    // Use a separate thread for ROS2 callbacks
    std::thread ros_thread([&executor]() {
        executor.spin();
    });
    
    // Show dashboard window
    dashboard->show();
    
    // Run Qt event loop
    int result = app.exec();
    
    // Clean up
    rclcpp::shutdown();
    if (ros_thread.joinable()) {
        ros_thread.join();
    }
    
    return result;
}