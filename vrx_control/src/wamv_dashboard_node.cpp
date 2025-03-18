#include "vrx_control/wamv_dashboard.h"
#include <QApplication>
#include <memory>

int main(int argc, char *argv[])
{
    std::cout << "Starting application..." << std::endl;
    
    try {
        // Initialize ROS2
        std::cout << "Initializing ROS2..." << std::endl;
        rclcpp::init(argc, argv);
        
        // Initialize Qt
        std::cout << "Initializing Qt..." << std::endl;
        QApplication app(argc, argv);
        
        // Create dashboard node
        std::cout << "Creating dashboard node..." << std::endl;
        auto node = std::make_shared<WAMVDashboardNode>();
        
        // Create dashboard window with node
        std::cout << "Creating dashboard window..." << std::endl;
        auto dashboard = new WAMVDashboard(node);
        
        // Set dashboard in node
        std::cout << "Setting dashboard in node..." << std::endl;
        node->setDashboard(dashboard);
        
        // Create executor for ROS2 callbacks
        std::cout << "Creating executor..." << std::endl;
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        
        // Use a separate thread for ROS2 callbacks
        std::cout << "Starting ROS thread..." << std::endl;
        std::thread ros_thread([&executor]() {
            executor.spin();
        });
        
        // Show dashboard window
        std::cout << "Showing dashboard window..." << std::endl;
        dashboard->show();
        
        std::cout << "Running Qt event loop..." << std::endl;
        int result = app.exec();
        std::cout << "Qt event loop ended with result: " << result << std::endl;
        
        // Clean up
        std::cout << "Shutting down ROS2..." << std::endl;
        rclcpp::shutdown();
        if (ros_thread.joinable()) {
            std::cout << "Joining ROS thread..." << std::endl;
            ros_thread.join();
        }
        
        std::cout << "Application exiting with result: " << result << std::endl;
        return result;
    } catch (const std::exception& e) {
        std::cerr << "Exception in main: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Unknown exception in main" << std::endl;
        return 1;
    }
}