#ifndef WAMV_DASHBOARD_H
#define WAMV_DASHBOARD_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <QMainWindow>
#include <QTimer>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QScatterSeries>
#include <QtCharts/QValueAxis>
#include <QLabel>
#include <QPushButton>
#include <QGridLayout>
#include <QFrame>
#include <QColor>
#include <deque>
#include <vector>

QT_CHARTS_USE_NAMESPACE

// Forward declare the ROS Node class
class WAMVDashboardNode;

class WAMVDashboard : public QMainWindow
{
    Q_OBJECT

public:
    explicit WAMVDashboard(std::shared_ptr<WAMVDashboardNode> node_ptr);
    virtual ~WAMVDashboard();

    // Callback handlers
    void handleDisturbanceMsg(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void handleOdomMsg(const nav_msgs::msg::Odometry::SharedPtr msg);
    void handleFaultDiagnosisMsg(const std_msgs::msg::String::SharedPtr msg);
    void handleFaultFeaturesMsg(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void handleWpsiCoefficientMsg(const std_msgs::msg::Float64::SharedPtr msg);
    void updateTimeAxis(QChart *chart, double min_time, double max_time);
    void updateValueAxis(QChart *chart, const std::deque<double> &data);

private slots:
    void updatePlots();
    void resetTrajectory();
    

private:
    double start_time_ = 0.0;  // Time reference
    // ROS2 node pointer
    std::shared_ptr<WAMVDashboardNode> node_ptr_;
    
    // Data storage
    std::deque<double> wx_data_;
    std::deque<double> wy_data_;
    std::deque<double> wpsi_data_;
    std::deque<double> wpsi_calibrated_data_;
    std::deque<double> time_data_;
    std::vector<double> trajectory_x_;
    std::vector<double> trajectory_y_;
    
    // Current fault status
    std::string current_fault_status_;
    double fault_confidence_;
    
    // Qt UI components
    QChart *wx_chart_;
    QChartView *wx_view_;
    QLineSeries *wx_series_;
    
    QChart *wy_chart_;
    QChartView *wy_view_;
    QLineSeries *wy_series_;
    
    QChart *wpsi_chart_;
    QChartView *wpsi_view_;
    QLineSeries *wpsi_series_;
    QLineSeries *wpsi_calibrated_series_;
    
    QChart *trajectory_chart_;
    QChartView *trajectory_view_;
    QScatterSeries *trajectory_series_;
    
    QLabel *fault_status_label_;
    QLabel *confidence_label_;
    QFrame *status_indicator_;
    QPushButton *reset_button_;
    
    QTimer *update_timer_;
    
    // Configuration
    const int MAX_DATA_POINTS = 1000;  // Maximum number of points to display in time series
    const int UPDATE_INTERVAL_MS = 100;  // Update interval in milliseconds
    
    // Helper functions
    void setupUI();
    void setupCharts();
    void updateStatusDisplay();
    QColor getFaultStatusColor();
};

// The ROS Node class - separate from the Qt UI class
class WAMVDashboardNode : public rclcpp::Node
{
public:
    explicit WAMVDashboardNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    virtual ~WAMVDashboardNode() = default;

    // Set the UI pointer
    void setDashboard(WAMVDashboard* dashboard);

private:
    // ROS2 subscribers
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr disturbance_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fault_diagnosis_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr fault_features_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr wpsi_coefficient_sub_;

    // Pointer to the dashboard UI
    WAMVDashboard* dashboard_;
};

#endif // WAMV_DASHBOARD_H