#ifndef WAMV_DASHBOARD_H
#define WAMV_DASHBOARD_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <QMainWindow>
#include <QTimer>
#include <QLabel>
#include <QPushButton>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFrame>
#include <QGroupBox>
#include <QProgressBar>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsItem>
#include <QGraphicsPolygonItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsLineItem>
#include <QGraphicsTextItem>
#include <QColor>
#include <QPen>
#include <QBrush>
#include <QPolygonF>
#include <QPointF>
#include <vector>
#include <cmath>

// Forward declare the ROS Node class
class WAMVDashboardNode;

// Custom graphics item for USV with heading
class USVGraphicsItem : public QGraphicsEllipseItem
{
public:
    USVGraphicsItem(double x, double y, double heading, QGraphicsItem* parent = nullptr);
    void updatePosition(double x, double y, double heading);
    void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;

private:
    double heading_;
    QGraphicsLineItem* heading_line_;
};

// Custom graphics item for environmental force arrows
class ForceArrowItem : public QGraphicsLineItem
{
public:
    ForceArrowItem(double start_x, double start_y, double force_x, double force_y, 
                   const QString& label, QGraphicsItem* parent = nullptr);
    void updateForce(double start_x, double start_y, double force_x, double force_y);
    void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;
    void setActive(bool active);

private:
    QString label_;
    QGraphicsTextItem* label_item_;
    bool is_active_;
    double arrow_scale_;
};

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
    void handleOperationalModeMsg(const std_msgs::msg::String::SharedPtr msg);
    void handleThrusterHealthMsg(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void handleEnvironmentalAssistanceMsg(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void handlePlanningStatusMsg(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void handleUSVStateMsg(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

private slots:
    void updateDashboard();
    void resetView();

private:
    // ROS2 node pointer
    std::shared_ptr<WAMVDashboardNode> node_ptr_;
    
    // Harbor zone static data
    std::vector<std::vector<QPointF>> harbor_zones_;
    std::vector<std::vector<QPointF>> dock_areas_;
    std::vector<QPointF> upper_boundary_;
    std::vector<QPointF> lower_boundary_;
    
    // Current system state
    std::string current_fault_status_;
    std::string current_operational_mode_;
    double usv_x_, usv_y_, usv_heading_;
    double wx_, wy_, wpsi_;
    double left_thruster_health_, right_thruster_health_;
    double commanded_tp_, commanded_ts_;
    double surge_assist_factor_, sway_assist_factor_, yaw_assist_factor_;
    
    // Planning data
    int selected_harbor_zone_;
    double target_x_, target_y_;
    double path_distance_;
    bool obstacle_free_;
    bool planning_active_;
    
    // Qt UI components - Harbor Zone Map
    QGraphicsView* harbor_map_view_;
    QGraphicsScene* harbor_map_scene_;
    std::vector<QGraphicsPolygonItem*> harbor_zone_items_;
    std::vector<QGraphicsPolygonItem*> dock_area_items_;
    QGraphicsLineItem* upper_boundary_item_;
    QGraphicsLineItem* lower_boundary_item_;
    USVGraphicsItem* usv_item_;
    QGraphicsLineItem* planned_path_item_;
    QGraphicsEllipseItem* target_zone_highlight_;
    
    // Environmental force arrows
    ForceArrowItem* wx_arrow_;
    ForceArrowItem* wy_arrow_;
    ForceArrowItem* wpsi_arrow_;
    
    // Qt UI components - Control Authority Panel
    QFrame* control_panel_;
    QLabel* operational_mode_label_;
    QLabel* fault_status_label_;
    QFrame* status_indicator_;
    
    // Thruster status
    QProgressBar* left_thruster_bar_;
    QProgressBar* right_thruster_bar_;
    QLabel* left_thrust_label_;
    QLabel* right_thrust_label_;
    
    // Environmental assistance status
    QLabel* surge_assist_label_;
    QLabel* sway_assist_label_;
    QLabel* yaw_assist_label_;
    QFrame* surge_assist_indicator_;
    QFrame* sway_assist_indicator_;
    QFrame* yaw_assist_indicator_;
    
    // Planning status
    QLabel* planning_status_label_;
    QLabel* target_zone_label_;
    QLabel* distance_label_;
    
    QPushButton* reset_button_;
    QTimer* update_timer_;
    
    // Configuration
    const int UPDATE_INTERVAL_MS = 100;  // Update interval in milliseconds
    
    // Helper functions
    void setupUI();
    void setupHarborMap();
    void setupControlPanel();
    void initializeHarborZones();
    void updateHarborMapDisplay();
    void updateControlPanelDisplay();
    QColor getFaultStatusColor();
    QColor getAssistanceColor(double factor);
    void updateEnvironmentalForces();
    void updatePlanningDisplay();
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
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr operational_mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr thruster_health_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr environmental_assistance_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr planning_status_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr usv_state_sub_;

    // Pointer to the dashboard UI
    WAMVDashboard* dashboard_;
};

#endif // WAMV_DASHBOARD_H