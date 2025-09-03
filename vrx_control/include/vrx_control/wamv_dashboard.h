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


class EnvironmentalForceBar : public QWidget {
    public:
        EnvironmentalForceBar(const QString& label, const QString& units, QWidget* parent = nullptr) 
            : QWidget(parent), label_(label), units_(units), 
              utilized_force_(0), available_force_(0), max_range_(50.0) {
            setFixedHeight(60);
            setMinimumWidth(300);
        }
        
        void setForces(double utilized, double available) {
            utilized_force_ = utilized;
            available_force_ = available;
            update();
        }
        
        void setMaxRange(double range) {
            max_range_ = range;
            update();
        }
    
    protected:
        void paintEvent(QPaintEvent* event) override {
            Q_UNUSED(event)
            
            QPainter painter(this);
            painter.setRenderHint(QPainter::Antialiasing);
            
            // Calculate dimensions
            int bar_height = 30;
            int bar_width = width() - 100;
            int bar_x = 50;
            int bar_y = 25;
            int center_x = bar_x + bar_width / 2;
            
            // Draw background
            painter.fillRect(bar_x, bar_y, bar_width, bar_height, QColor(240, 240, 240));
            
            // Draw scale markings
            painter.setPen(QPen(Qt::gray, 1));
            for (int i = -2; i <= 2; i++) {
                int mark_x = center_x + (i * bar_width / 4);
                painter.drawLine(mark_x, bar_y - 3, mark_x, bar_y + bar_height + 3);
                
                // Draw scale labels
                QFont small_font = painter.font();
                small_font.setPointSize(8);
                painter.setFont(small_font);
                
                double value = (i * max_range_) / 2;
                QString text = QString::number(value, 'f', 0);
                painter.drawText(mark_x - 15, bar_y + bar_height + 15, 30, 10, 
                               Qt::AlignCenter, text);
            }
            
            // Draw center line
            painter.setPen(QPen(Qt::black, 2));
            painter.drawLine(center_x, bar_y - 5, center_x, bar_y + bar_height + 5);
            
            // Calculate bar positions
            double scale = (bar_width / 2.0) / max_range_;
            
            // Draw available force (gray)
            if (std::abs(available_force_) > 0.1) {
                int available_width = std::abs(available_force_) * scale;
                available_width = std::max(2, available_width);  // Minimum visibility
                
                QRect available_rect;
                if (available_force_ > 0) {
                    available_rect = QRect(center_x, bar_y, available_width, bar_height);
                } else {
                    available_rect = QRect(center_x - available_width, bar_y, available_width, bar_height);
                }
                painter.fillRect(available_rect, QColor(180, 180, 180, 150));
            }
            
            // Draw utilized force (orange)
            if (std::abs(utilized_force_) > 0.1) {
                int utilized_width = std::abs(utilized_force_) * scale;
                utilized_width = std::max(2, utilized_width);  // Minimum visibility
                
                QRect utilized_rect;
                if (utilized_force_ > 0) {
                    utilized_rect = QRect(center_x, bar_y, utilized_width, bar_height);
                } else {
                    utilized_rect = QRect(center_x - utilized_width, bar_y, utilized_width, bar_height);
                }
                painter.fillRect(utilized_rect, QColor(255, 140, 0));
            }
            
            // Draw border
            painter.setPen(QPen(Qt::black, 1));
            painter.drawRect(bar_x, bar_y, bar_width, bar_height);
            
            // Draw label
            QFont label_font = painter.font();
            label_font.setPointSize(10);
            label_font.setBold(true);
            painter.setFont(label_font);
            painter.setPen(Qt::black);
            painter.drawText(5, bar_y + bar_height/2 + 5, label_);
            
            // Draw value text
            double utilization_percent = 0.0;
            if (std::abs(available_force_) > 0.1) {
                utilization_percent = (std::abs(utilized_force_) / std::abs(available_force_)) * 100.0;
            }
            
            QString value_text = QString("%1: %2%3 (%4%)")
                .arg(label_)
                .arg(utilized_force_ >= 0 ? "+" : "")
                .arg(utilized_force_, 0, 'f', 1)
                .arg(units_)
                .arg(utilization_percent, 0, 'f', 0);
            
            painter.drawText(bar_x, 5, bar_width, 20, Qt::AlignCenter, value_text);
        }
    
    private:
        QString label_;
        QString units_;
        double utilized_force_;
        double available_force_;
        double max_range_;
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
    void handleMissionMetricsMsg(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

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

    // Environmental force visualization components
    EnvironmentalForceBar* env_force_x_bar_;
    EnvironmentalForceBar* env_force_y_bar_;
    EnvironmentalForceBar* env_force_psi_bar_;
    QLabel* env_force_x_label_;
    QLabel* env_force_y_label_;
    QLabel* env_force_psi_label_;
    QLabel* env_force_x_value_;
    QLabel* env_force_y_value_;
    QLabel* env_force_psi_value_;
    
    // Configuration
    const int UPDATE_INTERVAL_MS = 100;  // Update interval in milliseconds
    double mission_duration_ = 0.0;
    double mission_energy_ = 0.0;
    bool mission_completed_flag_ = false;
    
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
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr mission_metrics_sub_;

    // Pointer to the dashboard UI
    WAMVDashboard* dashboard_;
};

#endif // WAMV_DASHBOARD_H