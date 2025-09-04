#include "vrx_control/wamv_dashboard.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QMessageBox>
#include <QPainter>
#include <QBrush>
#include <QPen>
#include <cmath>

// USVGraphicsItem implementation
USVGraphicsItem::USVGraphicsItem(double x, double y, double heading, QGraphicsItem* parent)
    : QGraphicsEllipseItem(-5, -5, 10, 10, parent), heading_(heading)
{
    setPos(x, y);
    setBrush(QBrush(QColor(0, 150, 0))); // Green USV
    setPen(QPen(QColor(0, 100, 0), 2));
    
    // Add heading line
    heading_line_ = new QGraphicsLineItem(0, 0, 15 * cos(heading), 15 * sin(heading), this);
    heading_line_->setPen(QPen(QColor(0, 100, 0), 3));
}

void USVGraphicsItem::updatePosition(double x, double y, double heading)
{
    setPos(x, y);
    heading_ = heading;
    
    // Update heading line
    heading_line_->setLine(0, 0, 15 * cos(heading), 15 * sin(heading));
}

void USVGraphicsItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
    Q_UNUSED(option)
    Q_UNUSED(widget)
    
    painter->setBrush(QBrush(QColor(0, 150, 0)));
    painter->setPen(QPen(QColor(0, 100, 0), 2));
    painter->drawEllipse(boundingRect());
}

// ForceArrowItem implementation
ForceArrowItem::ForceArrowItem(double start_x, double start_y, double force_x, double force_y, 
                              const QString& label, QGraphicsItem* parent)
    : QGraphicsLineItem(parent), label_(label), is_active_(false), arrow_scale_(0.5)
{
    updateForce(start_x, start_y, force_x, force_y);
    
    // Add label
    label_item_ = new QGraphicsTextItem(label, this);
    label_item_->setDefaultTextColor(QColor(50, 50, 50));
    QFont font = label_item_->font();
    font.setPointSize(8);
    label_item_->setFont(font);
}

void ForceArrowItem::updateForce(double start_x, double start_y, double force_x, double force_y)
{
    // Clear the previous drawing area before updating
    if (scene()) {
        scene()->update(boundingRect().translated(pos()));
    }
    
    setPos(start_x, start_y);
    
    // Scale force for visualization
    double scaled_x = force_x * arrow_scale_;
    double scaled_y = force_y * arrow_scale_;
    double magnitude = sqrt(scaled_x * scaled_x + scaled_y * scaled_y);
    
    if (magnitude > 0.5) {
        setLine(0, 0, scaled_x, scaled_y);
        setVisible(true);
        
        // Position label at end of arrow
        if (label_item_) {
            label_item_->setPos(scaled_x + 5, scaled_y - 10);
        }
    } else {
        setVisible(false);
    }
    
    // Update the new drawing area
    if (scene()) {
        scene()->update(boundingRect().translated(pos()));
    }
}

void ForceArrowItem::setActive(bool active)
{
    is_active_ = active;
    update();
}

void ForceArrowItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
    Q_UNUSED(option)
    Q_UNUSED(widget)
    
    QPen pen;
    if (is_active_) {
        pen = QPen(QColor(255, 100, 0), 3); // Orange for active
    } else {
        pen = QPen(QColor(100, 100, 100), 2); // Gray for inactive
    }
    
    painter->setPen(pen);
    painter->drawLine(line());
    
    // Draw arrowhead
    if (line().length() > 5) {
        QPointF end = line().p2();
        QPointF start = line().p1();
        double angle = atan2((end.y() - start.y()), (end.x() - start.x()));
        
        QPointF arrowP1 = end + QPointF(sin(angle + M_PI / 3) * 8, cos(angle + M_PI / 3) * 8);
        QPointF arrowP2 = end + QPointF(sin(angle + M_PI - M_PI / 3) * 8, cos(angle + M_PI - M_PI / 3) * 8);
        
        painter->drawLine(end, arrowP1);
        painter->drawLine(end, arrowP2);
    }
}

// WAMVDashboardNode implementation
WAMVDashboardNode::WAMVDashboardNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("wamv_dashboard_node", options), dashboard_(nullptr)
{
    // Initialize ROS2 subscribers
    disturbance_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/wamv/disturbance_world", 10,  // Changed from /wamv/disturbance
        [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
            if (dashboard_) dashboard_->handleDisturbanceMsg(msg);
        });
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/wamv/sensors/position/ground_truth_odometry", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            if (dashboard_) dashboard_->handleOdomMsg(msg);
        });
    
    fault_diagnosis_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/wamv/fault_diagnosis", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            if (dashboard_) dashboard_->handleFaultDiagnosisMsg(msg);
        });
    
    operational_mode_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/wamv/operational_mode", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            if (dashboard_) dashboard_->handleOperationalModeMsg(msg);
        });
    
    thruster_health_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/wamv/thruster_health", 10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            if (dashboard_) dashboard_->handleThrusterHealthMsg(msg);
        });
    
    environmental_assistance_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/wamv/environmental_assistance", 10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            if (dashboard_) dashboard_->handleEnvironmentalAssistanceMsg(msg);
        });
    
    planning_status_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/wamv/planning_status", 10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            if (dashboard_) dashboard_->handlePlanningStatusMsg(msg);
        });
    
    usv_state_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/wamv/usv_state", 20,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            if (dashboard_) dashboard_->handleUSVStateMsg(msg);
        });
    mission_metrics_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/wamv/mission_metrics", 10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            if (dashboard_) dashboard_->handleMissionMetricsMsg(msg);
        });
}

void WAMVDashboardNode::setDashboard(WAMVDashboard* dashboard) {
    dashboard_ = dashboard;
}

// WAMVDashboard implementation
WAMVDashboard::WAMVDashboard(std::shared_ptr<WAMVDashboardNode> node_ptr)
: QMainWindow(), node_ptr_(node_ptr)
{
    try {
        RCLCPP_INFO(node_ptr_->get_logger(), "WAMVDashboard constructor start");
        
        // Initialize data structures
        current_fault_status_ = "NO_FAULT";
        current_operational_mode_ = "FOLLOW_PRESET_TRAJECTORY";
        usv_x_ = 0.0; usv_y_ = 0.0; usv_heading_ = 0.0;
        wx_ = 0.0; wy_ = 0.0; wpsi_ = 0.0;
        left_thruster_health_ = 100.0; right_thruster_health_ = 100.0;
        commanded_tp_ = 0.0; commanded_ts_ = 0.0;
        surge_assist_factor_ = 0.0; sway_assist_factor_ = 0.0; yaw_assist_factor_ = 0.0;
        selected_harbor_zone_ = -1;
        target_x_ = 0.0; target_y_ = 0.0;
        path_distance_ = 0.0;
        obstacle_free_ = true;
        planning_active_ = false;
        
        // Initialize harbor zones
        initializeHarborZones();
        
        RCLCPP_INFO(node_ptr_->get_logger(), "Setting up UI");
        // Set up the UI
        setupUI();
        
        // Create update timer
        RCLCPP_INFO(node_ptr_->get_logger(), "Creating update timer");
        update_timer_ = new QTimer(this);
        connect(update_timer_, &QTimer::timeout, this, &WAMVDashboard::updateDashboard);
        update_timer_->start(UPDATE_INTERVAL_MS);
        
        // Set window properties
        setWindowTitle("WAMV Environmental Assistance Dashboard");
        resize(1800, 1000);
        
        RCLCPP_INFO(node_ptr_->get_logger(), "WAMVDashboard constructor completed");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_ptr_->get_logger(), "Exception in WAMVDashboard constructor: %s", e.what());
    } catch (...) {
        RCLCPP_ERROR(node_ptr_->get_logger(), "Unknown exception in WAMVDashboard constructor");
    }
}

WAMVDashboard::~WAMVDashboard()
{
    if (update_timer_) {
        update_timer_->stop();
        delete update_timer_;
    }
}

void WAMVDashboard::initializeHarborZones()
{
    // Harbor zones (safe approach areas)
    harbor_zones_.resize(3);
    
    // Zone 1: [-580, 258], [-572, 241], [-600, 236], [-600, 248]
    harbor_zones_[0] = {
        QPointF(-580, 258), QPointF(-572, 241), 
        QPointF(-600, 236), QPointF(-600, 248)
    };
    
    // Zone 2: [-570, 223], [-568, 209], [-595, 208], [-595, 220]
    harbor_zones_[1] = {
        QPointF(-570, 223), QPointF(-568, 209),
        QPointF(-595, 208), QPointF(-595, 220)
    };
    
    // Zone 3: [-568, 192], [-593, 191], [-593, 183], [-579, 184]
    harbor_zones_[2] = {
        QPointF(-568, 192), QPointF(-593, 191),
        QPointF(-593, 183), QPointF(-579, 184)
    };
    
    // Dock areas (obstacles to avoid)
    dock_areas_.resize(2);
    
    // Dock 1: [-572, 241], [-570, 223], [-595, 220], [-600, 236]
    dock_areas_[0] = {
        QPointF(-572, 241), QPointF(-570, 223),
        QPointF(-595, 220), QPointF(-600, 236)
    };
    
    // Dock 2: [-568, 209], [-568, 192], [-593, 191], [-595, 208]
    dock_areas_[1] = {
        QPointF(-568, 209), QPointF(-568, 192),
        QPointF(-593, 191), QPointF(-595, 208)
    };
    
    // Boundary lines
    upper_boundary_ = {QPointF(-580, 258), QPointF(-600, 248)};
    lower_boundary_ = {QPointF(-579, 184), QPointF(-593, 183)};
}

void WAMVDashboard::setupUI()
{
    // Create central widget and main layout
    QWidget *central_widget = new QWidget(this);
    QHBoxLayout *main_layout = new QHBoxLayout(central_widget);
    
    // Setup harbor map (left panel)
    setupHarborMap();
    
    // Setup control panel (right panel)
    setupControlPanel();
    
    // Add both panels to main layout
    main_layout->addWidget(harbor_map_view_, 2); // 2/3 of width
    main_layout->addWidget(control_panel_, 1);   // 1/3 of width
    
    // Set central widget
    setCentralWidget(central_widget);
}

void WAMVDashboard::setupHarborMap()
{
    // Create graphics view and scene
    harbor_map_view_ = new QGraphicsView(this);
    harbor_map_scene_ = new QGraphicsScene(this);
    harbor_map_view_->setScene(harbor_map_scene_);
    
    // Set scene rectangle to encompass harbor zones with proper scaling
    harbor_map_scene_->setSceneRect(-650, 150, 100, 150);
    
    // Add harbor zones with scaling factor for better visibility
    double scale_factor = 10.0; // Doubled from 5.0 to make harbor zones twice as big
    harbor_zone_items_.resize(harbor_zones_.size());
    for (size_t i = 0; i < harbor_zones_.size(); ++i) {
        QPolygonF polygon;
        for (const auto& point : harbor_zones_[i]) {
            // Apply 90-degree counter-clockwise rotation AND horizontal flip
            // Rotation: (x,y) -> (-y,x), then flip: (-y,x) -> (-(-y),x) = (y,x)
            // Combined: (x,y) -> (y,-x) for the flip you want
            double rotated_x = -point.y() * scale_factor;  // Negative for horizontal flip
            double rotated_y = -point.x() * scale_factor;
            polygon << QPointF(rotated_x, rotated_y);
        }
        harbor_zone_items_[i] = harbor_map_scene_->addPolygon(
            polygon, QPen(QColor(0, 150, 0), 2), QBrush(QColor(0, 255, 0, 80)));
        
        // Add zone label
        QPointF center = polygon.boundingRect().center();
        QGraphicsTextItem* label = harbor_map_scene_->addText(
            QString("Zone %1").arg(i + 1), QFont("Arial", 10, QFont::Bold));
        label->setPos(center - QPointF(20, 5));
        label->setDefaultTextColor(QColor(0, 100, 0));
    }
    
    // Add dock areas with same transformation
    dock_area_items_.resize(dock_areas_.size());
    for (size_t i = 0; i < dock_areas_.size(); ++i) {
        QPolygonF polygon;
        for (const auto& point : dock_areas_[i]) {
            // Apply same transformation as harbor zones
            double rotated_x = -point.y() * scale_factor;
            double rotated_y = -point.x() * scale_factor;
            polygon << QPointF(rotated_x, rotated_y);
        }
        dock_area_items_[i] = harbor_map_scene_->addPolygon(
            polygon, QPen(QColor(150, 0, 0), 2), QBrush(QColor(255, 0, 0, 80)));
        
        // Add dock label
        QPointF center = polygon.boundingRect().center();
        QGraphicsTextItem* label = harbor_map_scene_->addText(
            QString("Dock %1").arg(i + 1), QFont("Arial", 10, QFont::Bold));
        label->setPos(center - QPointF(20, 5));
        label->setDefaultTextColor(QColor(150, 0, 0));
    }
    
    // Add boundary lines with transformation
    QPointF upper_start(-upper_boundary_[0].y() * scale_factor, -upper_boundary_[0].x() * scale_factor);
    QPointF upper_end(-upper_boundary_[1].y() * scale_factor, -upper_boundary_[1].x() * scale_factor);
    upper_boundary_item_ = harbor_map_scene_->addLine(
        upper_start.x(), upper_start.y(), upper_end.x(), upper_end.y(),
        QPen(QColor(255, 0, 0), 2, Qt::DashLine));
    
    QPointF lower_start(-lower_boundary_[0].y() * scale_factor, -lower_boundary_[0].x() * scale_factor);
    QPointF lower_end(-lower_boundary_[1].y() * scale_factor, -lower_boundary_[1].x() * scale_factor);
    lower_boundary_item_ = harbor_map_scene_->addLine(
        lower_start.x(), lower_start.y(), lower_end.x(), lower_end.y(),
        QPen(QColor(255, 0, 0), 2, Qt::DashLine));
    
    // Update scene rectangle to fit transformed coordinates (doubled size)
    harbor_map_scene_->setSceneRect(-2600, 5000, 1000, 1500);
    
    // Initialize USV item (will be positioned in update function)
    usv_item_ = new USVGraphicsItem(0, 0, 0);
    harbor_map_scene_->addItem(usv_item_);
    
    // Initialize environmental force arrows
    wx_arrow_ = new ForceArrowItem(0, 0, 0, 0, "Fx");
    wy_arrow_ = new ForceArrowItem(0, 0, 0, 0, "Fy");
    wpsi_arrow_ = new ForceArrowItem(0, 0, 0, 0, "Mz");
    
    harbor_map_scene_->addItem(wx_arrow_);
    harbor_map_scene_->addItem(wy_arrow_);
    harbor_map_scene_->addItem(wpsi_arrow_);
    
    // Initialize planned path and target highlighting (initially hidden)
    planned_path_item_ = harbor_map_scene_->addLine(0, 0, 0, 0, QPen(QColor(0, 0, 255), 3, Qt::DashLine));
    planned_path_item_->setVisible(false);
    
    target_zone_highlight_ = harbor_map_scene_->addEllipse(0, 0, 30, 30, 
        QPen(QColor(255, 200, 0), 4), QBrush(QColor(255, 200, 0, 50)));
    target_zone_highlight_->setVisible(false);
    
    // Set view properties
    harbor_map_view_->setDragMode(QGraphicsView::ScrollHandDrag);
    harbor_map_view_->setRenderHint(QPainter::Antialiasing);
}

void WAMVDashboard::setupControlPanel()
{
    control_panel_ = new QFrame(this);
    control_panel_->setFrameShape(QFrame::Box);
    control_panel_->setMaximumWidth(400);
    
    QVBoxLayout *panel_layout = new QVBoxLayout(control_panel_);
    
    // Operational Mode Section
    QGroupBox *mode_group = new QGroupBox("Operational Mode");
    QVBoxLayout *mode_layout = new QVBoxLayout(mode_group);
    
    operational_mode_label_ = new QLabel("FOLLOW_PRESET_TRAJECTORY");
    operational_mode_label_->setFont(QFont("Arial", 12, QFont::Bold));
    mode_layout->addWidget(operational_mode_label_);
    
    // Fault Status Section
    QGroupBox *fault_group = new QGroupBox("System Status");
    QHBoxLayout *fault_layout = new QHBoxLayout(fault_group);
    
    status_indicator_ = new QFrame();
    status_indicator_->setFrameShape(QFrame::Box);
    status_indicator_->setFixedSize(50, 50);
    status_indicator_->setStyleSheet("background-color: green;");
    
    fault_status_label_ = new QLabel("Status: NO_FAULT");
    fault_status_label_->setFont(QFont("Arial", 12, QFont::Bold));
    
    fault_layout->addWidget(status_indicator_);
    fault_layout->addWidget(fault_status_label_);
    fault_layout->addStretch();
    
    // Thruster Health Section
    QGroupBox *thruster_group = new QGroupBox("Thruster Health");
    QVBoxLayout *thruster_layout = new QVBoxLayout(thruster_group);
    
    QHBoxLayout *left_layout = new QHBoxLayout();
    left_layout->addWidget(new QLabel("Left:"));
    left_thruster_bar_ = new QProgressBar();
    left_thruster_bar_->setRange(0, 100);
    left_thruster_bar_->setValue(100);
    left_thrust_label_ = new QLabel("0 N");
    left_layout->addWidget(left_thruster_bar_);
    left_layout->addWidget(left_thrust_label_);
    
    QHBoxLayout *right_layout = new QHBoxLayout();
    right_layout->addWidget(new QLabel("Right:"));
    right_thruster_bar_ = new QProgressBar();
    right_thruster_bar_->setRange(0, 100);
    right_thruster_bar_->setValue(100);
    right_thrust_label_ = new QLabel("0 N");
    right_layout->addWidget(right_thruster_bar_);
    right_layout->addWidget(right_thrust_label_);
    
    thruster_layout->addLayout(left_layout);
    thruster_layout->addLayout(right_layout);
    
    // Environmental Assistance Section
    QGroupBox *assist_group = new QGroupBox("Environmental Assistance");
    QVBoxLayout *assist_layout = new QVBoxLayout(assist_group);
    
    QHBoxLayout *surge_layout = new QHBoxLayout();
    surge_layout->addWidget(new QLabel("Surge (X):"));
    surge_assist_indicator_ = new QFrame();
    surge_assist_indicator_->setFrameShape(QFrame::Box);
    surge_assist_indicator_->setFixedSize(20, 20);
    surge_assist_indicator_->setStyleSheet("background-color: gray;");
    surge_assist_label_ = new QLabel("Inactive");
    surge_layout->addWidget(surge_assist_indicator_);
    surge_layout->addWidget(surge_assist_label_);
    surge_layout->addStretch();
    
    QHBoxLayout *sway_layout = new QHBoxLayout();
    sway_layout->addWidget(new QLabel("Sway (Y):"));
    sway_assist_indicator_ = new QFrame();
    sway_assist_indicator_->setFrameShape(QFrame::Box);
    sway_assist_indicator_->setFixedSize(20, 20);
    sway_assist_indicator_->setStyleSheet("background-color: gray;");
    sway_assist_label_ = new QLabel("Inactive");
    sway_layout->addWidget(sway_assist_indicator_);
    sway_layout->addWidget(sway_assist_label_);
    sway_layout->addStretch();
    
    QHBoxLayout *yaw_layout = new QHBoxLayout();
    yaw_layout->addWidget(new QLabel("Yaw (Z):"));
    yaw_assist_indicator_ = new QFrame();
    yaw_assist_indicator_->setFrameShape(QFrame::Box);
    yaw_assist_indicator_->setFixedSize(20, 20);
    yaw_assist_indicator_->setStyleSheet("background-color: gray;");
    yaw_assist_label_ = new QLabel("Inactive");
    yaw_layout->addWidget(yaw_assist_indicator_);
    yaw_layout->addWidget(yaw_assist_label_);
    yaw_layout->addStretch();
    
    assist_layout->addLayout(surge_layout);
    assist_layout->addLayout(sway_layout);
    assist_layout->addLayout(yaw_layout);

    // NEW: Environmental Forces Visualization Section
    QGroupBox *env_forces_group = new QGroupBox("Environmental Forces Utilized");
    QVBoxLayout *env_forces_layout = new QVBoxLayout(env_forces_group);
    
    // Create custom force bars
    env_force_x_bar_ = new EnvironmentalForceBar("Fx", "N", this);
    env_force_y_bar_ = new EnvironmentalForceBar("Fy", "N", this);
    env_force_psi_bar_ = new EnvironmentalForceBar("Mz", "Nm", this);
    
    // Set appropriate ranges
    env_force_x_bar_->setMaxRange(50.0);    // ±50N for forces
    env_force_y_bar_->setMaxRange(50.0);    // ±50N for forces
    env_force_psi_bar_->setMaxRange(20.0);  // ±20Nm for moment
    
    env_forces_layout->addWidget(env_force_x_bar_);
    env_forces_layout->addWidget(env_force_y_bar_);
    env_forces_layout->addWidget(env_force_psi_bar_);
    
    // Add a summary label
    QLabel* env_summary = new QLabel("Gray: Available | Orange: Utilized");
    env_summary->setAlignment(Qt::AlignCenter);
    QFont summary_font = env_summary->font();
    summary_font.setPointSize(9);
    summary_font.setItalic(true);
    env_summary->setFont(summary_font);
    env_forces_layout->addWidget(env_summary);
    
    // Planning Status Section
    QGroupBox *planning_group = new QGroupBox("Mission Planning");
    QVBoxLayout *planning_layout = new QVBoxLayout(planning_group);
    
    planning_status_label_ = new QLabel("Following preset trajectory");
    target_zone_label_ = new QLabel("Target Zone: None");
    distance_label_ = new QLabel("Distance: N/A");
    
    planning_layout->addWidget(planning_status_label_);
    planning_layout->addWidget(target_zone_label_);
    planning_layout->addWidget(distance_label_);
    
    // Reset Button
    reset_button_ = new QPushButton("Reset View");
    connect(reset_button_, &QPushButton::clicked, this, &WAMVDashboard::resetView);
    
    // Add all groups to panel
    panel_layout->addWidget(mode_group);
    panel_layout->addWidget(fault_group);
    panel_layout->addWidget(thruster_group);
    panel_layout->addWidget(assist_group);
    panel_layout->addWidget(env_forces_group);
    panel_layout->addWidget(planning_group);
    panel_layout->addWidget(reset_button_);
    panel_layout->addStretch();
}

// Message handlers
void WAMVDashboard::handleDisturbanceMsg(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    wx_ = msg->twist.linear.x;
    wy_ = msg->twist.linear.y;
    wpsi_ = msg->twist.angular.z;
}

void WAMVDashboard::handleOdomMsg(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Extract position and orientation from odometry
    usv_x_ = msg->pose.pose.position.x;
    usv_y_ = msg->pose.pose.position.y;
    
    // Convert quaternion to yaw
    tf2::Quaternion quat(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    usv_heading_ = yaw;
}

void WAMVDashboard::handleFaultDiagnosisMsg(const std_msgs::msg::String::SharedPtr msg)
{
    // Log every message received for debugging
    // RCLCPP_INFO(node_ptr_->get_logger(), "=== FAULT MESSAGE RECEIVED ===");
    // RCLCPP_INFO(node_ptr_->get_logger(), "Raw message: '%s'", msg->data.c_str());
    
    // Parse fault diagnosis message - handle different message formats
    std::string message = msg->data;
    std::string old_status = current_fault_status_;
    
    // Try multiple parsing approaches
    size_t fault_pos = message.find("Fault: ");
    if (fault_pos != std::string::npos) {
        // Format: "Fault: FAULT_TYPE (Confidence: XX.X%)"
        size_t fault_start = fault_pos + 7;
        size_t fault_end = message.find(" ", fault_start);
        if (fault_end == std::string::npos) {
            fault_end = message.length();
        }
        current_fault_status_ = message.substr(fault_start, fault_end - fault_start);
    } else {
        // Try simpler format - just the fault type
        if (message.find("LEFT_THRUST_FAILURE") != std::string::npos) {
            current_fault_status_ = "LEFT_THRUST_FAILURE";
        } else if (message.find("RIGHT_THRUST_FAILURE") != std::string::npos) {
            current_fault_status_ = "RIGHT_THRUST_FAILURE";
        } else if (message.find("NO_FAULT") != std::string::npos) {
            current_fault_status_ = "NO_FAULT";
        } else {
            // Use the entire message as fault status
            current_fault_status_ = message;
        }
    }
    
    // RCLCPP_INFO(node_ptr_->get_logger(), "Status changed: '%s' -> '%s'", 
    //             old_status.c_str(), current_fault_status_.c_str());
    // RCLCPP_INFO(node_ptr_->get_logger(), "=== END FAULT MESSAGE ===");
}

void WAMVDashboard::handleOperationalModeMsg(const std_msgs::msg::String::SharedPtr msg)
{
    current_operational_mode_ = msg->data;
    
    // Force update of planning status when mode changes to STATION_KEEPING
    if (current_operational_mode_ == "STATION_KEEPING") {
        planning_active_ = false;  // Disable planning display
        
        // Update the planning status label directly
        if (planning_status_label_) {
            planning_status_label_->setText("🎉 Mission Complete - Station Keeping");
        }
        if (target_zone_label_) {
            target_zone_label_->setText("📍 Target Zone: Arrived");
        }
        if (distance_label_) {
            distance_label_->setText("Distance: 0.0 m");
        }
    }
}

void WAMVDashboard::handleThrusterHealthMsg(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() >= 4) {
        left_thruster_health_ = msg->data[0];
        right_thruster_health_ = msg->data[1];
        commanded_tp_ = msg->data[2];
        commanded_ts_ = msg->data[3];
    }
}

void WAMVDashboard::handleEnvironmentalAssistanceMsg(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() >= 6) {
        surge_assist_factor_ = msg->data[0];
        sway_assist_factor_ = msg->data[1];
        yaw_assist_factor_ = msg->data[2];
        
        // Available forces (from EKF)
        double available_wx = msg->data[3];
        double available_wy = msg->data[4];
        double available_wpsi = msg->data[5];
        
        // Calculate utilized forces
        double utilized_wx = available_wx * surge_assist_factor_;
        double utilized_wy = available_wy * sway_assist_factor_;
        double utilized_wpsi = available_wpsi * yaw_assist_factor_;
        
        // Update the force bars
        if (env_force_x_bar_) {
            dynamic_cast<EnvironmentalForceBar*>(env_force_x_bar_)->setForces(utilized_wx, available_wx);
        }
        if (env_force_y_bar_) {
            dynamic_cast<EnvironmentalForceBar*>(env_force_y_bar_)->setForces(utilized_wy, available_wy);
        }
        if (env_force_psi_bar_) {
            dynamic_cast<EnvironmentalForceBar*>(env_force_psi_bar_)->setForces(utilized_wpsi, available_wpsi);
        }
        
        // Auto-adjust ranges if forces exceed current range
        if (std::abs(available_wx) > 45.0) {
            dynamic_cast<EnvironmentalForceBar*>(env_force_x_bar_)->setMaxRange(100.0);
        }
        if (std::abs(available_wy) > 45.0) {
            dynamic_cast<EnvironmentalForceBar*>(env_force_y_bar_)->setMaxRange(100.0);
        }
        if (std::abs(available_wpsi) > 18.0) {
            dynamic_cast<EnvironmentalForceBar*>(env_force_psi_bar_)->setMaxRange(40.0);
        }
    }
}

void WAMVDashboard::handlePlanningStatusMsg(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() >= 6) {
        selected_harbor_zone_ = static_cast<int>(msg->data[0]);
        target_x_ = msg->data[1];
        target_y_ = msg->data[2];
        path_distance_ = msg->data[3];
        obstacle_free_ = (msg->data[5] > 0.5);
        planning_active_ = true;
    }
}

void WAMVDashboard::handleUSVStateMsg(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // This provides enhanced USV state data if needed
    // For now, we use the odometry message for position updates
    Q_UNUSED(msg);
}

void WAMVDashboard::updateDashboard()
{
    try {
        updateHarborMapDisplay();
        updateControlPanelDisplay();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_ptr_->get_logger(), "Error in updateDashboard: %s", e.what());
    }
}

void WAMVDashboard::handleMissionMetricsMsg(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() >= 5) {
        mission_duration_ = msg->data[0];
        mission_energy_ = msg->data[1];
        mission_completed_flag_ = (msg->data[4] > 0.5);
    }
}

void WAMVDashboard::updateHarborMapDisplay()
{
    // Transform USV position to match harbor zone coordinate system
    double scale_factor = 10.0; // Updated to match the doubled harbor zone scaling
    double transformed_x = -usv_y_ * scale_factor;  // Apply horizontal flip
    double transformed_y = -usv_x_ * scale_factor;
    double transformed_heading = 3.0 * M_PI / 2.0 - usv_heading_; // Adjust heading for rotation
    
    // Update USV position and heading
    if (usv_item_) {
        usv_item_->updatePosition(transformed_x, transformed_y, transformed_heading);
    }
    
    // Update environmental force arrows
    updateEnvironmentalForces();
    
    // Update planning display
    updatePlanningDisplay();
}



void WAMVDashboard::updateControlPanelDisplay()
{
    // Update operational mode
    operational_mode_label_->setText(QString::fromStdString(current_operational_mode_));
    
    // Update fault status with debugging
    QString fault_display = QString("Status: %1").arg(QString::fromStdString(current_fault_status_));
    fault_status_label_->setText(fault_display);
    
    // Log fault status periodically for debugging
    static int debug_counter = 0;
    debug_counter++;
    if (debug_counter % 50 == 0) { // Log every 5 seconds at 100ms update rate
        RCLCPP_INFO(node_ptr_->get_logger(), "Current fault status in GUI: %s", current_fault_status_.c_str());
    }
    
    QColor status_color = getFaultStatusColor();
    status_indicator_->setStyleSheet(QString("background-color: %1;").arg(status_color.name()));
    
    // Update thruster health
    left_thruster_bar_->setValue(static_cast<int>(left_thruster_health_));
    right_thruster_bar_->setValue(static_cast<int>(right_thruster_health_));
    left_thrust_label_->setText(QString("%1 N").arg(commanded_tp_, 0, 'f', 1));
    right_thrust_label_->setText(QString("%1 N").arg(commanded_ts_, 0, 'f', 1));
    
    // Update environmental assistance status
    surge_assist_indicator_->setStyleSheet(
        QString("background-color: %1;").arg(getAssistanceColor(surge_assist_factor_).name()));
    surge_assist_label_->setText(surge_assist_factor_ > 0.1 ? "Active" : "Inactive");
    
    sway_assist_indicator_->setStyleSheet(
        QString("background-color: %1;").arg(getAssistanceColor(sway_assist_factor_).name()));
    sway_assist_label_->setText(sway_assist_factor_ > 0.1 ? "Active" : "Inactive");
    
    yaw_assist_indicator_->setStyleSheet(
        QString("background-color: %1;").arg(getAssistanceColor(yaw_assist_factor_).name()));
    yaw_assist_label_->setText(yaw_assist_factor_ > 0.1 ? "Active" : "Inactive");
    
    // Update planning status
    static QString last_planning_status = "";
    static QString last_target_zone = "";
    static QString last_distance = "";

    QString new_planning_status, new_target_zone, new_distance;

    // Determine new values based on state
    if (current_operational_mode_ == "STATION_KEEPING" && mission_completed_flag_) {
        // Mission completed state with metrics
        new_planning_status = "🎉 Mission Complete - Station Keeping";
        new_target_zone = QString("📍 Final Zone: %1 | Duration: %2s")
            .arg(selected_harbor_zone_ + 1)
            .arg(mission_duration_, 0, 'f', 1);
        new_distance = QString("Energy Used: %1 kJ (%2 kWh)")
            .arg(mission_energy_ / 1000.0, 0, 'f', 2)
            .arg(mission_energy_ / 3600000.0, 0, 'f', 4);
    } else if (planning_active_ && selected_harbor_zone_ >= 0) {
        // Active planning state
        new_planning_status = "Adaptive return to harbor";
        new_target_zone = QString("Target Zone: %1").arg(selected_harbor_zone_ + 1);
        new_distance = QString("Distance: %1 m").arg(path_distance_, 0, 'f', 1);
    } else {
        // Default state
        new_planning_status = "Following preset trajectory";
        new_target_zone = "Target Zone: None";
        new_distance = "Distance: N/A";
    }

    // Only update if values changed
    if (new_planning_status != last_planning_status) {
        planning_status_label_->setText(new_planning_status);
        last_planning_status = new_planning_status;
    }
    if (new_target_zone != last_target_zone) {
        target_zone_label_->setText(new_target_zone);
        last_target_zone = new_target_zone;
    }
    if (new_distance != last_distance) {
        distance_label_->setText(new_distance);
        last_distance = new_distance;
    }
}

void WAMVDashboard::updateEnvironmentalForces()
{
    if (wx_arrow_ && wy_arrow_ && wpsi_arrow_) {
        double scale_factor = 10.0; // Updated to match the doubled harbor zone scaling
        double transformed_usv_x = -usv_y_ * scale_factor;  // Apply horizontal flip
        double transformed_usv_y = -usv_x_ * scale_factor;
        
        // Clear previous arrow positions by forcing scene update
        harbor_map_scene_->update();
        
        // Transform environmental forces to match coordinate system
        // Apply the same transformation to force directions
        // Keep arrow size the same (15x scaling) as requested
        double transformed_wx = -wy_ * 15; // Keep arrow scale unchanged
        double transformed_wy = -wx_ * 15;
        
        // Update force arrows with transformed coordinates and forces
        wx_arrow_->updateForce(transformed_usv_x, transformed_usv_y, transformed_wx, 0);
        wy_arrow_->updateForce(transformed_usv_x, transformed_usv_y, 0, transformed_wy);
        
        // For yaw moment, show as offset arrow
        double wpsi_offset_x = 20 * cos(usv_heading_);
        double wpsi_offset_y = 20 * sin(usv_heading_);
        wpsi_arrow_->updateForce(transformed_usv_x + wpsi_offset_x, transformed_usv_y + wpsi_offset_y, 
                                wpsi_ * cos(usv_heading_) * 8, wpsi_ * sin(usv_heading_) * 8);
        
        // Set active status based on assistance factors
        wx_arrow_->setActive(surge_assist_factor_ > 0.1);
        wy_arrow_->setActive(sway_assist_factor_ > 0.1);
        wpsi_arrow_->setActive(yaw_assist_factor_ > 0.1);
    }
}

void WAMVDashboard::updatePlanningDisplay()
{
    double scale_factor = 10.0; // Updated to match the doubled harbor zone scaling
    double transformed_usv_x = -usv_y_ * scale_factor;  // Apply horizontal flip
    double transformed_usv_y = -usv_x_ * scale_factor;
    
    if (planning_active_ && selected_harbor_zone_ >= 0 && selected_harbor_zone_ < 3) {
        // Transform target coordinates with horizontal flip
        double transformed_target_x = -target_y_ * scale_factor;
        double transformed_target_y = -target_x_ * scale_factor;
        
        // Show planned path
        if (planned_path_item_) {
            planned_path_item_->setLine(transformed_usv_x, transformed_usv_y, 
                                       transformed_target_x, transformed_target_y);
            planned_path_item_->setVisible(true);
        }
        
        // Highlight target zone
        if (target_zone_highlight_) {
            target_zone_highlight_->setPos(transformed_target_x - 15, transformed_target_y - 15);
            target_zone_highlight_->setVisible(true);
        }
        
        // Highlight selected harbor zone
        if (selected_harbor_zone_ < static_cast<int>(harbor_zone_items_.size())) {
            for (size_t i = 0; i < harbor_zone_items_.size(); ++i) {
                if (i == static_cast<size_t>(selected_harbor_zone_)) {
                    harbor_zone_items_[i]->setPen(QPen(QColor(255, 200, 0), 4)); // Gold highlight
                } else {
                    harbor_zone_items_[i]->setPen(QPen(QColor(0, 150, 0), 2)); // Normal green
                }
            }
        }
    } else {
        // Hide planning elements
        if (planned_path_item_) planned_path_item_->setVisible(false);
        if (target_zone_highlight_) target_zone_highlight_->setVisible(false);
        
        // Reset zone highlighting
        for (auto* zone_item : harbor_zone_items_) {
            zone_item->setPen(QPen(QColor(0, 150, 0), 2));
        }
    }
}



void WAMVDashboard::resetView()
{
    // Reset harbor map view to show all elements
    harbor_map_view_->fitInView(harbor_map_scene_->itemsBoundingRect(), Qt::KeepAspectRatio);
    
    // Center on USV if it exists
    if (usv_item_) {
        harbor_map_view_->centerOn(usv_item_);
    }
}

QColor WAMVDashboard::getFaultStatusColor()
{
    if (current_fault_status_ == "NO_FAULT") {
        return QColor(0, 200, 0);  // Green
    } else {
        return QColor(200, 0, 0);  // Red
    }
}

QColor WAMVDashboard::getAssistanceColor(double factor)
{
    if (factor > 0.1) {
        return QColor(255, 150, 0);  // Orange for active
    } else {
        return QColor(128, 128, 128);  // Gray for inactive
    }
}