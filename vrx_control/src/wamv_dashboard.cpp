#include "vrx_control/wamv_dashboard.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QMessageBox>
#include <cmath>

// WAMVDashboardNode implementation
WAMVDashboardNode::WAMVDashboardNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("wamv_dashboard_node", options), dashboard_(nullptr)
{
    // Initialize ROS2 subscribers
    disturbance_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/wamv/disturbance", 10,
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
    
    fault_features_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/wamv/fault_features", 10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            if (dashboard_) dashboard_->handleFaultFeaturesMsg(msg);
        });
    
    wpsi_coefficient_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/wamv/wpsi_coefficient", 10,
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
            if (dashboard_) dashboard_->handleWpsiCoefficientMsg(msg);
        });
    fault_confidence_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/wamv/fault_confidences", 10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            if (dashboard_) dashboard_->handleFaultConfidenceMsg(msg);
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
        fault_confidence_ = 0.0;
        
        RCLCPP_INFO(node_ptr_->get_logger(), "Setting up UI");
        // Set up the UI
        setupUI();

        // Delay the first update to ensure UI is fully initialized
        QTimer::singleShot(1000, this, [this]() {
            RCLCPP_INFO(node_ptr_->get_logger(), "Starting update timer");
            update_timer_ = new QTimer(this);
            connect(update_timer_, &QTimer::timeout, this, &WAMVDashboard::updatePlots);
            update_timer_->start(UPDATE_INTERVAL_MS);
        });
        
        RCLCPP_INFO(node_ptr_->get_logger(), "Creating update timer");
        // Create update timer
        update_timer_ = new QTimer(this);
        connect(update_timer_, &QTimer::timeout, this, &WAMVDashboard::updatePlots);
        
        RCLCPP_INFO(node_ptr_->get_logger(), "Starting timer");
        update_timer_->start(UPDATE_INTERVAL_MS);
        
        // Set window properties with larger size
        setWindowTitle("WAM-V Fault Diagnosis Dashboard");
        resize(1600, 1200);  // Increased from 1200x800
        
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

void WAMVDashboard::setupUI()
{
    // Create central widget and main layout
    QWidget *central_widget = new QWidget(this);
    QVBoxLayout *main_layout = new QVBoxLayout(central_widget);
    
    // Create status indicator section
    QHBoxLayout *status_layout = new QHBoxLayout();
    QGroupBox *status_group = new QGroupBox("Fault Diagnosis Status");
    QHBoxLayout *status_group_layout = new QHBoxLayout(status_group);
    
    // Increase font size for the group box title
    QFont groupBoxFont = status_group->font();
    groupBoxFont.setPointSize(14);  // Increased from default
    groupBoxFont.setBold(true);
    status_group->setFont(groupBoxFont);
    
    status_indicator_ = new QFrame();
    status_indicator_->setFrameShape(QFrame::Box);
    status_indicator_->setFixedSize(70, 70);  // Increased size from 50x50
    status_indicator_->setStyleSheet("background-color: green;");
    
    QVBoxLayout *status_text_layout = new QVBoxLayout();
    fault_status_label_ = new QLabel("Status: NO_FAULT");
    
    // Replace single confidence label with multiple confidence labels
    QFont confidence_font;
    confidence_font.setPointSize(12);  // Increased from 10
    
    no_fault_confidence_label_ = new QLabel("NO_FAULT: 0.0%");
    no_fault_confidence_label_->setFont(confidence_font);
    
    left_fault_confidence_label_ = new QLabel("LEFT_THRUST: 0.0%");
    left_fault_confidence_label_->setFont(confidence_font);
    
    right_fault_confidence_label_ = new QLabel("RIGHT_THRUST: 0.0%");
    right_fault_confidence_label_->setFont(confidence_font);
    
    QFont status_font = fault_status_label_->font();
    status_font.setPointSize(16);  // Increased from 12
    status_font.setBold(true);
    fault_status_label_->setFont(status_font);
    
    status_text_layout->addWidget(fault_status_label_);
    status_text_layout->addWidget(no_fault_confidence_label_);
    status_text_layout->addWidget(left_fault_confidence_label_);
    status_text_layout->addWidget(right_fault_confidence_label_);
    
    status_group_layout->addWidget(status_indicator_);
    status_group_layout->addLayout(status_text_layout);
    status_group_layout->addStretch();
    
    reset_button_ = new QPushButton("Reset Trajectory");
    reset_button_->setFont(confidence_font);  // Match font size with confidence labels
    connect(reset_button_, &QPushButton::clicked, this, &WAMVDashboard::resetTrajectory);
    status_group_layout->addWidget(reset_button_);
    
    status_layout->addWidget(status_group);
    
    // Create charts
    setupCharts();
    
    // Create layout for charts
    QGridLayout *charts_layout = new QGridLayout();
    
    // Create container for disturbance charts
    QVBoxLayout *disturbance_layout = new QVBoxLayout();
    disturbance_layout->addWidget(wx_view_);
    disturbance_layout->addWidget(wy_view_);
    disturbance_layout->addWidget(wpsi_view_);
    
    // Add both layouts to the grid
    QWidget *disturbance_widget = new QWidget();
    disturbance_widget->setLayout(disturbance_layout);
    
    charts_layout->addWidget(disturbance_widget, 0, 0);
    charts_layout->addWidget(trajectory_view_, 0, 1);
    
    // Set column stretch to make the trajectory view a bit larger
    charts_layout->setColumnStretch(0, 1);
    charts_layout->setColumnStretch(1, 1);
    
    // Add layouts to main layout
    main_layout->addLayout(status_layout);
    main_layout->addLayout(charts_layout);
    
    // Set central widget
    setCentralWidget(central_widget);
}

void WAMVDashboard::setupCharts()
{
    // Font settings for better readability
    QFont axisFont;
    axisFont.setPointSize(12);
    
    QFont titleFont;
    titleFont.setPointSize(14);
    titleFont.setBold(true);
    
    // Setup wx chart
    wx_chart_ = new QChart();
    wx_chart_->setTitle("w_x (Linear X Disturbance)");
    wx_chart_->setTitleFont(titleFont);
    
    wx_series_ = new QLineSeries();
    wx_series_->setName("w_x");
    wx_series_->setPen(QPen(QColor(0, 0, 255), 3));  // Thicker blue line
    
    wx_chart_->addSeries(wx_series_);
    wx_chart_->createDefaultAxes();
    wx_chart_->axes(Qt::Horizontal).first()->setTitleText("Time (s)");
    wx_chart_->axes(Qt::Vertical).first()->setTitleText("wx");  // Simplified label
    
    QAbstractAxis* wxXAxis = wx_chart_->axes(Qt::Horizontal).first();
    QAbstractAxis* wxYAxis = wx_chart_->axes(Qt::Vertical).first();
    wxXAxis->setTitleFont(axisFont);
    wxYAxis->setTitleFont(axisFont);
    wxXAxis->setLabelsFont(axisFont);
    wxYAxis->setLabelsFont(axisFont);
    
    wx_view_ = new QChartView(wx_chart_);
    wx_view_->setRenderHint(QPainter::Antialiasing);
    
    // Setup wy chart (similar font changes)
    wy_chart_ = new QChart();
    wy_chart_->setTitle("w_y (Linear Y Disturbance)");
    wy_chart_->setTitleFont(titleFont);
    
    wy_series_ = new QLineSeries();
    wy_series_->setName("w_y");
    wy_series_->setPen(QPen(QColor(0, 0, 255), 3));  // Thicker blue line
    
    wy_chart_->addSeries(wy_series_);
    wy_chart_->createDefaultAxes();
    wy_chart_->axes(Qt::Horizontal).first()->setTitleText("Time (s)");
    wy_chart_->axes(Qt::Vertical).first()->setTitleText("wy");  // Simplified label
    
    QAbstractAxis* wyXAxis = wy_chart_->axes(Qt::Horizontal).first();
    QAbstractAxis* wyYAxis = wy_chart_->axes(Qt::Vertical).first();
    wyXAxis->setTitleFont(axisFont);
    wyYAxis->setTitleFont(axisFont);
    wyXAxis->setLabelsFont(axisFont);
    wyYAxis->setLabelsFont(axisFont);
    
    wy_view_ = new QChartView(wy_chart_);
    wy_view_->setRenderHint(QPainter::Antialiasing);
    
    // Setup wpsi chart (similar font changes)
    wpsi_chart_ = new QChart();
    wpsi_chart_->setTitle("w_psi (Angular Z Disturbance)");
    wpsi_chart_->setTitleFont(titleFont);

    wpsi_series_ = new QLineSeries();
    wpsi_series_->setName("w_psi");
    wpsi_series_->setPen(QPen(QColor(0, 0, 255), 3));  // Thicker blue line

    wpsi_chart_->addSeries(wpsi_series_);
    wpsi_chart_->createDefaultAxes();
    wpsi_chart_->axes(Qt::Horizontal).first()->setTitleText("Time (s)");
    wpsi_chart_->axes(Qt::Vertical).first()->setTitleText("wpsi");  // Simplified label

    QAbstractAxis* wpsiXAxis = wpsi_chart_->axes(Qt::Horizontal).first();
    QAbstractAxis* wpsiYAxis = wpsi_chart_->axes(Qt::Vertical).first();
    wpsiXAxis->setTitleFont(axisFont);
    wpsiYAxis->setTitleFont(axisFont);
    wpsiXAxis->setLabelsFont(axisFont);
    wpsiYAxis->setLabelsFont(axisFont);
    
    wpsi_view_ = new QChartView(wpsi_chart_);
    wpsi_view_->setRenderHint(QPainter::Antialiasing);

    // Make legends visible for all charts with bigger font
    QFont legendFont;
    legendFont.setPointSize(12);
    
    wx_chart_->legend()->setVisible(true);
    wx_chart_->legend()->setFont(legendFont);
    wy_chart_->legend()->setVisible(true);
    wy_chart_->legend()->setFont(legendFont);
    wpsi_chart_->legend()->setVisible(true);
    wpsi_chart_->legend()->setFont(legendFont);
    
    // Setup trajectory chart
    trajectory_chart_ = new QChart();
    trajectory_chart_->setTitle("USV Trajectory");
    trajectory_chart_->setTitleFont(titleFont);
    
    // Main trajectory series (blue dots for past positions)
    trajectory_series_ = new QScatterSeries();
    trajectory_series_->setName("Path");
    trajectory_series_->setMarkerSize(10);
    trajectory_series_->setColor(QColor(75, 0, 130));  // purple
    
    // Add a series for current vessel position (green dot)
    vessel_orientation_series_ = new QScatterSeries();
    vessel_orientation_series_->setName("Current Position");
    vessel_orientation_series_->setMarkerSize(15);  // Larger marker
    vessel_orientation_series_->setColor(QColor(0, 170, 0));  // Green
    
    // Add series to chart (order matters for visual layering)
    trajectory_chart_->addSeries(trajectory_series_);
    trajectory_chart_->addSeries(vessel_orientation_series_);
    
    trajectory_chart_->createDefaultAxes();
    trajectory_chart_->axes(Qt::Horizontal).first()->setTitleText("X Position (m)");
    trajectory_chart_->axes(Qt::Vertical).first()->setTitleText("Y Position (m)");
    
    QAbstractAxis* trajXAxis = trajectory_chart_->axes(Qt::Horizontal).first();
    QAbstractAxis* trajYAxis = trajectory_chart_->axes(Qt::Vertical).first();
    trajXAxis->setTitleFont(axisFont);
    trajYAxis->setTitleFont(axisFont);
    trajXAxis->setLabelsFont(axisFont);
    trajYAxis->setLabelsFont(axisFont);
    
    // Set chart axes to be equal to preserve aspect ratio with smaller range
    QValueAxis *x_axis = qobject_cast<QValueAxis*>(trajectory_chart_->axes(Qt::Horizontal).first());
    QValueAxis *y_axis = qobject_cast<QValueAxis*>(trajectory_chart_->axes(Qt::Vertical).first());
    
    if (x_axis && y_axis) {
        x_axis->setRange(-20, 20);  // Reduced from -50,50 for better visibility
        y_axis->setRange(-20, 20);  // Reduced from -50,50 for better visibility
    }
    
    trajectory_view_ = new QChartView(trajectory_chart_);
    trajectory_view_->setRenderHint(QPainter::Antialiasing);
}

void WAMVDashboard::handleFaultConfidenceMsg(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() >= 3) {
        no_fault_confidence_ = msg->data[0] * 100.0; // Convert to percentage
        left_fault_confidence_ = msg->data[1] * 100.0;
        right_fault_confidence_ = msg->data[2] * 100.0;
        
        // Update the labels
        no_fault_confidence_label_->setText(QString("NO_FAULT: %1%").arg(no_fault_confidence_, 0, 'f', 1));
        left_fault_confidence_label_->setText(QString("LEFT_THRUST: %1%").arg(left_fault_confidence_, 0, 'f', 1));
        right_fault_confidence_label_->setText(QString("RIGHT_THRUST: %1%").arg(right_fault_confidence_, 0, 'f', 1));
    }
}

void WAMVDashboard::handleDisturbanceMsg(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    // Get current time
    auto time_point = node_ptr_->now();
    double current_time = time_point.seconds();
    
    // If this is the first message, set the reference time
    if (time_data_.empty()) {
        start_time_ = current_time;
        time_data_.push_back(0.0);
    } else {
        // Calculate relative time since start
        time_data_.push_back(current_time - start_time_);
    }
    
    // Store disturbance values
    wx_data_.push_back(msg->twist.linear.x);
    wy_data_.push_back(msg->twist.linear.y);
    wpsi_data_.push_back(msg->twist.angular.z);
    
    // Calculate calibrated wpsi (placeholder - actual calculation will depend on your implementation)
    // This should be replaced with actual calibration logic based on your system
    double calibrated_wpsi = msg->twist.angular.z;  // Replace with actual calibration
    wpsi_calibrated_data_.push_back(calibrated_wpsi);
    
    // Limit data size
    if (time_data_.size() > static_cast<size_t>(MAX_DATA_POINTS)) {
        time_data_.pop_front();
        wx_data_.pop_front();
        wy_data_.pop_front();
        wpsi_data_.pop_front();
        wpsi_calibrated_data_.pop_front();
    }
    // Add the current fault status to history
    fault_active_history_.push_back(current_fault_status_ != "NO_FAULT");
    
    // Limit data size
    if (fault_active_history_.size() > static_cast<size_t>(MAX_DATA_POINTS)) {
        fault_active_history_.pop_front();
    }
}

void WAMVDashboard::handleOdomMsg(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Extract position
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    
    // Extract orientation (yaw)
    tf2::Quaternion quat(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    
    // Convert quaternion to RPY
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    
    // Store trajectory points
    trajectory_x_.push_back(x);
    trajectory_y_.push_back(y);
    trajectory_yaw_.push_back(yaw);
    
    // Update current vessel position marker
    if (vessel_orientation_series_) {
        vessel_orientation_series_->clear();
        vessel_orientation_series_->append(x, y);
    }
    
    // Limit data size to prevent memory issues for long runs
    const size_t MAX_TRAJECTORY_POINTS = 10000;
    if (trajectory_x_.size() > MAX_TRAJECTORY_POINTS) {
        trajectory_x_.erase(trajectory_x_.begin());
        trajectory_y_.erase(trajectory_y_.begin());
        trajectory_yaw_.erase(trajectory_yaw_.begin());
    }
}

void WAMVDashboard::handleFaultDiagnosisMsg(const std_msgs::msg::String::SharedPtr msg)
{
    // Parse fault diagnosis message
    // Expected format: "Fault: FAULT_TYPE (Confidence: XX.X%)"
    std::string message = msg->data;
    
    size_t fault_pos = message.find("Fault: ");
    size_t confidence_pos = message.find("Confidence: ");
    
    if (fault_pos != std::string::npos && confidence_pos != std::string::npos) {
        // Extract fault type
        size_t fault_start = fault_pos + 7;  // Length of "Fault: "
        size_t fault_end = message.find(" ", fault_start);
        
        if (fault_end != std::string::npos) {
            current_fault_status_ = message.substr(fault_start, fault_end - fault_start);
        }
        
        // Extract confidence
        size_t confidence_start = confidence_pos + 12;  // Length of "Confidence: "
        size_t confidence_end = message.find("%", confidence_start);
        
        if (confidence_end != std::string::npos) {
            std::string confidence_str = message.substr(confidence_start, confidence_end - confidence_start);
            try {
                fault_confidence_ = std::stod(confidence_str);
            } catch (const std::exception& e) {
                RCLCPP_WARN(node_ptr_->get_logger(), "Failed to parse confidence value: %s", e.what());
            }
        }

        // Store previous fault status
        bool was_fault_active = (current_fault_status_ != "NO_FAULT");
        
        // Get new fault status
        bool is_fault_active = (current_fault_status_ != "NO_FAULT");
        
        // If transitioning from normal to fault, record the time
        if (!was_fault_active && is_fault_active) {
            fault_start_time_ = time_data_.empty() ? 0.0 : time_data_.back();
            RCLCPP_INFO(node_ptr_->get_logger(), "Fault detected at time: %.2f", fault_start_time_);
        }
        
        // Update the status display
        updateStatusDisplay();
    }
}

void WAMVDashboard::handleFaultFeaturesMsg(const std_msgs::msg::Float64MultiArray::SharedPtr /*msg*/)
{
    // Process fault features if needed
    // Currently this is a placeholder - you can use this data for additional visualization
}

void WAMVDashboard::handleWpsiCoefficientMsg(const std_msgs::msg::Float64::SharedPtr /*msg*/)
{
    // Process wpsi coefficient if needed
    // This could be used for the calibrated wpsi calculation
}

void WAMVDashboard::updatePlots()
{
    try {
        // Update trajectory plot
        if (trajectory_series_ && !trajectory_x_.empty() && !trajectory_y_.empty()) {
            // Update main trajectory series
            trajectory_series_->clear();
            for (size_t i = 0; i < trajectory_x_.size(); i++) {
                trajectory_series_->append(trajectory_x_[i], trajectory_y_[i]);
            }
            // Auto-adjust trajectory chart axes if needed
            if (!trajectory_x_.empty() && !trajectory_y_.empty()) {
                double min_x = *std::min_element(trajectory_x_.begin(), trajectory_x_.end());
                double max_x = *std::max_element(trajectory_x_.begin(), trajectory_x_.end());
                double min_y = *std::min_element(trajectory_y_.begin(), trajectory_y_.end());
                double max_y = *std::max_element(trajectory_y_.begin(), trajectory_y_.end());
                
                // Calculate the trajectory bounds
                double x_range = max_x - min_x;
                double y_range = max_y - min_y;
                double max_range = std::max(x_range, y_range);
                
                // Add smaller margin for tighter framing
                double margin = std::max(5.0, max_range * 0.15);
                
                // Calculate the center point
                double center_x = (min_x + max_x) / 2.0;
                double center_y = (min_y + max_y) / 2.0;
                
                // Set symmetric bounds around the center point
                double new_min_x = center_x - max_range / 2.0 - margin;
                double new_max_x = center_x + max_range / 2.0 + margin;
                double new_min_y = center_y - max_range / 2.0 - margin;
                double new_max_y = center_y + max_range / 2.0 + margin;
                
                QValueAxis *x_axis = qobject_cast<QValueAxis*>(trajectory_chart_->axes(Qt::Horizontal).first());
                QValueAxis *y_axis = qobject_cast<QValueAxis*>(trajectory_chart_->axes(Qt::Vertical).first());
                
                if (x_axis && y_axis) {
                    x_axis->setRange(new_min_x, new_max_x);
                    y_axis->setRange(new_min_y, new_max_y);
                }
            }
        }
        
        // Update wx chart with color-coded series
        if (wx_chart_ && !wx_data_.empty() && !time_data_.empty()) {
            // Create separate series for normal and fault conditions
            QLineSeries *wx_normal_series = new QLineSeries();
            QLineSeries *wx_fault_series = new QLineSeries();
            QScatterSeries *wx_transition_points = new QScatterSeries();
            
            // Set colors and styles with thicker lines
            QPen normalPen(QColor(0, 100, 255));  // Darker blue
            normalPen.setWidth(3);  // Thicker line
            
            QPen faultPen(QColor(255, 50, 50));   // Brighter red
            faultPen.setWidth(3);  // Thicker line
            
            wx_normal_series->setPen(normalPen);
            wx_normal_series->setName("Normal");
            
            wx_fault_series->setPen(faultPen);
            wx_fault_series->setName("Fault Active");
            
            wx_transition_points->setMarkerSize(12);  // Increased from 8
            wx_transition_points->setColor(QColor(255, 200, 0));  // Brighter yellow
            wx_transition_points->setName("Fault Start");
            
            // Fill the series with data
            for (size_t i = 0; i < std::min(wx_data_.size(), time_data_.size()); i++) {
                bool is_fault = (i < fault_active_history_.size()) ? fault_active_history_[i] : false;
                bool is_transition = (i > 0 && 
                                    i < fault_active_history_.size() && 
                                    !fault_active_history_[i-1] && 
                                    fault_active_history_[i]);
                if (is_fault) {
                    wx_fault_series->append(time_data_[i], wx_data_[i]);
                } else {
                    wx_normal_series->append(time_data_[i], wx_data_[i]);
                }
                if (is_transition) {
                    wx_transition_points->append(time_data_[i], wx_data_[i]);
                }
            }
            
            // Update the chart
            wx_chart_->removeAllSeries();
            wx_chart_->addSeries(wx_normal_series);
            wx_chart_->addSeries(wx_fault_series);
            wx_chart_->addSeries(wx_transition_points);
            
            // Re-attach axes
            wx_chart_->createDefaultAxes();
            wx_chart_->axes(Qt::Horizontal).first()->setTitleText("Time (s)");
            wx_chart_->axes(Qt::Vertical).first()->setTitleText("wx");  // Simplified label
            
            // Set larger fonts for axes
            QFont axisFont;
            axisFont.setPointSize(12);
            wx_chart_->axes(Qt::Horizontal).first()->setTitleFont(axisFont);
            wx_chart_->axes(Qt::Vertical).first()->setTitleFont(axisFont);
            wx_chart_->axes(Qt::Horizontal).first()->setLabelsFont(axisFont);
            wx_chart_->axes(Qt::Vertical).first()->setLabelsFont(axisFont);
            
            // Update time axis range
            QValueAxis *x_axis = qobject_cast<QValueAxis*>(wx_chart_->axes(Qt::Horizontal).first());
            if (x_axis) {
                double max_time = time_data_.back();
                double min_time = std::max(0.0, max_time - 10.0); // 10 second window
                x_axis->setRange(min_time, max_time);
            }
            
            // Update y-axis range
            updateValueAxis(wx_chart_, wx_data_);
        }
        
        // Update wy chart with color-coded series (same changes as for wx chart)
        if (wy_chart_ && !wy_data_.empty() && !time_data_.empty()) {
            // Create separate series for normal and fault conditions
            QLineSeries *wy_normal_series = new QLineSeries();
            QLineSeries *wy_fault_series = new QLineSeries();
            QScatterSeries *wy_transition_points = new QScatterSeries();
            
            // Set colors and styles with thicker lines
            QPen normalPen(QColor(0, 100, 255));  // Darker blue
            normalPen.setWidth(3);  // Thicker line
            
            QPen faultPen(QColor(255, 50, 50));   // Brighter red
            faultPen.setWidth(3);  // Thicker line
            
            wy_normal_series->setPen(normalPen);
            wy_normal_series->setName("Normal");
            
            wy_fault_series->setPen(faultPen);
            wy_fault_series->setName("Fault Active");
            
            wy_transition_points->setMarkerSize(12);  // Increased from 8
            wy_transition_points->setColor(QColor(255, 200, 0));  // Brighter yellow
            wy_transition_points->setName("Fault Start");
            
            // Fill the series with data
            for (size_t i = 0; i < std::min(wy_data_.size(), time_data_.size()); i++) {
                bool is_fault = (i < fault_active_history_.size()) ? fault_active_history_[i] : false;
                bool is_transition = (i > 0 && 
                                    i < fault_active_history_.size() && 
                                    !fault_active_history_[i-1] && 
                                    fault_active_history_[i]);
                if (is_fault) {
                    wy_fault_series->append(time_data_[i], wy_data_[i]);
                } else {
                    wy_normal_series->append(time_data_[i], wy_data_[i]);
                }
                if (is_transition) {
                    wy_transition_points->append(time_data_[i], wy_data_[i]);
                }
            }
            
            // Update the chart
            wy_chart_->removeAllSeries();
            wy_chart_->addSeries(wy_normal_series);
            wy_chart_->addSeries(wy_fault_series);
            wy_chart_->addSeries(wy_transition_points);
            
            // Re-attach axes
            wy_chart_->createDefaultAxes();
            wy_chart_->axes(Qt::Horizontal).first()->setTitleText("Time (s)");
            wy_chart_->axes(Qt::Vertical).first()->setTitleText("wy");  // Simplified label
            
            // Set larger fonts for axes
            QFont axisFont;
            axisFont.setPointSize(12);
            wy_chart_->axes(Qt::Horizontal).first()->setTitleFont(axisFont);
            wy_chart_->axes(Qt::Vertical).first()->setTitleFont(axisFont);
            wy_chart_->axes(Qt::Horizontal).first()->setLabelsFont(axisFont);
            wy_chart_->axes(Qt::Vertical).first()->setLabelsFont(axisFont);
            
            // Update time axis range
            QValueAxis *x_axis = qobject_cast<QValueAxis*>(wy_chart_->axes(Qt::Horizontal).first());
            if (x_axis) {
                double max_time = time_data_.back();
                double min_time = std::max(0.0, max_time - 10.0); // 10 second window
                x_axis->setRange(min_time, max_time);
            }
            
            // Update y-axis range
            updateValueAxis(wy_chart_, wy_data_);
        }
        
        // Update wpsi chart with color-coded series (same changes as for other charts)
        if (wpsi_chart_ && !wpsi_calibrated_data_.empty() && !time_data_.empty()) {
            // Create separate series for normal and fault conditions
            QLineSeries *wpsi_normal_series = new QLineSeries();
            QLineSeries *wpsi_fault_series = new QLineSeries();
            QScatterSeries *wpsi_transition_points = new QScatterSeries();
            
            // Set colors and styles with thicker lines
            QPen normalPen(QColor(0, 100, 255));  // Darker blue
            normalPen.setWidth(3);  // Thicker line
            
            QPen faultPen(QColor(255, 50, 50));   // Brighter red
            faultPen.setWidth(3);  // Thicker line
            
            wpsi_normal_series->setPen(normalPen);
            wpsi_normal_series->setName("Normal");
            
            wpsi_fault_series->setPen(faultPen);
            wpsi_fault_series->setName("Fault Active");
            
            wpsi_transition_points->setMarkerSize(12);  // Increased from 8
            wpsi_transition_points->setColor(QColor(255, 200, 0));  // Brighter yellow
            wpsi_transition_points->setName("Fault Start");
            
            // Fill the series with data
            for (size_t i = 0; i < std::min(wpsi_calibrated_data_.size(), time_data_.size()); i++) {
                bool is_fault = (i < fault_active_history_.size()) ? fault_active_history_[i] : false;
                bool is_transition = (i > 0 && 
                                    i < fault_active_history_.size() && 
                                    !fault_active_history_[i-1] && 
                                    fault_active_history_[i]);
                if (is_fault) {
                    wpsi_fault_series->append(time_data_[i], wpsi_calibrated_data_[i]);
                } else {
                    wpsi_normal_series->append(time_data_[i], wpsi_calibrated_data_[i]);
                }
                if (is_transition) {
                    wpsi_transition_points->append(time_data_[i], wpsi_calibrated_data_[i]);
                }
            }
            
            // Update the chart
            wpsi_chart_->removeAllSeries();
            wpsi_chart_->addSeries(wpsi_normal_series);
            wpsi_chart_->addSeries(wpsi_fault_series);
            wpsi_chart_->addSeries(wpsi_transition_points);
            
            // Re-attach axes
            wpsi_chart_->createDefaultAxes();
            wpsi_chart_->axes(Qt::Horizontal).first()->setTitleText("Time (s)");
            wpsi_chart_->axes(Qt::Vertical).first()->setTitleText("wpsi");  // Simplified label
            
            // Set larger fonts for axes
            QFont axisFont;
            axisFont.setPointSize(12);
            wpsi_chart_->axes(Qt::Horizontal).first()->setTitleFont(axisFont);
            wpsi_chart_->axes(Qt::Vertical).first()->setTitleFont(axisFont);
            wpsi_chart_->axes(Qt::Horizontal).first()->setLabelsFont(axisFont);
            wpsi_chart_->axes(Qt::Vertical).first()->setLabelsFont(axisFont);
            
            // Update time axis range
            QValueAxis *x_axis = qobject_cast<QValueAxis*>(wpsi_chart_->axes(Qt::Horizontal).first());
            if (x_axis) {
                double max_time = time_data_.back();
                double min_time = std::max(0.0, max_time - 10.0); // 10 second window
                x_axis->setRange(min_time, max_time);
            }
            
            // Update y-axis range
            updateValueAxis(wpsi_chart_, wpsi_calibrated_data_);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_ptr_->get_logger(), "Error in updatePlots: %s", e.what());
    } catch (...) {
        RCLCPP_ERROR(node_ptr_->get_logger(), "Unknown error in updatePlots");
    }
}



void WAMVDashboard::resetTrajectory()
{
    // Clear trajectory data
    trajectory_x_.clear();
    trajectory_y_.clear();
    trajectory_yaw_.clear();
    
    // Clear all trajectory-related series
    trajectory_series_->clear();
    vessel_orientation_series_->clear();
    
    // Reset trajectory chart axes
    QValueAxis *x_axis = qobject_cast<QValueAxis*>(trajectory_chart_->axes(Qt::Horizontal).first());
    QValueAxis *y_axis = qobject_cast<QValueAxis*>(trajectory_chart_->axes(Qt::Vertical).first());
    
    if (x_axis && y_axis) {
        x_axis->setRange(-20, 20);  // Use smaller initial range for better visibility
        y_axis->setRange(-20, 20);
    }
    
    // Notify user
    QMessageBox::information(this, "Trajectory Reset", "Trajectory display has been reset.");
}

void WAMVDashboard::updateStatusDisplay()
{
    // Update fault status label
    fault_status_label_->setText("Status: " + QString::fromStdString(current_fault_status_));
    
    // Update status indicator color
    QColor status_color = getFaultStatusColor();
    status_indicator_->setStyleSheet(QString("background-color: %1;").arg(status_color.name()));
}

QColor WAMVDashboard::getFaultStatusColor()
{
    // Choose color based on fault status and confidence
    if (current_fault_status_ == "NO_FAULT") {
        return QColor(0, 200, 0);  // Green for normal status
    } 
    else if (current_fault_status_ == "LEFT_THRUST_FAILURE") {
        // Red with intensity based on confidence (darker red for higher confidence)
        int green = static_cast<int>(100 * (1.0 - left_fault_confidence_ / 100.0));
        return QColor(255, green, 0);  // Red for left thrust failure
    } 
    else if (current_fault_status_ == "RIGHT_THRUST_FAILURE") {
        // Also use red for right thrust failure (previously would have been yellowish)
        int green = static_cast<int>(100 * (1.0 - right_fault_confidence_ / 100.0));
        return QColor(255, green, 0);  // Red for right thrust failure
    } 
    else {
        // Gray for unknown status
        return QColor(128, 128, 128);
    }
}

// Add helper methods for axis updates
void WAMVDashboard::updateTimeAxis(QChart *chart, double min_time, double max_time) {
    QValueAxis *x_axis = qobject_cast<QValueAxis*>(chart->axes(Qt::Horizontal).first());
    if (x_axis) {
        x_axis->setRange(min_time, max_time);
    }
}

void WAMVDashboard::updateValueAxis(QChart *chart, const std::deque<double> &data) {
    if (data.empty()) return;
    
    double min_val = *std::min_element(data.begin(), data.end());
    double max_val = *std::max_element(data.begin(), data.end());
    double margin = std::max(0.5, (max_val - min_val) * 0.1);
    
    QValueAxis *y_axis = qobject_cast<QValueAxis*>(chart->axes(Qt::Vertical).first());
    if (y_axis) {
        y_axis->setRange(min_val - margin, max_val + margin);
    }
}