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
        
        // Set window properties
        setWindowTitle("WAM-V Fault Diagnosis Dashboard");
        resize(1200, 800);
        
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
    
    status_indicator_ = new QFrame();
    status_indicator_->setFrameShape(QFrame::Box);
    status_indicator_->setFixedSize(50, 50);
    status_indicator_->setStyleSheet("background-color: green;");
    
    QVBoxLayout *status_text_layout = new QVBoxLayout();
    fault_status_label_ = new QLabel("Status: NO_FAULT");
    
    // Replace single confidence label with multiple confidence labels
    QFont confidence_font;
    confidence_font.setPointSize(10);
    
    no_fault_confidence_label_ = new QLabel("NO_FAULT: 0.0%");
    no_fault_confidence_label_->setFont(confidence_font);
    
    left_fault_confidence_label_ = new QLabel("LEFT_THRUST: 0.0%");
    left_fault_confidence_label_->setFont(confidence_font);
    
    right_fault_confidence_label_ = new QLabel("RIGHT_THRUST: 0.0%");
    right_fault_confidence_label_->setFont(confidence_font);
    
    QFont status_font = fault_status_label_->font();
    status_font.setPointSize(12);
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
    // Setup wx chart
    wx_chart_ = new QChart();
    wx_chart_->setTitle("w_x (Linear X Disturbance)");
    
    wx_series_ = new QLineSeries();
    wx_series_->setName("w_x");
    
    wx_chart_->addSeries(wx_series_);
    wx_chart_->createDefaultAxes();
    wx_chart_->axes(Qt::Horizontal).first()->setTitleText("Time (s)");
    wx_chart_->axes(Qt::Vertical).first()->setTitleText("w_x (N)");
    
    wx_view_ = new QChartView(wx_chart_);
    wx_view_->setRenderHint(QPainter::Antialiasing);
    
    // Setup wy chart
    wy_chart_ = new QChart();
    wy_chart_->setTitle("w_y (Linear Y Disturbance)");
    
    wy_series_ = new QLineSeries();
    wy_series_->setName("w_y");
    
    wy_chart_->addSeries(wy_series_);
    wy_chart_->createDefaultAxes();
    wy_chart_->axes(Qt::Horizontal).first()->setTitleText("Time (s)");
    wy_chart_->axes(Qt::Vertical).first()->setTitleText("w_y (N)");
    
    wy_view_ = new QChartView(wy_chart_);
    wy_view_->setRenderHint(QPainter::Antialiasing);
    
    // Setup wpsi chart
    wpsi_chart_ = new QChart();
    wpsi_chart_->setTitle("w_psi (Angular Z Disturbance)");

    // Only create one series for the calibrated value
    wpsi_series_ = new QLineSeries();
    wpsi_series_->setName("w_psi");  // Remove the "(calibrated)" label

    wpsi_chart_->addSeries(wpsi_series_);
    wpsi_chart_->createDefaultAxes();
    wpsi_chart_->axes(Qt::Horizontal).first()->setTitleText("Time (s)");
    wpsi_chart_->axes(Qt::Vertical).first()->setTitleText("w_psi (Nm)");

    wpsi_view_ = new QChartView(wpsi_chart_);
    wpsi_view_->setRenderHint(QPainter::Antialiasing);

    // wx_normal_series_->setName("Normal");
    // wx_fault_series_->setName("Fault Active");
    // wx_transition_points_->setName("Fault Start");

    // Make legends visible for all charts
    wx_chart_->legend()->setVisible(true);
    wy_chart_->legend()->setVisible(true);
    wpsi_chart_->legend()->setVisible(true);
    
    
    // Setup trajectory chart
    trajectory_chart_ = new QChart();
    trajectory_chart_->setTitle("USV Trajectory");
    
    trajectory_series_ = new QScatterSeries();
    trajectory_series_->setName("Position");
    trajectory_series_->setMarkerSize(5);
    
    trajectory_chart_->addSeries(trajectory_series_);
    trajectory_chart_->createDefaultAxes();
    trajectory_chart_->axes(Qt::Horizontal).first()->setTitleText("X Position (m)");
    trajectory_chart_->axes(Qt::Vertical).first()->setTitleText("Y Position (m)");
    
    // Set chart axes to be equal to preserve aspect ratio
    QValueAxis *x_axis = qobject_cast<QValueAxis*>(trajectory_chart_->axes(Qt::Horizontal).first());
    QValueAxis *y_axis = qobject_cast<QValueAxis*>(trajectory_chart_->axes(Qt::Vertical).first());
    
    if (x_axis && y_axis) {
        x_axis->setRange(-50, 50);
        y_axis->setRange(-50, 50);
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
    
    // Store trajectory points
    trajectory_x_.push_back(x);
    trajectory_y_.push_back(y);
    
    // Limit data size to prevent memory issues for long runs
    const size_t MAX_TRAJECTORY_POINTS = 10000;
    if (trajectory_x_.size() > MAX_TRAJECTORY_POINTS) {
        trajectory_x_.erase(trajectory_x_.begin());
        trajectory_y_.erase(trajectory_y_.begin());
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
                
                // Add some margin
                double margin_x = std::max(10.0, (max_x - min_x) * 0.1);
                double margin_y = std::max(10.0, (max_y - min_y) * 0.1);
                
                QValueAxis *x_axis = qobject_cast<QValueAxis*>(trajectory_chart_->axes(Qt::Horizontal).first());
                QValueAxis *y_axis = qobject_cast<QValueAxis*>(trajectory_chart_->axes(Qt::Vertical).first());
                
                if (x_axis && y_axis) {
                    x_axis->setRange(min_x - margin_x, max_x + margin_x);
                    y_axis->setRange(min_y - margin_y, max_y + margin_y);
                }
            }
        }
        
        // Update wx chart with color-coded series
        if (wx_chart_ && !wx_data_.empty() && !time_data_.empty()) {
            // Create separate series for normal and fault conditions
            QLineSeries *wx_normal_series = new QLineSeries();
            QLineSeries *wx_fault_series = new QLineSeries();
            QScatterSeries *wx_transition_points = new QScatterSeries();
            
            // Set colors and styles
            QPen normalPen(QColor(0, 0, 255));  // Blue
            normalPen.setWidth(2);
            
            QPen faultPen(QColor(255, 0, 0));   // Red
            faultPen.setWidth(2);
            
            wx_normal_series->setPen(normalPen);
            wx_normal_series->setName("Normal");
            
            wx_fault_series->setPen(faultPen);
            wx_fault_series->setName("Fault Active");
            
            wx_transition_points->setMarkerSize(8);
            wx_transition_points->setColor(QColor(255, 255, 0));  // Yellow
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
            wx_chart_->axes(Qt::Vertical).first()->setTitleText("w_x Value");
            
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
        
        // Update wy chart with color-coded series
        if (wy_chart_ && !wy_data_.empty() && !time_data_.empty()) {
            // Create separate series for normal and fault conditions
            QLineSeries *wy_normal_series = new QLineSeries();
            QLineSeries *wy_fault_series = new QLineSeries();
            QScatterSeries *wy_transition_points = new QScatterSeries();
            
            // Set colors and styles
            QPen normalPen(QColor(0, 0, 255));  // Blue
            normalPen.setWidth(2);
            
            QPen faultPen(QColor(255, 0, 0));   // Red
            faultPen.setWidth(2);
            
            wy_normal_series->setPen(normalPen);
            wy_normal_series->setName("Normal");
            
            wy_fault_series->setPen(faultPen);
            wy_fault_series->setName("Fault Active");
            
            wy_transition_points->setMarkerSize(8);
            wy_transition_points->setColor(QColor(255, 255, 0));  // Yellow
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
            wy_chart_->axes(Qt::Vertical).first()->setTitleText("w_y Value");
            
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
        
        // Update wpsi chart with color-coded series
        if (wpsi_chart_ && !wpsi_calibrated_data_.empty() && !time_data_.empty()) {
            // Create separate series for normal and fault conditions
            QLineSeries *wpsi_normal_series = new QLineSeries();
            QLineSeries *wpsi_fault_series = new QLineSeries();
            QScatterSeries *wpsi_transition_points = new QScatterSeries();
            
            // Set colors and styles
            QPen normalPen(QColor(0, 0, 255));  // Blue
            normalPen.setWidth(2);
            
            QPen faultPen(QColor(255, 0, 0));   // Red
            faultPen.setWidth(2);
            
            wpsi_normal_series->setPen(normalPen);
            wpsi_normal_series->setName("Normal");
            
            wpsi_fault_series->setPen(faultPen);
            wpsi_fault_series->setName("Fault Active");
            
            wpsi_transition_points->setMarkerSize(8);
            wpsi_transition_points->setColor(QColor(255, 255, 0));  // Yellow
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
            wpsi_chart_->axes(Qt::Vertical).first()->setTitleText("w_psi Value");
            
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
    
    // Reset trajectory chart axes
    QValueAxis *x_axis = qobject_cast<QValueAxis*>(trajectory_chart_->axes(Qt::Horizontal).first());
    QValueAxis *y_axis = qobject_cast<QValueAxis*>(trajectory_chart_->axes(Qt::Vertical).first());
    
    if (x_axis && y_axis) {
        x_axis->setRange(-50, 50);
        y_axis->setRange(-50, 50);
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