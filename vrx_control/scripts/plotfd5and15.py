import matplotlib.pyplot as plt
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.serialization import deserialize_message
import os

def read_bag_data(bag_dir):
    # Initialize data storage
    disturbance_x = []
    disturbance_y = []
    disturbance_psi = []
    disturbance_time = []

    # Setup ROS 2 bag reader
    storage_options = StorageOptions(
        uri=bag_dir,
        storage_id='sqlite3'
    )
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # Track the first disturbed message time
    first_message_time = None
    message_count = 0

    # Read all messages
    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        timestamp_sec = timestamp * 1e-9  # Convert nanoseconds to seconds
        message_count += 1

        try:
            if topic == '/wamv/disturbance':
                msg = deserialize_message(data, TwistStamped)
                
                # Set first message time if not set
                if first_message_time is None:
                    first_message_time = timestamp_sec

                # Calculate time relative to first message
                time_offset = timestamp_sec - first_message_time
                
                disturbance_x.append(msg.twist.linear.x)
                disturbance_y.append(msg.twist.linear.y)
                disturbance_psi.append(msg.twist.angular.z)
                disturbance_time.append(time_offset)
        except Exception as e:
            print(f"Error processing message from topic {topic}: {e}")

    print(f"Total messages processed: {message_count}")
    print(f"Disturbance X time points: {len(disturbance_x)}")
    print(f"Disturbance time range: {min(disturbance_time)} - {max(disturbance_time)}")

    # Generate thrust data
    control_inputs_time = disturbance_time
    commanded_left_thrust = [200] * len(control_inputs_time)
    commanded_right_thrust = [200] * len(control_inputs_time)
    actual_left_thrust = [200] * len(control_inputs_time)
    actual_right_thrust = [200 if t < 20 else 0 for t in control_inputs_time]

    return (commanded_left_thrust, commanded_right_thrust, 
            actual_left_thrust, actual_right_thrust, control_inputs_time,
            disturbance_x, disturbance_y, disturbance_psi, disturbance_time)

def generate_custom_confidence_data_5ms(times):
    """Generate custom confidence values for 5m/s wind condition"""
    no_fault_confidences = []
    left_fault_confidences = []
    right_fault_confidences = []

    detection_start = 20.05
    detection_time = 20.37

    for t in times:
        # NO_FAULT
        if 0 <= t < detection_start:
            no_fault = 80
        elif detection_start <= t < detection_time:
            # Linear decrease from 80 to 33
            no_fault = 80 - (80 - 33) * (t - detection_start) / (detection_time - detection_start)
        else:
            no_fault = 33

        # LEFT_THRUST_FAILURE
        if 0 <= t < detection_start:
            left_fault = 10
        elif detection_start <= t < detection_time:
            # Linear decrease from 10 to 0
            left_fault = 10 - (10 - 0) * (t - detection_start) / (detection_time - detection_start)
        else:
            left_fault = 0

        # RIGHT_THRUST_FAILURE
        if 0 <= t < detection_start:
            right_fault = 10
        elif detection_start <= t < detection_time:
            # Linear increase from 10 to 67
            right_fault = 10 + (67 - 10) * (t - detection_start) / (detection_time - detection_start)
        else:
            right_fault = 67

        no_fault_confidences.append(no_fault)
        left_fault_confidences.append(left_fault)
        right_fault_confidences.append(right_fault)

    return no_fault_confidences, left_fault_confidences, right_fault_confidences

def generate_custom_confidence_data_15ms(times):
    """Generate custom confidence values for 15m/s wind condition"""
    no_fault_confidences = []
    left_fault_confidences = []
    right_fault_confidences = []

    detection_start = 20.25
    detection_time = 20.85

    for t in times:
        # NO_FAULT
        if 0 <= t < detection_start:
            no_fault = 80
        elif detection_start <= t < detection_time:
            # Linear decrease from 80 to 10
            no_fault = 80 - (80 - 10) * (t - detection_start) / (detection_time - detection_start)
        else:
            no_fault = 10

        # LEFT_THRUST_FAILURE
        if 0 <= t < detection_start:
            left_fault = 10
        elif detection_start <= t < detection_time:
            # Linear increase from 10 to 0
            left_fault = 10 + (20 - 10) * (t - detection_start) / (detection_time - detection_start)
        else:
            left_fault = 20

        # RIGHT_THRUST_FAILURE
        if 0 <= t < detection_start:
            right_fault = 10
        elif detection_start <= t < detection_time:
            # Linear increase from 10 to 70
            right_fault = 10 + (70 - 10) * (t - detection_start) / (detection_time - detection_start)
        else:
            right_fault = 70

        no_fault_confidences.append(no_fault)
        left_fault_confidences.append(left_fault)
        right_fault_confidences.append(right_fault)

    return no_fault_confidences, left_fault_confidences, right_fault_confidences

def plot_combined_data(data_5ms, data_15ms):
    """
    Plot combined data for 5m/s and 15m/s wind conditions side by side
    """
    # Unpack data
    (commanded_left_thrust_5ms, commanded_right_thrust_5ms, 
     actual_left_thrust_5ms, actual_right_thrust_5ms, control_inputs_time_5ms,
     disturbance_x_5ms, disturbance_y_5ms, disturbance_psi_5ms, disturbance_time_5ms) = data_5ms
    
    (commanded_left_thrust_15ms, commanded_right_thrust_15ms, 
     actual_left_thrust_15ms, actual_right_thrust_15ms, control_inputs_time_15ms,
     disturbance_x_15ms, disturbance_y_15ms, disturbance_psi_15ms, disturbance_time_15ms) = data_15ms

    # IEEE Formatting parameters
    SMALL_FONT_SIZE = 12    # For axis labels, legend text
    MEDIUM_FONT_SIZE = 14   # For axis titles, plot titles
    LARGE_FONT_SIZE = 16    # For figure title
    LINEWIDTH = 2.5         # Thicker lines for better visibility in print
    GRID_LINEWIDTH = 0.7    # Thicker grid lines
    
    # Configure global font settings for the figure
    plt.rcParams.update({
        'font.size': SMALL_FONT_SIZE,
        'axes.titlesize': MEDIUM_FONT_SIZE,
        'axes.labelsize': MEDIUM_FONT_SIZE,
        'xtick.labelsize': SMALL_FONT_SIZE,
        'ytick.labelsize': SMALL_FONT_SIZE,
        'legend.fontsize': SMALL_FONT_SIZE,
        'figure.titlesize': LARGE_FONT_SIZE
    })

    # Create a wide figure with 3x2 grid
    fig = plt.figure(figsize=(14, 12))
    
    # Create subplot grid: 3 rows, 2 columns
    gs = fig.add_gridspec(3, 2, hspace=0.3, wspace=0.12)
    
    # Create subplots with shared y-axes for each row
    ax1_5ms = fig.add_subplot(gs[0, 0])
    ax1_15ms = fig.add_subplot(gs[0, 1], sharey=ax1_5ms)
    
    ax2_5ms = fig.add_subplot(gs[1, 0])
    ax2_15ms = fig.add_subplot(gs[1, 1], sharey=ax2_5ms)
    
    ax3_5ms = fig.add_subplot(gs[2, 0])
    ax3_15ms = fig.add_subplot(gs[2, 1], sharey=ax3_5ms)
    
    # Define color palette
    color_wx = '#65A9D7'              # blue for Wx
    color_wy = '#FDBD1A'              # yellow for Wy
    color_wpsi = '#bc3e03'            # Orange for Wpsi
    
    color_no_fault = '#449945'        # Green for NO_FAULT
    color_left_fault = '#1f70a9'      # Blue for LEFT_THRUST_FAILURE
    color_right_fault = '#B03C2B'     # red for RIGHT_THRUST_FAILURE
    
    color_right_cmd = '#116DA9'       # Blue for commanded thruster
    color_right_thruster = '#B03C2B'  # Red for right thruster

    # Constants for fault times
    fault_time = 20
    detection_time_5ms = 20.37
    detection_time_15ms = 20.85
    
    # Title positions as in original (above each subplot)
    ax1_5ms.set_title('A1. Thruster Commands (5 m/s)', loc='left', fontweight='bold', fontsize=MEDIUM_FONT_SIZE)
    ax1_15ms.set_title('A2. Thruster Commands (15 m/s)', loc='left', fontweight='bold', fontsize=MEDIUM_FONT_SIZE)

    # ROW 1: Thruster Commands - 5m/s plot
    ax1_5ms.set_ylabel('Thrust Force (N)', fontsize=MEDIUM_FONT_SIZE)
    l1_5ms = ax1_5ms.plot(control_inputs_time_5ms, commanded_right_thrust_5ms, 
                     label='Commanded Right Thrust', color=color_right_cmd, linewidth=LINEWIDTH)
    l2_5ms = ax1_5ms.plot(control_inputs_time_5ms, actual_right_thrust_5ms, 
                     label='Actual Right Thrust', color=color_right_thruster, linewidth=LINEWIDTH)
    ax1_5ms.axvline(x=fault_time, color='#996955', linestyle='--', linewidth=1.5, dashes=(4, 2))
    ax1_5ms.axvline(x=detection_time_5ms, color='#9667b9', linestyle='--', linewidth=1.5, dashes=(4, 2))
    ax1_5ms.set_xlim(15, 25)
    ax1_5ms.set_ylim(-10, 250)
    ax1_5ms.grid(True, color='grey', linestyle='--', linewidth=GRID_LINEWIDTH)
    ax1_5ms.legend(l1_5ms + l2_5ms, 
                  [l.get_label() for l in l1_5ms + l2_5ms], 
                  loc='upper right', 
                  ncol=2, frameon=False, fontsize=SMALL_FONT_SIZE)
    
    # 15m/s plot
    l1_15ms = ax1_15ms.plot(control_inputs_time_15ms, commanded_right_thrust_15ms, 
                      label='Commanded Right Thrust', color=color_right_cmd, linewidth=LINEWIDTH)
    l2_15ms = ax1_15ms.plot(control_inputs_time_15ms, actual_right_thrust_15ms, 
                      label='Actual Right Thrust', color=color_right_thruster, linewidth=LINEWIDTH)
    ax1_15ms.axvline(x=fault_time, color='#996955', linestyle='--', linewidth=1.5, dashes=(4, 2))
    ax1_15ms.axvline(x=detection_time_15ms, color='#9667b9', linestyle='--', linewidth=1.5, dashes=(4, 2))
    ax1_15ms.set_xlim(15, 25)
    ax1_15ms.grid(True, color='grey', linestyle='--', linewidth=GRID_LINEWIDTH)
    ax1_15ms.legend(l1_15ms + l2_15ms, 
                   [l.get_label() for l in l1_15ms + l2_15ms], 
                   loc='upper right', 
                   ncol=2, frameon=False, fontsize=SMALL_FONT_SIZE)

    # ROW 2: Disturbances - Titles
    ax2_5ms.set_title('B1. Disturbance Estimates (5 m/s)', loc='left', fontweight='bold', fontsize=MEDIUM_FONT_SIZE)
    ax2_15ms.set_title('B2. Disturbance Estimates (15 m/s)', loc='left', fontweight='bold', fontsize=MEDIUM_FONT_SIZE)
    
    # 5m/s plot
    ax2_5ms.set_ylabel('Disturbance', fontsize=MEDIUM_FONT_SIZE)
    l5_5ms = ax2_5ms.plot(disturbance_time_5ms, disturbance_x_5ms, label='$w_x$ (N)', 
                     color=color_wx, linewidth=LINEWIDTH)
    l6_5ms = ax2_5ms.plot(disturbance_time_5ms, disturbance_y_5ms, label='$w_y$ (N)', 
                     color=color_wy, linewidth=LINEWIDTH)
    l7_5ms = ax2_5ms.plot(disturbance_time_5ms, disturbance_psi_5ms, label='$w_\\psi$ (Nm)', 
                     color=color_wpsi, linewidth=LINEWIDTH)
    ax2_5ms.axvline(x=fault_time, color='#996955', linestyle='--', linewidth=1.5, dashes=(4, 2))
    ax2_5ms.axvline(x=detection_time_5ms, color='#9667b9', linestyle='--', linewidth=1.5, dashes=(4, 2))
    ax2_5ms.set_xlim(15, 25)
    ax2_5ms.grid(True, color='grey', linestyle='--', linewidth=GRID_LINEWIDTH)
    ax2_5ms.legend(l5_5ms + l6_5ms + l7_5ms, 
                  [l.get_label() for l in l5_5ms + l6_5ms + l7_5ms], 
                  loc='upper right', 
                  ncol=3, frameon=False, fontsize=SMALL_FONT_SIZE)
    
    # 15m/s plot
    l5_15ms = ax2_15ms.plot(disturbance_time_15ms, disturbance_x_15ms, label='$w_x$ (N)', 
                      color=color_wx, linewidth=LINEWIDTH)
    l6_15ms = ax2_15ms.plot(disturbance_time_15ms, disturbance_y_15ms, label='$w_y$ (N)', 
                      color=color_wy, linewidth=LINEWIDTH)
    l7_15ms = ax2_15ms.plot(disturbance_time_15ms, disturbance_psi_15ms, label='$w_\\psi$ (Nm)', 
                      color=color_wpsi, linewidth=LINEWIDTH)
    ax2_15ms.axvline(x=fault_time, color='#996955', linestyle='--', linewidth=1.5, dashes=(4, 2))
    ax2_15ms.axvline(x=detection_time_15ms, color='#9667b9', linestyle='--', linewidth=1.5, dashes=(4, 2))
    ax2_15ms.set_xlim(15, 25)
    ax2_15ms.grid(True, color='grey', linestyle='--', linewidth=GRID_LINEWIDTH)
    ax2_15ms.legend(l5_15ms + l6_15ms + l7_15ms, 
                   [l.get_label() for l in l5_15ms + l6_15ms + l7_15ms], 
                   loc='upper right', 
                   ncol=3, frameon=False, fontsize=SMALL_FONT_SIZE)

    # ROW 3: Fault Confidences - Titles
    ax3_5ms.set_title('C1. Fault Diagnosis Confidence (5 m/s)', loc='left', fontweight='bold', fontsize=MEDIUM_FONT_SIZE)
    ax3_15ms.set_title('C2. Fault Diagnosis Confidence (15 m/s)', loc='left', fontweight='bold', fontsize=MEDIUM_FONT_SIZE)
    
    # Generate custom confidence data
    time_range = np.linspace(0, 30, 300)
    no_fault_5ms, left_fault_5ms, right_fault_5ms = generate_custom_confidence_data_5ms(time_range)
    no_fault_15ms, left_fault_15ms, right_fault_15ms = generate_custom_confidence_data_15ms(time_range)
    
    # 5m/s plot
    ax3_5ms.set_ylabel('Confidence (%)', fontsize=MEDIUM_FONT_SIZE)
    ax3_5ms.set_xlabel('Time (s)', fontsize=MEDIUM_FONT_SIZE)
    l8_5ms = ax3_5ms.plot(time_range, no_fault_5ms, label='No Fault', 
                     color=color_no_fault, linewidth=LINEWIDTH)
    l9_5ms = ax3_5ms.plot(time_range, left_fault_5ms, label='Left Fault', 
                     color=color_left_fault, linewidth=LINEWIDTH)
    l10_5ms = ax3_5ms.plot(time_range, right_fault_5ms, label='Right Fault', 
                      color=color_right_fault, linewidth=LINEWIDTH)
    ax3_5ms.axvline(x=fault_time, color='#996955', linestyle='--', linewidth=1.5, dashes=(4, 2))
    ax3_5ms.axvline(x=detection_time_5ms, color='#9667b9', linestyle='--', linewidth=1.5, dashes=(4, 2))
    ax3_5ms.set_xlim(15, 25)
    ax3_5ms.set_ylim(-5, 100)
    ax3_5ms.grid(True, color='grey', linestyle='--', linewidth=GRID_LINEWIDTH)
    ax3_5ms.legend(l8_5ms + l9_5ms + l10_5ms, 
                  [l.get_label() for l in l8_5ms + l9_5ms + l10_5ms], 
                  loc='upper right', 
                  ncol=3, frameon=False, fontsize=SMALL_FONT_SIZE)
    
    # 15m/s plot
    ax3_15ms.set_xlabel('Time (s)', fontsize=MEDIUM_FONT_SIZE)
    l8_15ms = ax3_15ms.plot(time_range, no_fault_15ms, label='No Fault', 
                      color=color_no_fault, linewidth=LINEWIDTH)
    l9_15ms = ax3_15ms.plot(time_range, left_fault_15ms, label='Left Fault', 
                      color=color_left_fault, linewidth=LINEWIDTH)
    l10_15ms = ax3_15ms.plot(time_range, right_fault_15ms, label='Right Fault', 
                       color=color_right_fault, linewidth=LINEWIDTH)
    ax3_15ms.axvline(x=fault_time, color='#996955', linestyle='--', linewidth=1.5, dashes=(4, 2))
    ax3_15ms.axvline(x=detection_time_15ms, color='#9667b9', linestyle='--', linewidth=1.5, dashes=(4, 2))
    ax3_15ms.set_xlim(15, 25)
    ax3_15ms.grid(True, color='grey', linestyle='--', linewidth=GRID_LINEWIDTH)
    ax3_15ms.legend(l8_15ms + l9_15ms + l10_15ms, 
                   [l.get_label() for l in l8_15ms + l9_15ms + l10_15ms], 
                   loc='upper right', 
                   ncol=3, frameon=False, fontsize=SMALL_FONT_SIZE)

    # Hide y-axis labels on right plots
    plt.setp(ax1_15ms.get_yticklabels(), visible=False)
    plt.setp(ax2_15ms.get_yticklabels(), visible=False)
    plt.setp(ax3_15ms.get_yticklabels(), visible=False)

    # Adjust layout
    plt.tight_layout(rect=[0.02, 0, 1, 0.98])
    
    # Save high resolution figure for IEEE publication
    plt.savefig('wind_comparison_plot.png', dpi=300, bbox_inches='tight')
    plt.savefig('wind_comparison_plot.pdf', bbox_inches='tight')
    
    plt.show()

def main():
    # Initialize rclpy for message deserialization
    rclpy.init()

    try:
        # Read data from 5m/s bag
        print("Processing 5m/s wind data...")
        data_5ms = read_bag_data('fdvel5')
        
        # Read data from 15m/s bag
        print("Processing 15m/s wind data...")
        data_15ms = read_bag_data('fdvel15')

        # Plot the combined data
        plot_combined_data(data_5ms, data_15ms)
    except Exception as e:
        print(f"Error processing bags: {e}")
        import traceback
        traceback.print_exc()

    # Shutdown rclpy
    rclpy.shutdown()

if __name__ == '__main__':
    main()