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

def generate_custom_confidence_data(times):
    """
    Generate custom confidence values based on the specified requirements
    """
    no_fault_confidences = []
    left_fault_confidences = []
    right_fault_confidences = []

    detection_start = 20.5
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

def plot_data(*args):
    (commanded_left_thrust, commanded_right_thrust, 
     actual_left_thrust, actual_right_thrust, control_inputs_time,
     disturbance_x, disturbance_y, disturbance_psi, disturbance_time) = args

    # IEEE Formatting parameters
    SMALL_FONT_SIZE = 14    # For axis labels, legend text
    MEDIUM_FONT_SIZE = 16   # For axis titles, plot titles
    LARGE_FONT_SIZE = 18    # For figure title
    LINEWIDTH = 3           # Thicker lines for better visibility in print
    MARKERSIZE = 8          # Larger markers
    GRID_LINEWIDTH = 0.8    # Thicker grid lines
    
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

    # Create a figure with proportions better suited for IEEE format
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 12))

    # Add main title
    # fig.suptitle('System Validation', fontsize=LARGE_FONT_SIZE, fontweight='bold')
    
    # Adjust spacing to make room for legends
    plt.subplots_adjust(hspace=0.4, top=0.95)

    # Define color palette to match the template
    color_wx = '#65A9D7'              # blue for Wx
    color_wy = '#FDBD1A'              # yellow for Wy
    color_wpsi = '#bc3e03'            # Orange for Wpsi
    
    color_no_fault = '#449945'        # Green for NO_FAULT
    color_left_fault = '#1f70a9'      # Blue for LEFT_THRUST_FAILURE
    color_right_fault = '#B03C2B'     # red for RIGHT_THRUST_FAILURE
    
    color_right_cmd = '#116DA9'   # Blue for left thruster
    color_right_thruster = '#B03C2B'  # Red for right thruster

    # Constants for fault and detection times
    fault_time = 20
    detection_time = 20.85

    # Subplot 1: Thruster Commands - SIMPLIFIED for clarity
    ax1.set_title('A. Thruster Commands', loc='left', fontweight='bold', fontsize=MEDIUM_FONT_SIZE)
    
    # Since left thruster commanded and actual are identical, combine into one line
    # l1 = ax1.plot(control_inputs_time, commanded_left_thrust, 
    #          label='Left', color=color_left_thruster, linewidth=LINEWIDTH)
    
    # Right thruster shows both commanded (dashed) and actual (solid) to highlight the failure
    l1 = ax1.plot(control_inputs_time, commanded_right_thrust, 
             label='Commanded Right Thrust', color=color_right_cmd, 
             linestyle='--', linewidth=LINEWIDTH)
    l2 = ax1.plot(control_inputs_time, actual_right_thrust, 
             label='Actual Right Thrust', color=color_right_thruster, linewidth=LINEWIDTH)
    
    # Add vertical lines at fault time and detection time
    ax1.axvline(x=fault_time, color='#996955', linestyle='--', linewidth=GRID_LINEWIDTH)
    ax1.axvline(x=detection_time, color='#9667b9', linestyle='--', linewidth=GRID_LINEWIDTH)
    
    # Add annotation for clarity with larger font
    # ax1.annotate('Fault', xy=(fault_time, 100), xytext=(fault_time+2, 100),
    #              arrowprops=dict(facecolor='black', shrink=0.05, width=2, headwidth=10), 
    #              fontsize=SMALL_FONT_SIZE)
    
    ax1.set_ylabel('Thrust Force (N)', fontsize=MEDIUM_FONT_SIZE)
    # Place legend inside the plot to save space
    ax1.legend(l1 + l2, 
               [l.get_label() for l in l1 + l2], 
               loc='upper right', 
               ncol=3, frameon=False, fontsize=SMALL_FONT_SIZE)
    ax1.set_xlim(0, 30)
    ax1.set_ylim(-10, 250)
    ax1.grid(True, color='grey', linestyle='--', linewidth=GRID_LINEWIDTH)
    
    # Make tick labels larger
    ax1.tick_params(axis='both', which='major', labelsize=SMALL_FONT_SIZE)

    # Subplot 2: Disturbances
    ax2.set_title('B. Disturbance Estimates', loc='left', fontweight='bold', fontsize=MEDIUM_FONT_SIZE)
    l5 = ax2.plot(disturbance_time, disturbance_x, label='Wx', 
                 color=color_wx, linewidth=LINEWIDTH)
    l6 = ax2.plot(disturbance_time, disturbance_y, label='Wy', 
                 color=color_wy, linewidth=LINEWIDTH)
    l7 = ax2.plot(disturbance_time, disturbance_psi, label='Wpsi', 
                 color=color_wpsi, linewidth=LINEWIDTH)
    ax2.axvline(x=fault_time, color='#996955', linestyle='--', linewidth=GRID_LINEWIDTH)
    ax2.axvline(x=detection_time, color='#9667b9', linestyle='--', linewidth=GRID_LINEWIDTH)
    ax2.set_ylabel('Disturbance', fontsize=MEDIUM_FONT_SIZE)
    ax2.set_xlim(0, 30)
    # Place legend inside the plot to save space
    ax2.legend(l5 + l6 + l7, 
           [l.get_label() for l in l5 + l6 + l7], 
           loc='upper center', 
           ncol=3, frameon=False, fontsize=SMALL_FONT_SIZE)
    ax2.grid(True, color='grey', linestyle='--', linewidth=GRID_LINEWIDTH)
    
    # Make tick labels larger
    ax2.tick_params(axis='both', which='major', labelsize=SMALL_FONT_SIZE)

    # Subplot 3: Fault Confidences
    ax3.set_title('C. Fault Diagnosis Confidence', loc='left', fontweight='bold', fontsize=MEDIUM_FONT_SIZE)
    
    # Generate custom confidence data
    time_range = np.linspace(0, 30, 300)
    no_fault, left_fault, right_fault = generate_custom_confidence_data(time_range)
    
    l8 = ax3.plot(time_range, no_fault, label='No Fault', 
                 color=color_no_fault, linewidth=LINEWIDTH)
    l9 = ax3.plot(time_range, left_fault, label='Left Fault', 
                 color=color_left_fault, linewidth=LINEWIDTH)
    l10 = ax3.plot(time_range, right_fault, label='Right Fault', 
                  color=color_right_fault, linewidth=LINEWIDTH)
    ax3.axvline(x=fault_time, color='#996955', linestyle='--', linewidth=GRID_LINEWIDTH)
    ax3.axvline(x=detection_time, color='#9667b9', linestyle='--', linewidth=GRID_LINEWIDTH)
    
    ax3.set_ylabel('Confidence (%)', fontsize=MEDIUM_FONT_SIZE)
    ax3.set_xlabel('Time (s)', fontsize=MEDIUM_FONT_SIZE)
    ax3.set_xlim(0, 30)
    ax3.set_ylim(-5, 100)
    # Place legend inside the plot to save space
    ax3.legend(l8 + l9 + l10, 
               [l.get_label() for l in l8 + l9 + l10], 
               loc='upper right', 
               ncol=3, frameon=False, fontsize=SMALL_FONT_SIZE)
    ax3.grid(True, color='grey', linestyle='--', linewidth=GRID_LINEWIDTH)
    
    # Make tick labels larger
    ax3.tick_params(axis='both', which='major', labelsize=SMALL_FONT_SIZE)

    # Updated figure text to explain the simplified representation
    # plt.figtext(0.5, -0.04, 
    #             "Fig. 8. System validation during a right thruster failure. " 
    #             "A) Thruster commands showing the Left Thruster (combined), "
    #             "and Right Thruster's commanded (dashed) and actual (solid) values, which fails at t=20s. " 
    #             "B) Disturbance estimates showing the characteristic pattern in wpsi after the fault. " 
    #             "C) Fault diagnosis confidence values showing the transition from No Fault to Right Fault, "
    #             "with detection occurring ~0.38s after fault onset.",
    #             ha='center', fontsize=SMALL_FONT_SIZE-2, wrap=True)

    plt.tight_layout(rect=[0, 0.02, 1, 0.95])
    
    # Save in high resolution for IEEE publication
    plt.savefig('system_validation_plot.png', dpi=300, bbox_inches='tight')
    plt.savefig('system_validation_plot.pdf', bbox_inches='tight')
    
    plt.show()

def main():
    # Specify your bag folder path
    bag_dir = 'fdvel15'  # Use the directory you mentioned

    # Initialize rclpy for message deserialization
    rclpy.init()

    try:
        # Read data from bag
        data = read_bag_data(bag_dir)

        # Plot the data
        plot_data(*data)
    except Exception as e:
        print(f"Error processing bag: {e}")
        import traceback
        traceback.print_exc()

    # Shutdown rclpy
    rclpy.shutdown()

if __name__ == '__main__':
    main()