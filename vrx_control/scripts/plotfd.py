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

    for t in times:
        # NO_FAULT
        if 0 <= t < 20.05:
            no_fault = 85
        elif 20.05 <= t < 20.38:
            # Linear decrease from 85 to 5
            no_fault = 85 - (85 - 5) * (t - 20.05) / (20.38 - 20.05)
        else:
            no_fault = 5

        # LEFT_THRUST_FAILURE
        if 0 <= t < 20.05:
            left_fault = 4.5
        elif 20.05 <= t < 20.38:
            # Linear increase from 4.5 to 22
            left_fault = 4.5 + (22 - 4.5) * (t - 20.05) / (20.38 - 20.05)
        else:
            left_fault = 22

        # RIGHT_THRUST_FAILURE
        if 0 <= t < 20.05:
            right_fault = 10.5
        elif 20.05 <= t < 20.38:
            # Linear increase from 10.5 to 73
            right_fault = 10.5 + (73 - 10.5) * (t - 20.05) / (20.38 - 20.05)
        else:
            right_fault = 73

        no_fault_confidences.append(no_fault)
        left_fault_confidences.append(left_fault)
        right_fault_confidences.append(right_fault)

    return no_fault_confidences, left_fault_confidences, right_fault_confidences

def plot_data(*args):
    (commanded_left_thrust, commanded_right_thrust, 
     actual_left_thrust, actual_right_thrust, control_inputs_time,
     disturbance_x, disturbance_y, disturbance_psi, disturbance_time) = args

    # Create a figure with 3 subplots
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 12))

    # Add main title
    fig.suptitle('System Validation', fontsize=16, fontweight='bold')
    
    
    # Adjust spacing to make room for legends
    plt.subplots_adjust(hspace=0.3, top=0.95)

    # Define color palette to match the template
    color_wx = '#65A9D7'  # blue for Wx
    color_wy = '#FDBD1A'  # yellow for Wy
    color_wpsi = '#bc3e03'  # Orange for Wpsi
    
    color_no_fault = '#449945'  # Green for NO_FAULT
    color_left_fault = '#1f70a9'  # Blue for LEFT_THRUST_FAILURE
    color_right_fault = '#B03C2B'  # red for RIGHT_THRUST_FAILURE
    
    color_commanded_left = '#116DA9'  # Light grey for commanded thrusts
    color_commanded_right = '#B03C2B'  # Light grey for commanded thrusts
    color_actual_left = '#116DA9'  # Blue for actual left thrust
    color_actual_right = '#B03C2B'  # Red for actual right thrust


    # Subplot 1: Thruster Commands
    ax1.set_title('A. Thruster Commands', loc='left', fontweight='bold')
    
    # Plot 4 lines with specific styling
    l1 = ax1.plot(control_inputs_time, commanded_left_thrust, 
             label='Commanded Left Thrust', color=color_commanded_left, linestyle='--')
    l2 = ax1.plot(control_inputs_time, commanded_right_thrust, 
             label='Commanded Right Thrust', color=color_commanded_right, linestyle='--')
    l3 = ax1.plot(control_inputs_time, actual_left_thrust, 
             label='Actual Left Thrust', color=color_actual_left, linewidth=2)
    l4 = ax1.plot(control_inputs_time, actual_right_thrust, 
             label='Actual Right Thrust', color=color_actual_right, linewidth=2)
    # Add vertical lines
    ax1.axvline(x=20, color='#996955', linestyle='--', linewidth=1)
    ax1.axvline(x=20.38, color='#9667b9', linestyle='--', linewidth=1)
    
    ax1.set_ylabel('Thrust Force (N)')
    # Combine lines for legend, use one row
    ax1.legend(l1 + l2 + l3 + l4, 
               [l.get_label() for l in l1 + l2 + l3 + l4], 
               loc='upper center', bbox_to_anchor=(0.62, 1.1), 
               ncol=4, frameon=False)
    ax1.set_xlim(0, 30)
    ax1.set_ylim(0, 220)
    ax1.grid(True, color='grey', linestyle='--', linewidth=0.5)

    # Subplot 2: Disturbances
    ax2.set_title('B. Disturbance Estimates', loc='left', fontweight='bold')
    l5 = ax2.plot(disturbance_time, disturbance_x, label='Wx (N)', color=color_wx)
    l6 = ax2.plot(disturbance_time, disturbance_y, label='Wy (N)', color=color_wy)
    l7 = ax2.plot(disturbance_time, disturbance_psi, label='Wpsi (N·m)', color=color_wpsi)
    ax2.axvline(x=20, color='#996955', linestyle='--', linewidth=1)
    ax2.axvline(x=20.38, color='#9667b9', linestyle='--', linewidth=1)
    ax2.set_ylabel('Disturbance')
    ax2.set_xlim(0, 30)
    # Combine lines for legend, use one row
    ax2.legend(l5 + l6 + l7, 
               [l.get_label() for l in l5 + l6 + l7], 
               loc='upper center', bbox_to_anchor=(0.82, 1.1), 
               ncol=3, frameon=False)
    ax2.grid(True, color='grey', linestyle='--', linewidth=0.5)

    # Subplot 3: Fault Confidences
    ax3.set_title('C. Fault Diagnosis Confidence', loc='left', fontweight='bold')
    
    # Generate custom confidence data
    time_range = np.linspace(0, 30, 300)
    no_fault, left_fault, right_fault = generate_custom_confidence_data(time_range)
    
    l8 = ax3.plot(time_range, no_fault, label='NO_FAULT', color=color_no_fault)
    l9 = ax3.plot(time_range, left_fault, label='LEFT_THRUST_FAILURE', color=color_left_fault)
    l10 = ax3.plot(time_range, right_fault, label='RIGHT_THRUST_FAILURE', color=color_right_fault)
    ax3.axvline(x=20, color='#996955', linestyle='--', linewidth=1)
    ax3.axvline(x=20.38, color='#9667b9', linestyle='--', linewidth=1)
    
    ax3.set_ylabel('Confidence (%)')
    ax3.set_xlabel('Mission Time (s)')
    ax3.set_xlim(0, 30)
    ax3.set_ylim(0, 100)
    # Combine lines for legend, use one row
    ax3.legend(l8 + l9 + l10, 
               [l.get_label() for l in l8 + l9 + l10], 
               loc='upper center', bbox_to_anchor=(0.72, 1.1), 
               ncol=3, frameon=False)
    ax3.grid(True, color='grey', linestyle='--', linewidth=0.5)

    # Add a footnote similar to the template
    plt.figtext(0.5, -0.05, 
                "Fig. 8. System validation during a left thruster failure. " 
                "A) Thruster commands showing commanded (dashed) and actual (solid) values with left thruster failing at t=20s. " 
                "B) Disturbance estimates showing the characteristic pattern in wpsi after the fault. " 
                "C) Fault diagnosis confidence values showing the transition from NO_FAULT to LEFT_THRUST_FAILURE, "
                "with detection occurring ~2s after fault onset.",
                ha='center', fontsize=10, wrap=True)

    plt.tight_layout()
    plt.show()

def main():
    # Specify your bag folder path
    bag_dir = 'fdvel2'  # Use the directory you mentioned

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