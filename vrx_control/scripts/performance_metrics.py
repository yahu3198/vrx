#!/usr/bin/env python3

import os
import sys
import numpy as np
import pandas as pd
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation
import argparse
from pathlib import Path
from collections import defaultdict
import re

class BatchMetricsExtractor:
    def __init__(self, folder_path):
        self.folder_path = folder_path
        self.fault_time = 15.0
        self.results = defaultdict(list)
        
    def categorize_bag(self, bag_name):
        """Categorize bag by fault type and condition"""
        category = {}
        
        # Extract method type
        if 'baseline' in bag_name:
            category['method'] = 'baseline'
        elif 'env' in bag_name:
            category['method'] = 'eampc'
        else:
            category['method'] = 'unknown'
        
        # Extract fault side and severity
        if 'left' in bag_name:
            category['side'] = 'L'
        elif 'right' in bag_name:
            category['side'] = 'R'
        else:
            category['side'] = 'unknown'
        
        # Extract degradation percentage
        match = re.search(r'0[._](\d+)', bag_name)
        if match:
            severity = int(match.group(1))
            category['severity'] = f"{severity}%"
        else:
            category['severity'] = 'unknown'
        
        # For baseline, we don't differentiate environmental conditions
        if category['method'] == 'baseline':
            category['condition'] = 'both'
        else:
            # Determine environmental condition for EAMPC
            if 'beneficial' in bag_name or 'aligned' in bag_name:
                category['condition'] = 'beneficial'
            else:
                category['condition'] = 'nominal'
        
        return category
    
    def extract_metrics_from_bag(self, bag_path):
        """Extract metrics from a single bag"""
        storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
        converter_options = ConverterOptions('', '')
        reader = SequentialReader()
        
        try:
            reader.open(storage_options, converter_options)
        except:
            print(f"  ⚠ Could not open bag: {bag_path}")
            return None
        
        topics_and_types = reader.get_all_topics_and_types()
        type_map = {topic.name: topic.type for topic in topics_and_types}
        
        # Data storage
        data = {
            'odometry': {'time': [], 'x': [], 'y': []},
            'thrusters': {'time': [], 'left': [], 'right': []},
            'mission_metrics': {'time': [], 'duration': [], 'energy': [], 'completed': []},
            'operational_mode': {'time': [], 'mode': []},
            'environmental_assistance': {'time': [], 'surge': [], 'sway': [], 'yaw': []}
        }
        
        start_time = None
        
        while reader.has_next():
            (topic, msg_data, timestamp) = reader.read_next()
            time_sec = timestamp * 1e-9
            
            if start_time is None:
                start_time = time_sec
            
            relative_time = time_sec - start_time
            
            try:
                msg_type = get_message(type_map[topic])
                msg = deserialize_message(msg_data, msg_type)
                
                if topic == '/wamv/sensors/position/ground_truth_odometry':
                    data['odometry']['time'].append(relative_time)
                    data['odometry']['x'].append(msg.pose.pose.position.x)
                    data['odometry']['y'].append(msg.pose.pose.position.y)
                
                elif topic == '/wamv/thrusters/left/thrust':
                    data['thrusters']['time'].append(relative_time)
                    data['thrusters']['left'].append(msg.data)
                
                elif topic == '/wamv/thrusters/right/thrust':
                    if len(data['thrusters']['time']) > 0 and \
                       len(data['thrusters']['right']) < len(data['thrusters']['left']):
                        data['thrusters']['right'].append(msg.data)
                
                elif topic == '/wamv/mission_metrics':
                    data['mission_metrics']['time'].append(relative_time)
                    if len(msg.data) >= 5:
                        data['mission_metrics']['duration'].append(msg.data[0])
                        data['mission_metrics']['energy'].append(msg.data[1])
                        data['mission_metrics']['completed'].append(msg.data[4])
                
                elif topic == '/wamv/operational_mode':
                    data['operational_mode']['time'].append(relative_time)
                    data['operational_mode']['mode'].append(msg.data)
                
                elif topic == '/wamv/environmental_assistance':
                    data['environmental_assistance']['time'].append(relative_time)
                    if len(msg.data) >= 3:
                        data['environmental_assistance']['surge'].append(msg.data[0])
                        data['environmental_assistance']['sway'].append(msg.data[1])
                        data['environmental_assistance']['yaw'].append(msg.data[2])
                        
            except Exception as e:
                continue
        
        # Calculate metrics
        metrics = self.calculate_metrics(data)
        return metrics
    
    def calculate_metrics(self, data):
        """Calculate metrics from extracted data"""
        metrics = {}
        
        # Mission completion time
        mission_complete_time = None
        
        if data['mission_metrics']['completed']:
            for i, completed in enumerate(data['mission_metrics']['completed']):
                if completed > 0.5 and data['mission_metrics']['time'][i] > self.fault_time:
                    mission_complete_time = data['mission_metrics']['time'][i]
                    break
        
        if mission_complete_time is None and data['operational_mode']['mode']:
            for i, mode in enumerate(data['operational_mode']['mode']):
                if 'STATION_KEEPING' in mode and data['operational_mode']['time'][i] > self.fault_time:
                    mission_complete_time = data['operational_mode']['time'][i]
                    break
        
        if mission_complete_time is None and data['odometry']['x']:
            for i, x in enumerate(data['odometry']['x']):
                if x < -570 and data['odometry']['time'][i] > self.fault_time:
                    mission_complete_time = data['odometry']['time'][i]
                    break
        
        if mission_complete_time:
            metrics['recovery_time'] = mission_complete_time - self.fault_time
            metrics['success'] = True
        else:
            metrics['recovery_time'] = None
            metrics['success'] = False
        
        # Energy consumption
        if data['mission_metrics']['energy'] and mission_complete_time:
            for i, t in enumerate(data['mission_metrics']['time']):
                if t >= mission_complete_time:
                    metrics['energy_kj'] = data['mission_metrics']['energy'][i] / 1000.0
                    break
        
        # Path length
        if data['odometry']['x']:
            path_length = 0
            times = data['odometry']['time']
            x_vals = data['odometry']['x']
            y_vals = data['odometry']['y']
            
            # Find starting index at fault time
            fault_idx = 0
            for i, t in enumerate(times):
                if t >= self.fault_time:
                    fault_idx = i
                    break
            
            # Calculate path length from fault to mission complete (or end)
            end_idx = len(times) - 1
            if mission_complete_time:
                for i, t in enumerate(times):
                    if t >= mission_complete_time:
                        end_idx = i
                        break
            
            for i in range(fault_idx + 1, end_idx + 1):
                dx = x_vals[i] - x_vals[i-1]
                dy = y_vals[i] - y_vals[i-1]
                path_length += np.sqrt(dx**2 + dy**2)
            
            metrics['path_length'] = path_length
        
        # Environmental utilization (only for EAMPC)
        if data['environmental_assistance']['surge']:
            env_vals = []
            for i, t in enumerate(data['environmental_assistance']['time']):
                if t > self.fault_time and (not mission_complete_time or t <= mission_complete_time):
                    avg_val = (data['environmental_assistance']['surge'][i] + 
                              data['environmental_assistance']['sway'][i] + 
                              data['environmental_assistance']['yaw'][i]) / 3.0
                    env_vals.append(avg_val)
            
            if env_vals:
                metrics['env_use'] = np.mean(env_vals) * 100
        
        return metrics
    
    def process_folder(self):
        """Process all bags in folder and compute statistics"""
        bag_folders = [d for d in os.listdir(self.folder_path) 
                      if os.path.isdir(os.path.join(self.folder_path, d))]
        
        print(f"\nFound {len(bag_folders)} bags in {self.folder_path}\n")
        
        # Separate storage for EAMPC and baseline results
        eampc_results = defaultdict(lambda: {
            'success_count': 0,
            'total_count': 0,
            'recovery_times': [],
            'energies': [],
            'path_lengths': [],
            'env_uses': []
        })
        
        baseline_results = defaultdict(lambda: {
            'success_count': 0,
            'total_count': 0,
            'recovery_times': [],
            'energies': [],
            'path_lengths': []
        })
        
        for bag_folder in bag_folders:
            bag_path = os.path.join(self.folder_path, bag_folder)
            print(f"Processing: {bag_folder}")
            
            # Categorize the bag
            category = self.categorize_bag(bag_folder)
            
            # Extract metrics
            metrics = self.extract_metrics_from_bag(bag_path)
            
            if metrics:
                if category['method'] == 'baseline':
                    # Process baseline MPC results
                    key = f"L/R {category['severity']}"
                    baseline_results[key]['total_count'] += 1
                    if metrics['success']:
                        baseline_results[key]['success_count'] += 1
                        if metrics.get('recovery_time'):
                            baseline_results[key]['recovery_times'].append(metrics['recovery_time'])
                        if metrics.get('energy_kj'):
                            baseline_results[key]['energies'].append(metrics['energy_kj'])
                        if metrics.get('path_length'):
                            baseline_results[key]['path_lengths'].append(metrics['path_length'])
                
                elif category['method'] == 'eampc':
                    # Process EAMPC results
                    key = f"{category['side']}{category['severity']}_{category['condition']}"
                    eampc_results[key]['total_count'] += 1
                    if metrics['success']:
                        eampc_results[key]['success_count'] += 1
                        if metrics.get('recovery_time'):
                            eampc_results[key]['recovery_times'].append(metrics['recovery_time'])
                        if metrics.get('energy_kj'):
                            eampc_results[key]['energies'].append(metrics['energy_kj'])
                        if metrics.get('path_length'):
                            eampc_results[key]['path_lengths'].append(metrics['path_length'])
                        if metrics.get('env_use'):
                            eampc_results[key]['env_uses'].append(metrics['env_use'])
        
        # Print both tables
        self.print_eampc_table(eampc_results)
        self.print_baseline_table(baseline_results)
    
    def print_eampc_table(self, results):
        """Print EAMPC results in LaTeX table format"""
        print("\n" + "="*80)
        print("EAMPC (PROPOSED) RESULTS")
        print("="*80)
        
        sorted_keys = sorted(results.keys())
        
        print("\nLaTeX Table Rows:")
        print("-"*80)
        
        for key in sorted_keys:
            data = results[key]
            parts = key.split('_')
            fault_type = parts[0] if len(parts) > 0 else 'Unknown'
            condition = parts[1] if len(parts) > 1 else 'unknown'
            
            success_rate = f"{data['success_count']}/{data['total_count']}"
            
            time_str = f"{np.mean(data['recovery_times']):.0f}±{np.std(data['recovery_times']):.0f}" if data['recovery_times'] else "--"
            energy_str = f"{np.mean(data['energies']):.0f}±{np.std(data['energies']):.0f}" if data['energies'] else "--"
            path_str = f"{np.mean(data['path_lengths']):.0f}" if data['path_lengths'] else "--"
            
            condition_str = "Beneficial" if condition == "beneficial" else "Nominal"
            print(f"{fault_type} & {condition_str} & {success_rate} & {time_str} & {energy_str} & {path_str} \\\\")
    
    def print_baseline_table(self, results):
        """Print Baseline MPC results in LaTeX table format"""
        print("\n" + "="*80)
        print("BASELINE MPC RESULTS")
        print("="*80)
        
        sorted_keys = sorted(results.keys())
        
        print("\nLaTeX Table Rows:")
        print("-"*80)
        
        for key in sorted_keys:
            data = results[key]
            
            success_rate = f"{data['success_count']}/{data['total_count']}"
            
            if data['recovery_times']:
                time_str = f"{np.mean(data['recovery_times']):.0f}±{np.std(data['recovery_times']):.0f}"
            else:
                time_str = "Failed" if data['success_count'] == 0 else "--"
            
            energy_str = f"{np.mean(data['energies']):.0f}±{np.std(data['energies']):.0f}" if data['energies'] else "--"
            path_str = f"{np.mean(data['path_lengths']):.0f}" if data['path_lengths'] else "--"
            
            print(f"{key} & Both & {success_rate} & {time_str} & {energy_str} & {path_str} \\\\")

def main():
    parser = argparse.ArgumentParser(description='Process all ROS2 bags in a folder')
    parser.add_argument('--folder', default='/home/yang/usv_ws/experiments/bags/statR95b',
                       help='Path to folder containing ROS2 bags')
    args = parser.parse_args()
    
    if not os.path.exists(args.folder):
        print(f"Error: Folder does not exist: {args.folder}")
        sys.exit(1)
    
    extractor = BatchMetricsExtractor(args.folder)
    extractor.process_folder()

if __name__ == "__main__":
    main()