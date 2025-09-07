#!/bin/bash

# Color codes for terminal output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Configuration
EXPERIMENT_NAME=${1:-"env_mpc_nf"}
FAULT_TYPE=${2:-"left_0.95"}
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BAG_NAME="${EXPERIMENT_NAME}_${FAULT_TYPE}_${TIMESTAMP}"
BAG_DIR="$HOME/usv_ws/experiments/bags"

# Create directory if it doesn't exist
mkdir -p ${BAG_DIR}

echo -e "${GREEN}Starting ROS2 bag recording...${NC}"
echo -e "${YELLOW}Experiment: ${EXPERIMENT_NAME}${NC}"
echo -e "${YELLOW}Fault Type: ${FAULT_TYPE}${NC}"
echo -e "${YELLOW}Saving to: ${BAG_DIR}/${BAG_NAME}${NC}"

# Record all essential topics for paper figures
ros2 bag record -o ${BAG_DIR}/${BAG_NAME} \
  /wamv/sensors/position/ground_truth_odometry \
  /wamv/thrusters/left/thrust \
  /wamv/thrusters/right/thrust \
  /wamv/ref_pose \
  /wamv/error_pose \
  /wamv/control_inputs \
  /wamv/ekf_pose \
  /wamv/disturbance \
  /wamv/disturbance_world \
  /wamv/fault_diagnosis \
  /wamv/operational_mode \
  /wamv/thruster_health \
  /wamv/environmental_assistance \
  /wamv/planning_status \
  /wamv/prediction_metrics \
  /wamv/mission_metrics \
  /wamv/usv_state \
  /tf \
  /tf_static

echo -e "${GREEN}Recording stopped. Bag saved to: ${BAG_DIR}/${BAG_NAME}${NC}"