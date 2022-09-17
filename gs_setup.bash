DT_ID=0
DT_PATH=$(dirname $(readlink -f "${BASH_SOURCE:-$0}"))
DT_IP=$(expr 100 + $DT_ID)
source $DT_PATH/ros_base_nodes/catkin_ws/devel/setup.bash
source $DT_PATH/ros_simulation_nodes/catkin_ws/devel/setup.bash
export GAZEBO_MODEL_PATH=$DT_PATH/ros_simulation_nodes/catkin_ws/src/simulator/models	
export ROS_HOSTNAME=192.168.100.$DT_IP