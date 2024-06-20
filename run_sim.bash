#!/bin/bash

# Function to handle SIGINT (Ctrl+C)
cleanup() {
    echo "Ctrl+C detected. Cleaning up..."
    # Loop through the process list and kill each process
    for pid in $process_list; do
        echo "Killing process $pid"
        kill $pid
    done
    exit 1
}

# Prompt for the number of drones to be spawned
read -p "Please enter number of drones to be spawned [default: 2]: " user_input
user_input=${user_input:-2}
read -p "Please enter the module number (1-3) [default: 1]: " module 
module=${module:-1}

# Check if the module is within the valid range
if [[ $module =~ ^[1-3]$ ]]; then
    echo "You selected module: $module"
else
    echo "Invalid module. Please enter a number between 1 and 3."
    exit 1
fi

namespace="iris_"
counter=0
pose_x=0
process_list=""

# Trap SIGINT and call the cleanup function
trap cleanup SIGINT

roslaunch sitl_tutorials spawn_gazebo_world.launch & 
sleep 5

while [ $counter -le $user_input ]
do 
    for ((i=0; i<4; i++))
    do 
        # Check if the counter is within the specified range
        if [ $counter -lt $user_input ]; then
            roslaunch --wait sitl_tutorials spawn_sitl.launch ID:=$((counter+=100)) pose_x:=$((pose_x * 5)) pose_y:=$((i * 5)) uav_name:="${namespace}${counter}" module:=$module &
            process_list+=" $!"
            sleep 5
        fi
        ((counter++))
    done 
    ((pose_x++))
done

# Wait for Ctrl+C to be pressed
echo "Waiting for Ctrl+C..."
wait $process_list
echo "All processes are killed successfully"
