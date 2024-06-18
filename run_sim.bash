#!/bin/bash

read -p "Please enter number of drones to be spawned: " user_input
user_input=${user_input:-10}
module=4
namespace="iris_"
counter=0
pose_x=0
process_list=""


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

# Check if the input is a valid integer
if [[ $user_input =~ ^[0-9]+$ ]]; then
    echo "You entered: $user_input"
else
    echo "Invalid input. Please enter a valid integer."
    exit 1
fi




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


# Trap SIGINT and call the cleanup function
trap cleanup SIGINT

# Wait for Ctrl+C to be pressed
echo "Waiting for Ctrl+C..."
wait $process_list
echo "All process are killed successfully"