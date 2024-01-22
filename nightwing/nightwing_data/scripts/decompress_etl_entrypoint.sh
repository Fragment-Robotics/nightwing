#!/bin/bash

EXTRACT_TO=$1

# Get arguments 
if [ -z "$EXTRACT_TO" ]; then
    echo "No extract_to directory argument provided, using default /data/extracted_images"
    EXTRACT_TO="/data/extracted_images"
fi

# Pass the extract_to argument to the launch file
ros2 launch nightwing_data decompress_etl.launch.py extract_to:=$EXTRACT_TO &
LAUNCH_PID=$!
echo "[ DECOMPRESSOR ] Launch PID: ${LAUNCH_PID}"

# Launch the bag playback
ros2 bag play -s mcap -r 5.0 -d 5 /data/rosbags/*.mcap &
BAG_PLAY_PID=$!
echo "[ DECOMPRESSOR ] Bag player PID: ${BAG_PLAY_PID}"

while kill -0 $BAG_PLAY_PID 2> /dev/null; do
    sleep 1
done

# Send SIGTERM (Ctrl+C) to the ros2 launch process
echo "[ DECOMPRESSOR ] Sending SIGTERM to launch process!"
kill -SIGTERM $LAUNCH_PID

# Give some time for the process to terminate
sleep 5

# Check if the launch process has exited and force kill if not
if kill -0 $LAUNCH_PID 2> /dev/null; then
    echo "[ DECOMPRESSOR ] Launch process still running, sending SIGKILL."
    kill -9 $LAUNCH_PID
fi

echo "[ DECOMPRESSOR ] ROS2 launch process has been terminated."

# Exit the script
exit 0
