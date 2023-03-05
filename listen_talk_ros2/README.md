# listen_talk_ros2
A ROS2 packages for speech recognition and text to speech audio.

# Command line
`ros2 launch listen_talk_ros2 listen_talk.launch.py` - Default uses jetson nano

Ensure that the `.ppn` and `.rhn` files are correct for your system. Add them to the config directory and in the `setup.py`

# BEFORE DOING ANYTHING RUN blow command to generate audio files while connected to the internet after that no internet needed
# in the 'listen_talk_ros2/listen_talk_ros2/' folder
`python3 generate_audio_files.py`

# To run on Linux computer
`ros2 run listen_talk_ros2 listen --ros-args -p use_jetson_nano:=False`
# TO run on Jetson Nano
`ros2 run listen_talk_ros2 listen --ros-args -p use_jetson_nano:=True` or `ros2 run listen_talk_ros2 listen` as it defaults to true.
