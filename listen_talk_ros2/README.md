# listen_talk_ros2
A ROS2 packages for speech recognition and text to speech audio.

# Command line
`ros2 launch listen_talk_ros2 listen_talk.launch.py`

Ensure that the `.ppn` and `.rhn` files are correct for your system. Add them to the config directory and in the `setup.py`

# BEFORE DOING ANYTHING RUN blow command to generate audio files while connected to the internet after that no internet needed
# in the 'listen_talk_ros2/listen_talk_ros2/' folder
`python3 generate_audio_files.py`