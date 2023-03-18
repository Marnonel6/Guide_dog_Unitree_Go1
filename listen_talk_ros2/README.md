# listen_talk_ros2
A ROS2 packages for speech recognition and text to speech audio.

On-device voice assistant platform powered by deep learning 

# Command line
`ros2 launch listen_talk_ros2 listen_talk.launch.py` - Default uses jetson nano

Ensure that the `.ppn` and `.rhn` files are correct for your system. Add them to the config directory and in the `setup.py`

# First run below command to generate audio files while connected to the internet after that no internet needed
# in the 'listen_talk_ros2/listen_talk_ros2/' folder
`python3 generate_audio_files.py`

# To run on Linux computer
`ros2 run listen_talk_ros2 listen --ros-args -p use_jetson_nano:=False`
# To run on Jetson Nano
`ros2 run listen_talk_ros2 listen --ros-args -p use_jetson_nano:=True` or `ros2 run listen_talk_ros2 listen` as it defaults to true.

# Dependencies
    sudo pip3 install picovoice
    sudo pip3 install gTTS

# Demo video

https://user-images.githubusercontent.com/60977336/226095385-699c7d70-c843-4602-816c-8e003dd0e82b.mp4
