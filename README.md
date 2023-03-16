# Guide_dog_Unitree_Go1
A Unitree Go1 robot dog is currently being programmed to be a guide dog for the visually impaired.


`ros2 launch guide_dog_unitree_go1 guide_dog.launch.py use_nav2:=<A> use_voice_control:=<B> use_speech_recognition:=<C>`


# REALSENS ON DOG
ros2 launch guide_dog_unitree_go1 guide_dog.launch.py use_nav2:=false use_voice_control:=true use_speech_recognition:=true use_object_detection:=false use_Go1_vision:=true

# Vision

# Voice


# On computer
ros2 launch guide_dog_unitree_go1 guide_dog.launch.py use_nav2:=true use_voice_control:=false use_speech_recognition:=false use_object_detection:=true use_Go1_vision:=false

# Vision

# Voice