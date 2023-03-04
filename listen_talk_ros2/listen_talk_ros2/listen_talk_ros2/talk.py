"""
Loads prewritten .mp3 files and play's it back over speakers.

PUBLISHERS:
    + None

SUBSCRIBERS:
    + /voice_command (String) - The voice command to preform as a string.

SERVICES:
    + None

PARAMETERS:
    + bark (string) - Audio file of a German sheperd barking.

"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import ament_index_python


class Talk(Node):
    """This node preforms audio output by loading existing .mp3 files and playing the file."""

    def __init__(self):
        super().__init__("talk")

        # Load .mp3 files from the shared directory as parameters
        self.declare_parameter("bark", ament_index_python.get_package_share_directory(
            "listen_talk_ros2") + "/German_shepherd_barking.mp3")
        self.declare_parameter("allan", ament_index_python.get_package_share_directory(
            "listen_talk_ros2") + "/allan.mp3")
        self.declare_parameter("easter_egg", ament_index_python.get_package_share_directory(
            "listen_talk_ros2") + "/easter_egg.mp3")
        self.declare_parameter("not_understanding", ament_index_python.get_package_share_directory(
            "listen_talk_ros2") + "/not_understanding.mp3")

        self.bark = self.get_parameter("bark").get_parameter_value().string_value
        self.allan = self.get_parameter("allan").get_parameter_value().string_value
        self.easter_egg = self.get_parameter("easter_egg").get_parameter_value().string_value
        self.not_understanding = self.get_parameter("not_understanding").get_parameter_value().string_value


        # Publishers, Subscribers, Services and Timer
        self.sub = self.create_subscription(String, "/voice_command", self.voice_command, 10)

    def voice_command(self,data):
        """
        Subscribtion topic: /voice_command
        """

        if data.data == "bark":
            bark = 'mpg123' + ' ' + self.bark
            os.system(bark)

        if data.data == "allan":
            allan = 'mpg123' + ' ' + self.allan
            os.system(allan)

        if data.data == "easter egg":
            easter_egg = 'mpg123' + ' ' + self.easter_egg
            os.system(easter_egg)

        if data.data == "not understanding":
            not_understanding = 'mpg123' + ' ' + self.not_understanding
            os.system(not_understanding)

def main(args=None):
    """ The main() function. """
    rclpy.init(args=args)
    talk = Talk()
    rclpy.spin(talk)
    rclpy.shutdown()


if __name__ == '__main__':
    main()



