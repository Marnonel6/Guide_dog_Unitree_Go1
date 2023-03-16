# MIT License
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
    + talk (string) - Audio file of a person saying 'Woof'
    + allan (string) - Audio file of the soundtrack allan.
    + easter_egg (string) - Easter egg audio file.
    + not_understanding (string) - Audio file for when the voice recognition does not understand.
    + willie_greet (string) - Audio file for greeting upon wake up.

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
        self.declare_parameter("talk", ament_index_python.get_package_share_directory(
            "listen_talk_ros2") + "/talk.mp3")
        self.declare_parameter("allan", ament_index_python.get_package_share_directory(
            "listen_talk_ros2") + "/allan.mp3")
        self.declare_parameter("easter_egg", ament_index_python.get_package_share_directory(
            "listen_talk_ros2") + "/easter_egg.mp3")
        self.declare_parameter("not_understanding", ament_index_python.get_package_share_directory(
            "listen_talk_ros2") + "/not_understanding.mp3")
        self.declare_parameter("willie_greet", ament_index_python.get_package_share_directory(
            "listen_talk_ros2") + "/willie_greet.mp3")

        self.bark = self.get_parameter("bark").get_parameter_value().string_value
        self.talk = self.get_parameter("talk").get_parameter_value().string_value
        self.allan = self.get_parameter("allan").get_parameter_value().string_value
        self.easter_egg = self.get_parameter("easter_egg").get_parameter_value().string_value
        self.not_understanding = self.get_parameter("not_understanding").get_parameter_value().string_value
        self.willie_greet = self.get_parameter("willie_greet").get_parameter_value().string_value

        self.bark_Flag = 0

        # Subscribers
        self.sub = self.create_subscription(String, "/voice_command", self.voice_command, 10)

    def voice_command(self,data):
        """
        Subscribtion topic: /voice_command
        """

        if data.data == "bark" and self.bark_Flag == 0:
            bark = 'mpg123' + ' ' + self.bark
            self.bark_Flag = 1
            os.system(bark)
        elif data.data == "bark" and self.bark_Flag == 1:
            talk = 'mpg123' + ' ' + self.talk
            self.bark_Flag = 0
            os.system(talk)

        if data.data == "allan":
            allan = 'mpg123' + ' ' + self.allan
            os.system(allan)

        if data.data == "easter egg":
            easter_egg = 'mpg123' + ' ' + self.easter_egg
            os.system(easter_egg)

        if data.data == "not understanding":
            not_understanding = 'mpg123' + ' ' + self.not_understanding
            os.system(not_understanding)

        if data.data == "willie greet":
            willie_greet = 'mpg123' + ' ' + self.willie_greet
            os.system(willie_greet)


def main(args=None):
    """ The main() function. """
    rclpy.init(args=args)
    talk = Talk()
    rclpy.spin(talk)
    rclpy.shutdown()


if __name__ == '__main__':
    main()



