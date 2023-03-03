#
# Copyright 2020-2022 Picovoice Inc.
#
# You may not use this file except in compliance with the license. A copy of the license is located in the "LICENSE"
# file accompanying this source.
#
# Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on
# an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the
# specific language governing permissions and limitations under the License.
#

"""
Resets turtlesim and then allows user to load an arbritary amount of waypoints. The waypoints are then plotted using the turtle and a control algorithm is 
initiated to pubslish the twist and move the turtle through all the waypoints and back to the start.


PUBLISHERS:
    + turtle1/cmd_vel (Twist) - Linear and angular velocity to move the turtle forward and change its heading towards the waypoint.

SUBSCRIBERS:
    + turtle1/pose (Pose) - The position, orientation and velocity of the current turtle. This is used in the control algorithm to move the turtle through the waypoints list.

SERVICES:
    + toggle (Empty) - Changes the state of the system from ANY state to STOPPED or from STOPPED to MOVING
    + load (Waypoints) - Load an arbritary amount of waypoints

PARAMETERS:
    + frequency (double) - Sets the frequency of the timer, thus controlling the timer_callback function speed.
    + tolerance (double) - This is the maximum relative distance error that the turtle must have to verify that a waypoint was reached and thus the turtle can move to the next waypoint. 


"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor
import sys
import struct
from threading import Thread
from picovoice import *
from pvrecorder import PvRecorder
import ament_index_python

class PicovoiceDemo(Thread):
    def __init__(
            self,
            access_key,
            audio_device_index,
            keyword_path,
            context_path,
            porcupine_library_path=None,
            porcupine_model_path=None,
            porcupine_sensitivity=0.5,
            rhino_library_path=None,
            rhino_model_path=None,
            rhino_sensitivity=0.5,
            endpoint_duration_sec=1.,
            require_endpoint=True,
            output_path=None):
        super(PicovoiceDemo, self).__init__()


class Listen(Node):
    """
    """

    def __init__(self):
        super().__init__("listen")

        # self.state = State.RESET # Starts with RESET state
        self.flag_state_stopped = 0 # This is used to log "STOPPING" only once to debug.

        self.declare_parameter("frequency", 100.0, ParameterDescriptor(description="Timer callback frequency"))
        self.declare_parameter("keyword_path", ament_index_python.get_package_share_directory(
            "listen_talk_ros2")+ "/Hey-Willie_en_linux_v2_1_0.ppn")
        self.declare_parameter("context_path", ament_index_python.get_package_share_directory(
            "listen_talk_ros2")+ "/Dog-command_en_linux_v2_1_0.rhn")
        self.declare_parameter("access_key", "bz3cScyGLZpi/dcR5/xHDJJ/pCBdswpMGXHL2Djgik7Rn04q54tdYA==", ParameterDescriptor(description="The error tolerance for the waypoint"))


        self.frequency = self.get_parameter("frequency").get_parameter_value().double_value
        self.keyword_path = self.get_parameter("keyword_path").get_parameter_value().string_value
        self.context_path = self.get_parameter("context_path").get_parameter_value().string_value
        self.access_key = self.get_parameter("access_key").get_parameter_value().string_value
        self.voice_command = String()


        # Publishers, Subscribers, Services and Timer
        self.pub_voice_command = self.create_publisher(String, "/voice_command", 10)
        self.timer = self.create_timer(1.0/self.frequency, self.timer_callback)


    def _wake_word_callback(self):
        print('[wake word]\n')

    def _inference_callback(self,inference):
        if inference.is_understood:
            print('{')
            print("  intent : '%s'" % inference.intent)
            print('  slots : {')
            for slot, value in inference.slots.items():
                print("    %s : '%s'" % (slot, value))
                self.voice_command.data = value # Set voice command
            print('  }')
            print('}\n')
            
            # Publish voice command to a topic
            self.pub_voice_command.publish(self.voice_command)

        else:
            print("I didn't understand the command.\n")

    def run(self):
        recorder = None
        wav_file = None

        try:
            recorder = PvRecorder(device_index=-1, frame_length=self._picovoice.frame_length)
            recorder.start()

            # if self.output_path is not None:
            #     wav_file = wave.open(self.output_path, "w")
            #     # noinspection PyTypeChecker
            #     wav_file.setparams((1, 2, 16000, 512, "NONE", "NONE"))

            print("Using device: %s" % recorder.selected_device)
            print('[Listening ...]')

            while True:
                pcm = recorder.read()

                if wav_file is not None:
                    wav_file.writeframes(struct.pack("h" * len(pcm), *pcm))

                self._picovoice.process(pcm)
        except KeyboardInterrupt:
            sys.stdout.write('\b' * 2)
            print('Stopping ...')
        finally:
            if recorder is not None:
                recorder.delete()

            if wav_file is not None:
                wav_file.close()

            self._picovoice.delete()



    def show_audio_devices(self, cls):
        devices = PvRecorder.get_audio_devices()

        for i in range(len(devices)):
            print('index: %d, device name: %s' % (i, devices[i]))


    def timer_callback(self):
        """ 
        Timer Callback
        """

        # print(f"{self.keyword_path}")

        try:
            self._picovoice = Picovoice(
                access_key=self.access_key,
                keyword_path=self.keyword_path,
                wake_word_callback=self._wake_word_callback,
                context_path=self.context_path,
                inference_callback=self._inference_callback,
                porcupine_library_path=None,
                porcupine_model_path=None,
                porcupine_sensitivity=0.5,
                rhino_library_path=None,
                rhino_model_path=None,
                rhino_sensitivity=0.5,
                endpoint_duration_sec=1.0,
                require_endpoint=True)
            
            self.run()
            
            PicovoiceDemo(
                access_key=self.access_key,
                audio_device_index=-1,
                keyword_path=self.keyword_path,
                context_path=self.context_path,
                porcupine_library_path=None,
                porcupine_model_path=None,
                porcupine_sensitivity=0.5,
                rhino_library_path=None,
                rhino_model_path=None,
                rhino_sensitivity=0.5,
                endpoint_duration_sec=1.0,
                require_endpoint=True,
                output_path=None).run()



        except PicovoiceInvalidArgumentError as e:
            print("One or more arguments provided to Picovoice is invalid: ")
            print("If all other arguments seem valid, ensure that is a valid AccessKey")
            raise e
        except PicovoiceActivationError as e:
            print("AccessKey activation error")
            raise e
        except PicovoiceActivationLimitError as e:
            print("AccessKey has reached it's temporary device limit")
            raise e
        except PicovoiceActivationRefusedError as e:
            print("AccessKey refused")
            raise e
        except PicovoiceActivationThrottledError as e:
            print("AccessKey has been throttled")
            raise e
        except PicovoiceError as e:
            print("Failed to initialize Picovoice")
            raise e

        # self.audio_device_index = audio_device_index
        self.output_path = None


def main(args=None):
    """ The main() function. """

    rclpy.init(args=args)
    listen = Listen()
    listen.timer_callback()
    rclpy.spin(listen)
    rclpy.shutdown()


if __name__ == '__main__':
    main()