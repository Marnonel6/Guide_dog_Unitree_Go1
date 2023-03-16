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
Voice recognition node. Uses picovoice to listen for a wake word (Hey Willie), then start listening for pretrained phrases to preform an action.
The desired action is then published to a voice command topic.

PUBLISHERS:
    + voice_command (String) - The desired voice command action is published to a voice command topic.

SUBSCRIBERS:
    + None

SERVICES:
    + None

PARAMETERS:
    + frequency (double) - Sets the frequency of the timer, thus controlling the timer_callback function speed.
    + use_jetson_nano (bool) - Choose to run the program on a Jetson nanon or a linux laptop.
    + access_key (string) - Picovoice API key.
    + keyword_path (string) - .ppn (Porcupine) file for the wake word from Picovoice
    + context_path (string) - .rhn (Rhino) file for the pretrained voice recognition commands from Picovoice.

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
    """Speech recognition node."""

    def __init__(self):
        super().__init__("listen")

        self.flag_state_stopped = 0 # This is used to log "STOPPING" only once to debug.

        self.declare_parameter("use_jetson_nano", True, ParameterDescriptor(description="Run program on jetson nano, otherwise is linux computer"))
        self.declare_parameter("frequency", 100.0, ParameterDescriptor(description="Timer callback frequency"))
        self.declare_parameter("access_key", "bz3cScyGLZpi/dcR5/xHDJJ/pCBdswpMGXHL2Djgik7Rn04q54tdYA==", ParameterDescriptor(description="Picovoice API key"))

        self.use_jetson_nano = self.get_parameter("use_jetson_nano").get_parameter_value().bool_value
        self.frequency = self.get_parameter("frequency").get_parameter_value().double_value
        self.access_key = self.get_parameter("access_key").get_parameter_value().string_value

        if (self.use_jetson_nano == False):
            self.declare_parameter("keyword_path", ament_index_python.get_package_share_directory(
                "listen_talk_ros2")+ "/Hey-Willie_en_linux_v2_1_0.ppn")
            self.declare_parameter("context_path", ament_index_python.get_package_share_directory(
                "listen_talk_ros2")+ "/Dog-command_en_linux_v2_1_0.rhn")
            self.keyword_path = self.get_parameter("keyword_path").get_parameter_value().string_value
            self.context_path = self.get_parameter("context_path").get_parameter_value().string_value
        else:
            self.declare_parameter("keyword_path", ament_index_python.get_package_share_directory(
                "listen_talk_ros2")+ "/Hey-Willie_en_jetson_v2_1_0.ppn")
            self.declare_parameter("context_path", ament_index_python.get_package_share_directory(
                "listen_talk_ros2")+ "/Dog-command_en_jetson_v2_1_0.rhn")
            self.keyword_path = self.get_parameter("keyword_path").get_parameter_value().string_value
            self.context_path = self.get_parameter("context_path").get_parameter_value().string_value


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
            self.voice_command.data = "not understanding"
            self.pub_voice_command.publish(self.voice_command)

    def run(self):
        recorder = None
        wav_file = None

        try:
            recorder = PvRecorder(device_index=-1, frame_length=self._picovoice.frame_length)
            recorder.start()

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
            

            self.voice_command.data = "willie greet"
            self.pub_voice_command.publish(self.voice_command)
            
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