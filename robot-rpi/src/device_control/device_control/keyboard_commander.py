import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

from rpi_interfaces.srv import Speak, Transcribe, Display


class KeyboardCommander(Node):
    def __init__(self):
        super().__init__('keyboard_commander')

        self.speak_client = self.create_client(Speak, "speak_speech")
        if not self.speak_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('speak_speech not available.')
        self.speak_request = Speak.Request()

        self.transcribe_client = self.create_client(Transcribe, "transcribe_speech")
        if not self.transcribe_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('transcribe_service not available.')
        self.transcribe_request = Transcribe.Request()

        self.display_client = self.create_client(Display, "control_display")
        if not self.display_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('control_display not available.')
        self.display_request = Display.Request()

        self.thread = threading.Thread(target=self.key_listener, daemon=True)
        self.thread.start()

    def key_listener(self):
        print("input command: ", end="")
        while rclpy.ok():
            command = input()
            if command.startswith("speak ") and 2 < len(command.split()):
                self.get_logger().info(f'speak')

                speech = " ".join(command.split()[2:])
                lang = command.split()[1]
                self.send_speak_request(speech, lang)

            elif command.startswith("trs ") and len(command.split()) == 2:
                self.get_logger().info(f'transcribe')
                lang = command.split()[1]
                self.send_transcribe_request(lang)

            elif command.startswith("display ") and len(command.split()) == 2:
                self.get_logger().info(f'display')
                command = command.split()[1]
                self.send_display_request(command)

            else:
                msg = String()
                msg.data = command
                self.get_logger().info(f'KeyCom: "{msg.data}"')

    def send_speak_request(self, speech, lang):
        self.speak_request.speech = speech
        self.speak_request.lang = lang
        self.future = self.speak_client.call_async(self.speak_request)
        self.future.add_done_callback(self.response_callback)

    def send_transcribe_request(self, lang):
        self.transcribe_request.lang = lang
        self.future = self.transcribe_client.call_async(self.transcribe_request)
        self.future.add_done_callback(self.response_callback)

    def send_display_request(self, command):
        self.display_request.command = command
        self.future = self.display_client.call_async(self.display_request)
        self.future.add_done_callback(self.response_callback)

    # service からresponceがあると呼ばれる。
    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'response: "{response}"')
            print("input command: ", end="", flush=True)

        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")

        return


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardCommander()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
