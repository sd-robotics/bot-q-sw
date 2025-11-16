#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16MultiArray, Float32MultiArray
import numpy as np

import subprocess

import sounddevice as sd
from scipy.io import wavfile

from rpi_interfaces.srv import Speak


class Speaker(Node):
    SUPPORTED_LANG = ["en", "ja"]

    JA_DIC_PATH = "/var/lib/mecab/dic/open-jtalk/naist-jdic"
    JA_VOICE_PATH = "/usr/local/share/hts-voice/mei_normal.htsvoice"
    JA_SAMPLERATE = 24000
    JA_ADD_SEMITONE = 2.0
    JA_ALLPASS_CONSTANT = 0.24  # Change this make the voice sounds different.
    # JA_POSTFILTER = 0.4  # make the quality higher but increase calcuration cost. (about x3)

    def __init__(self):
        super().__init__('speaker')
        self.get_logger().info(f"Node '{self.get_name()}' is initializing...")
        subprocess.run(["espeak", "Speaker test."])

        self.speak_service = self.create_service(
            Speak, 'speak_speech', self.speak_service_callback)

        self.get_logger().info(f"Node '{self.get_name()}' has been initialized.")

    def speak_service_callback(self, request, response):
        if request.lang not in self.SUPPORTED_LANG:
            self.get_logger().error(
                f'Unsupported langage was specified: "{request.lang}"')
            response.status =  1
            return response

        self.speak(request.speech, request.lang)
        self.get_logger().info(f'speech:"{request.speech}"')
        response.status = 0

        return response

    def speak(self, speech, lang="en"):
        if lang == "en":
            self._speak_en(speech)
        elif lang == "ja":
            self._speak_ja(speech)

    def _speak_en(self, speech):
        subprocess.run(["espeak", speech])

    def _speak_ja(self, speech):
        command = [
            "open_jtalk",
            "-x", self.JA_DIC_PATH,
            "-m", self.JA_VOICE_PATH,
            "-ow", "/dev/stdout",
            "-s", str(self.JA_SAMPLERATE),
            "-p", str(self.JA_SAMPLERATE // 200),
            "-a", str(self.JA_ALLPASS_CONSTANT),
            "-fm", str(self.JA_ADD_SEMITONE),

            # "-b", str(self.JA_POSTFILTER)
        ]
        result = subprocess.run(
            command,
            input=speech.encode("utf-8"),  # 入力テキストをエンコード
            stdout=subprocess.PIPE,  # 標準出力をキャプチャ
            check=True
        )

        audio_data = result.stdout[44:]  # Skip the wav header (44 bytes).
        audio_array = np.frombuffer(audio_data, dtype=np.int16)
        sd.play(audio_array, self.JA_SAMPLERATE)
        sd.wait()

    def destroy_node(self):
        self.get_logger().info(f"Node '{self.get_name()}' was destroyed.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Speaker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
