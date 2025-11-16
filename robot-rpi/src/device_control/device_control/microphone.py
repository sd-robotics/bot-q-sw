#!/usr/bin/env python3
import json
import queue
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import re
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Int16MultiArray, Float32MultiArray
import sounddevice as sd
import numpy as np
import os
# import pyaudio
import speech_recognition as sr
import time
from vosk import Model, KaldiRecognizer
import wave

from rpi_interfaces.srv import Transcribe, Display, Speak

SAMPLERATE = 44100
# CHUNK_SIZE = 4000
CHUNK_SIZE = 2048
CHANNELS = 1
# JA_MODEL_PATH = os.path.expanduser("~/material/vosk-model-ja-0.22")         # Downloads link: https://alphacephei.com/vosk/models/vosk-model-ja-0.22.zip, Licence: Apache 2.0
# Downloads link: https://alphacephei.com/vosk/models/vosk-model-small-ja-0.22.zip, Licence: Apache 2.0
JA_MODEL_PATH = os.path.expanduser("~/material/vosk-model-small-ja-0.22")
# EN_MODEL_PATH = os.path.expanduser("~/material/vosk-model-en-us-0.22")      # Downloads link: https://alphacephei.com/vosk/models/vosk-model-en-0.22.zip, Licence: Apache 2.0
# Downloads link: https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip, Licence: Apache 2.0
EN_MODEL_PATH = os.path.expanduser("~/material/vosk-model-small-en-us-0.15")

SUPPORTED_LANG = ["en", "ja"]

ja_model = Model(JA_MODEL_PATH)
en_model = Model(EN_MODEL_PATH)


class Transcriber(Node):
    WAKE_WORD = r"ねえ( )?ねえ( )?(イケボ)"
    WAKE_DIC = '["ねえ", "イケボ"]'
    ONESHOT_TIMER_INTERVAL = 0.001
    REC_DURATION = 4.0

    def __init__(self):
        super().__init__('transcriber')
        self.get_logger().info(f"Node '{self.get_name()}' is initializing...")

        self.transcribe_service = self.create_service(Transcribe, 'transcribe_speech', self.transcribe_service_callback)

        self.ja_recognizer = KaldiRecognizer(ja_model, SAMPLERATE)
        self.ja_trigger_recognizer = KaldiRecognizer(ja_model, SAMPLERATE, self.WAKE_DIC)

        self.get_logger().info(f"transcribe_server start")

        self.audio_buffer = queue.Queue()
        self.oneshot_timer = self.create_timer(5.0, self.detect_name_call)  # One shot execution.
        self.enable_detect_name_call = True
        self.get_logger().info('recognizer ready.')

        self.stream = sd.RawInputStream(dtype='int16',
                                        channels=CHANNELS,
                                        callback=self.input_stream_callback)

        ####### for demo ######
        self.display_client = self.create_client(Display, "control_display")
        if not self.display_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('display_service not available.')
        self.display_request = Display.Request()

        self.speak_client = self.create_client(Speak, "speak_speech")
        if not self.speak_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('speak_service not available.')
        self.speak_request = Speak.Request()
        ####### for demo ######

        self.get_logger().info(f"Node '{self.get_name()}' has been initialized.")

    def input_stream_callback(self, indata, frames, time, status):
        if status:
            self.get_logger().error(f"input stream status {status}")

        self.audio_buffer.put(bytes(indata))

    def transcribe_service_callback(self, request, response):
        self.enable_detect_name_call = False

        if request.lang not in SUPPORTED_LANG:
            self.get_logger().error(f'Unsupported langage was specified: "{request.lang}"')
            response.status = 1
            return response

        response.speech = self.transcribe(lang=request.lang)
        response.status = 0
        self.enable_detect_name_call = True
        self.get_logger().info(f"transcribe:'{response.speech}'")
        return response

    def transcribe(self, lang="ja", duration=REC_DURATION):
        self.destroy_timer(self.oneshot_timer)  # Delete timer for one shot execution.
        with self.audio_buffer.mutex:
            self.audio_buffer.queue.clear()

        if self.stream.active:
            self.get_logger().info("The mic stream for call detection restarted.")
            self.stream.stop()

        self.get_logger().info('● REC start')
        audio_data = sd.rec(int(duration * SAMPLERATE), samplerate=SAMPLERATE, channels=CHANNELS, dtype='int16')
        sd.wait()
        self.get_logger().info('■ REC stop')
        self.get_logger().info('Transcribing...⌛')
        byte_data = audio_data.tobytes()
        speech = ""
        for i in range(0, len(byte_data), CHUNK_SIZE):
            chunk = byte_data[i:i + CHUNK_SIZE]
            if self.ja_recognizer.AcceptWaveform(chunk):
                result = self.ja_recognizer.Result()
                speech += eval(result)['text'] + " "

        final_result = self.ja_recognizer.FinalResult()
        speech += eval(final_result)['text']
        self.get_logger().info('Transcribe completed! 👍')

        self.enable_detect_name_call = True

        self.oneshot_timer = self.create_timer(self.ONESHOT_TIMER_INTERVAL, self.detect_name_call)
        return speech

    def detect_name_call(self):
        self.destroy_timer(self.oneshot_timer)  # Delete timer for one shot execution.

        if not self.stream.active:
            try:
                self.stream.start()
                self.get_logger().info("The mic stream for call detection restarted.")
            except Exception as e:
                self.get_logger().error(f"Error on restarting the mic stream for call detection: {e}")
                self.stream.close()
                return False

        for n in range(4):
            audio_data = self.audio_buffer.get()

            if audio_data and self.ja_trigger_recognizer.AcceptWaveform(audio_data):

                result = self.ja_trigger_recognizer.Result()
                result_text = json.loads(result).get("text", "")

                if re.search(self.WAKE_WORD, result_text):
                    self.enable_detect_name_call = False
                    self.audio_buffer.queue.clear()
                    speech = self.transcribe()

                    ###### for demo ######
                    self.get_logger().info(f"speech: {speech}")
                    self.reaction_controll(speech)
                    ###### for demo ######

                    break

                ###### for demo ######
                if result_text:
                    self.get_logger().info(result_text)
                else:
                    print("no input", self.audio_buffer.qsize())
                ###### for demo ######

            if self.audio_buffer.qsize() > 500:
                self.get_logger().warn("over size", self.audio_buffer.qsize())
                with self.audio_buffer.mutex:
                    self.audio_buffer.queue.clear()

        self.oneshot_timer = self.create_timer(self.ONESHOT_TIMER_INTERVAL, self.detect_name_call)

    ####### for demo ######

    def reaction_controll(self, speech):
        self.get_logger().info(speech.replace(" ", ""))
        if re.search(r"(わら|笑|あら|洗)って", speech.replace(" ", "")):
            self.get_logger().info("笑って")
            self.send_display_request("smile")
        elif re.search(r"(わら|笑|あら|洗)わないで", speech.replace(" ", "")):
            self.get_logger().info("笑わないで")
            self.send_display_request("neutral")
        elif re.search(r"空気品質", speech.replace(" ", "")):
            self.get_logger().info("空気品質")
            self.send_display_request("air_quality")
            self.send_speak_request("空気品質を表示します", "ja")
        else:
            self.get_logger().info("???")
    ####### for demo ######

    def send_display_request(self, command):
        self.display_request.command = command
        self.future = self.display_client.call_async(self.display_request)
        self.future.add_done_callback(self.response_callback)

    def send_speak_request(self, speech, lang):
        self.speak_request.speech = speech
        self.speak_request.lang = lang
        self.future = self.speak_client.call_async(self.speak_request)
        self.future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'response: "{response}"')
            self.get_logger().info("input command: ", end="", flush=True)

        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")

        return
    ####### for demo ######

    # def record(self, duration, file_path=os.path.expanduser("~/output/rec.wav")):
    #     print('● REC start')
    #     audio_data = sd.rec(int(duration * SAMPLERATE), samplerate=SAMPLERATE, channels=CHANNELS, dtype='int16')
    #     sd.wait()
    #     print('■ REC stop')
    #
    #
    #     with wave.open(file_path, "wb") as wf:
    #         wf.setnchannels(CHANNELS)
    #         wf.setsampwidth(2)  # 16ビット（2バイト）
    #         wf.setframerate(SAMPLERATE)
    #         wf.writeframes(audio_data.tobytes())

    def destroy_node(self):
        self.stream.close()
        self.get_logger().info(f"Node '{self.get_name()}' was destroyed.")
        super().destroy_node()

    def __del__(self):
        self.destroy_node()
        self.get_logger().info('Microphone node was destroyed.')


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()

    transcriber = Transcriber()

    executor.add_node(transcriber)

    try:
        executor.spin()

    except KeyboardInterrupt:
        pass
    finally:
        transcriber.destroy_node()

        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
