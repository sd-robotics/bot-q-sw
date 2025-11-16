#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from collections import OrderedDict
import os
import pathlib
import rclpy
from rclpy.node import Node
import threading
import tkinter
from PIL import Image, ImageTk
import itertools

from std_msgs.msg import Float32MultiArray
from rpi_interfaces.srv import Display


class DisplayController(Node):
    SUPPORTED_COMMAND = ["smile", "neutral", "air_quality"]
    GIF_NAMES = ["neutral", "smile"]

    def __init__(self):
        super().__init__("display_controller")
        self.get_logger().info(f"Node '{self.get_name()}' is initializing...")

        self.display_service = self.create_service(Display, "control_display", self.display_service_callback)
        self.gui = Gui(logger=self.get_logger(), thread_func=self.ros_spin, thread_delay=0.05)

        self.air_quality_subscription = self.create_subscription(Float32MultiArray, 'air_quality', self.air_quality_subscription_callback, 10)

        self.get_logger().info(f"Node '{self.get_name()}' has been initialized.")

    # ros_spin() is executed in a sub separate thread since the mainloop() of tkinter can only run on the main thread.
    def ros_spin(self):
        rclpy.spin_once(self, timeout_sec=0.05)

    def display_service_callback(self, request, response):
        self.get_logger().info(f"Display got request: {request.command}")
        if request.command not in self.SUPPORTED_COMMAND:
            self.get_logger().error(f'Unsupported command was specified: "{request.command}"')
            response.status = 1

        elif request.command in ["smile", "neutral"]:
            face_type = request.command
            self.gui.activate_panel("face")
            self.gui.get_active_panel().change_face(face_type)
            self.gui.update()

            response.status = 0

        elif request.command == "air_quality":
            self.gui.activate_panel("air_quality")
            self.gui.update()

            response.status = 0

        else:
            self.get_logger().error(f'Unknown request error for display_service: "{request.command}"')
            response.status = 1

        return response

    def air_quality_subscription_callback(self, msg):
        co2, temp, humidity = msg.data
        self.gui.panels["air_quality"].co2       = co2
        self.gui.panels["air_quality"].temp      = temp
        self.gui.panels["air_quality"].humidity  = humidity

    def destroy_node(self):
        self.get_logger().info(f"Node '{self.get_name()}' was destroyed.")
        super().destroy_node()


class Gui(tkinter.Tk):
    DEFAULT_DELAY = 0.1

    def __init__(self, thread_func=None, thread_delay=0.05, logger=None):
        for i in range(4):
            display = ':' + str(i)
            os.environ['DISPLAY'] = display
            try:
                super().__init__()
                logger.info(f'Connected to display {display}')
                break
            except tkinter.TclError:
                pass
        else:
            logger.error(f'Failed to connect display.')

        self.thread_func = thread_func
        self.thread_delay = thread_delay
        self.logger = logger

        self.title("GIFPlayer")
        self.overrideredirect(True)  # Disable the window frame
        self.geometry("720x720+0+0")
        self.config(bg="black")

        # The last element of $panels is the active panel in the GUI.
        self.panels = OrderedDict()
        self.panels["face"] = FacePanel(self)
        self.panels["air_quality"] = AirQualityPanel(self)
        self.activate_panel("face")

        self.update_after_id = None

        self.after(50, self.run_thread)
        self.after(50, self.update)

    def run_thread(self):
        if self.thread_func is None:
            return

        self.thread_func()
        self.after(int(self.thread_delay*1000), self.run_thread)

    # Update the active panel and reserve calling itself after $next_update_delay sec.
    def update(self):
        if self.update_after_id is not None:
            self.after_cancel(self.update_after_id)

        next_update_delay = self.get_active_panel().update()

        if next_update_delay:
            self.update_after_id = self.after(int(next_update_delay*1000), self.update)
        else:
            self.update_after_id = self.after(int(self.DEFAULT_DELAY*1000), self.update)

    def activate_panel(self, panel_name):
        self.panels.move_to_end(panel_name)
        self.panels[panel_name].tkraise()

    def get_active_panel_names(self):
        return list(self.panels.keys())[-1]

    def get_active_panel(self):
        return list(self.panels.values())[-1]

# Base class for panels defining GUI functions
class Panel(tkinter.Frame):
    DEFAULT_DELAY = 0.1  # sec

    def __init__(self, master, logger=None):
        super().__init__(master)
        self.grid(row=0, column=0, sticky="nsew")

        self.master = master
        self.logger = logger

    # Override this. update() can return next update timing [sec]. If it returns None. $DEFAULT_DELAY will be used.
    def update(self):
        pass

class FacePanel(Panel):
    FACES = ["neutral", "smile"]
    GIF_DIR = pathlib.Path("~/material/robot_face/").expanduser()
    DEFAULT_GIF_NAME = "neutral.gif"  # File to read instead when file fails to load

    def __init__(self, master, logger=None):
        super().__init__(master, logger=None)

        self.face = self.FACES[0]

        self.gif_frame_iters = {}
        self.frame_iter = None

        self.load_gifs()

        self.label = tkinter.Label(self)
        self.label.pack()
        self.label.config(bg="black")

        self.change_face("neutral")

    def update(self):
        image, delay = next(self.frame_iter)
        self.label.config(image=image)

        return delay

    def change_face(self, face_type):
        if face_type in self.FACES:
            _, self.frame_iter = itertools.tee(self.gif_frame_iters[face_type])

    def open_gif(self, gif_name):
        try:
            gif = Image.open(self.GIF_DIR / (gif_name + ".gif"))
            gif = Image.open(self.GIF_DIR / (gif_name + ".gif"))

            return gif
        except Exception as e:
            if self.logger:
                self.logger().error(f"Failed to open '{self.GIF_DIR + gif_name}'. Using '{self.DEFAULT_GIF_NAME}'.gif instead.\n{e}")

            default_gif = Image.open(self.GIF_DIR / self.DEFAULT_GIF_NAME)
            return default_gif

    def load_gifs(self):
        for gif_name in self.FACES:
            gif = self.open_gif(gif_name)
            frames = []
            try:
                while True:
                    image = ImageTk.PhotoImage(gif.convert("RGBA"))
                    delay = gif.info.get("duration", 0)
                    frames += [(image, delay/1000)]

                    gif.seek(gif.tell() + 1)

            except EOFError:
                if self.logger:
                    self.logger().info(gif_name + ".gif was loaded.")
                pass

            self.gif_frame_iters[gif_name] = itertools.cycle(frames)


class AirQualityPanel(Panel):
    FONT = "Monospace"
    FONTSIZE = 48
    PADY = 10

    def __init__(self, master, logger=None):
        super().__init__(master, logger=None)

        self.co2 = None
        self.temp = None
        self.humidity = None

        self.grid_rowconfigure((0, 4), weight=1)
        self.grid_rowconfigure((1, 2, 3), weight=0)
        self.grid_columnconfigure(0, weight=1)

        self.co2_label               = tkinter.Label(self, text="- ppm", font=(self.FONT, self.FONTSIZE), bg="black", fg="white")
        self.temp_label              = tkinter.Label(self, text="- °C ", font=(self.FONT, self.FONTSIZE), bg="black", fg="white")
        self.humidity_label          = tkinter.Label(self, text="-  % ", font=(self.FONT, self.FONTSIZE), bg="black", fg="white")

        self.co2_label.grid(row=1, column=0, pady=self.PADY)
        self.temp_label.grid(row=2, column=0, pady=self.PADY)
        self.humidity_label.grid(row=3, column=0, pady=self.PADY)

        self.config(bg="black")

        self.update()

    def update(self):
        str_co2      = str(round(self.co2)).rjust(5) if self.co2 is not None else " - "
        str_temp     = str(round(self.temp, 1)).rjust(5) if self.temp is not None else " - "
        str_humidity = str(round(self.humidity, 1)).rjust(5) if self.humidity is not None else " - "

        self.co2_label.config(     text="CO2 :" + str_co2      + " ppm")
        self.temp_label.config(    text="Temp:" + str_temp     + " °C ")
        self.humidity_label.config(text="Hum :" + str_humidity + " %  ")

        return 1


def main(args=None):
    rclpy.init(args=args)
    display_node = DisplayController()

    try:
        display_node.gui.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        if display_node:
            display_node.gui.quit()
            display_node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()
