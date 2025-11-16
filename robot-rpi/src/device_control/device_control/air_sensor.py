#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rclpy
from rclpy.node import Node
from smbus2 import SMBus
from std_msgs.msg import Float32MultiArray
from scd30_i2c import SCD30

DEBUG = True


class AirSensor(Node):
    TRIAL = 10
    UPDATE_INTERVAL = 5.0  # sec

    def __init__(self):
        super().__init__('air_sensor')
        self.get_logger().info(f"Node '{self.get_name()}' is initializing...")

        self.airqualiity_publisher = self.create_publisher(Float32MultiArray, 'air_quality', 10)
        self.co2 = 0
        self.temp = 0
        self.humidity = 0
        self.update_timer = self.create_timer(self.UPDATE_INTERVAL, self.update_values)

        self.scd30 = SCD30()
        self.scd30.set_measurement_interval(2)
        self.scd30.start_periodic_measurement()

        self.get_logger().info(f"Node '{self.get_name()}' has been initialized.")

    def publish_airqualiity(self):
        msg = Float32MultiArray()
        msg.data = self.co2, self.temp, self.humidity
        self.airqualiity_publisher.publish(msg)

    def update_values(self):
        for n in range(self.TRIAL):
            if self.scd30.get_data_ready():
                values = self.scd30.read_measurement()
                if values is not None:
                    self.co2, self.temp, self.humidity = values
                    self.publish_airqualiity()

                    if DEBUG:
                        self.get_logger().info(f"CO2: {self.co2:.2f}ppm, temp: {self.temp:.2f}'C, rh: {self.humidity:.2f}%")

                    break

            time.sleep(0.2)

        else:
            self.get_logger().error('Failed to read air quality.')

    def destroy_node(self):
        self.get_logger().info(f"Node '{self.get_name()}' was destroyed.")
        super().destroy_node()

    def __del__(self):
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AirSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
