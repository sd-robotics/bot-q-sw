#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rclpy
from rclpy.node import Node
from smbus2 import SMBus
from std_msgs.msg import Float32MultiArray

DEBUG = True

class BatteryMonitor(Node):
    # register map (p22 on the datasheet)
    ADDRESS_OF_CONFIGULATION_REGISTER   = 0x00
    ADDRESS_OF_SHUNT_VOLTAGE_REGISTER   = 0x01
    ADDRESS_OF_BUS_VOLTAGE_REGISTER     = 0x02
    ADDRESS_OF_POWER_REGISTER           = 0x03
    ADDRESS_OF_CURRENT_REGISTER         = 0x04
    ADDRESS_OF_CALIBRATION_REGISTER     = 0x05
    ADDRESS_OF_MASK_ENABLE_REGISTER     = 0x06
    ADDRESS_OF_ALERT_LIMIT_REGISTER     = 0x07
    ADDRESS_OF_MANUFACTURER_ID_REGISTER = 0xFE
    ADDRESS_OF_DIE_ID_REGISTER          = 0xFF

    LSB_OF_SHUNT_VOLTAGE = 2.5  / 1000 / 1000  # 2.5  uV
    LSB_OF_BUS_VOLTAGE   = 1.25 / 1000         # 1.25 mV

    def __init__(self, i2c_bus=1, i2c_address=0x40, shunt_resistance=0.1):
        super().__init__('battery_monitor')
        self.get_logger().info(f"Node '{self.get_name()}' is initializing...")

        self.i2c_bus = i2c_bus
        self.i2c_address = i2c_address
        self.shunt_resistance = shunt_resistance
        self.volt = 0
        self.ampere = 0

        self.vi_publisher = self.create_publisher(Float32MultiArray, 'battery_VI', 10)
        self.update_timer = self.create_timer(5.0, self.update_values)
        self.get_logger().info(f"Node '{self.get_name()}' has been initialized.")

    def update_values(self):
        with SMBus(self.i2c_bus) as bus:
            data1_high, data1_low = bus.read_i2c_block_data(self.i2c_address, self.ADDRESS_OF_SHUNT_VOLTAGE_REGISTER, 2)
            data2_high, data2_low = bus.read_i2c_block_data(self.i2c_address, self.ADDRESS_OF_BUS_VOLTAGE_REGISTER, 2)

            data1 = ((data1_high & 0xFF) << 8) | (data1_low & 0xFF)
            data2 = ((data2_high & 0xFF) << 8) | (data2_low & 0xFF)

            # data1が負だった時、負数に変換
            if data1 & 0x8000:
                data1 -= 0x10000

            self.ampere = data1 * self.LSB_OF_SHUNT_VOLTAGE / self.shunt_resistance  # I = V/R  V: Voltage at ends of shunt resistor R:

            self.volt =   ( ((data2_high & 0xFF) << 8) | (data2_low & 0xFF) ) * self.LSB_OF_BUS_VOLTAGE

            self.publish_vi()

    def publish_vi(self):
        msg = Float32MultiArray()
        msg.data = self.volt, self.ampere
        self.vi_publisher.publish(msg)
        if DEBUG:
            self.get_logger().info(f'Battery (VI): "{msg.data}"')
        else:
            self.get_logger().debug(f'Battery (VI): "{msg.data}"')

    def destroy_node(self):
        self.get_logger().info(f"Node '{self.get_name()}' was destroyed.")
        super().destroy_node()

    def __del__(self):
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
