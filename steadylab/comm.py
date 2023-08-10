# ----------------------------------------------------------------------------
# Copyright (C) [2023] Byounggun Park
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ----------------------------------------------------------------------------


#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import serial
import numpy as np
from steadylab.msg import ErpRead, ErpWrite

# Constants
MAX = 18
PI = np.pi


class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        
        # Variables
        self.steer1 = 0
        self.steer2 = 0
        self.speed1 = 0
        self.brake = 0
        self.gear = 0
        self.E_stop = False
        self.inputbool = 0
        
        # Publishers, Subscribers and Timers
        self.serial_pub = self.create_publisher(ErpRead, "erp_read", 1)
        self.serial_sub = self.create_subscription(ErpWrite, "erp_write", self.write_callback, 1)
        
        # Initialize serial communication
        self.ser = serial.Serial(port="/dev/ttyUSB0", baudrate=115200, timeout=1)
        if self.ser.isOpen():
            self.get_logger().info("Serial Port initialized")
        else:
            self.get_logger().fatal("Unable to open serial port")
            exit(1)

        self.timer = self.create_timer(0.02, self.main_loop)  # 50 Hz

        self.a = 0
        self.PCU_to_UPPER = [0x00] * 18
        self.erp42_state = ErpRead()

    def write_callback(self, write):
        self.inputbool = 1
        self.E_stop = write.write_e_stop
        self.gear = write.write_gear
        steer = write.write_steer
        self.steer1 = int(steer / 256)
        self.steer2 = int(steer % 256)
        if steer < 0: 
            self.steer1 = 0
        self.speed1 = write.write_speed
        self.brake = write.write_brake

    def main_loop(self):
        if self.ser.in_waiting:
            self.PCU_to_UPPER = list(self.ser.read(18))
            self.get_logger().info(" ".join("{:02X}".format(c) for c in self.PCU_to_UPPER))

            # When the callback function is called
            if self.inputbool:
                # self.a += 1
                self.a = (self.a + 1) % 256
                UPPER_to_PCU = [0x53, 0x54, 0x58, 0x01, 0x00, self.gear, 0x00, self.speed1, self.steer1, self.steer2, self.brake, self.a, 0x0D, 0x0A]
                self.ser.write(bytearray(UPPER_to_PCU))

            # When the callback function is not called
            else:
                # self.a += 1
                self.a = (self.a + 1) % 256
                UPPER_to_PCU = [0x53, 0x54, 0x58, 0x01, 0x00, 0x01, 0x00, 0x00, self.steer1, self.steer2, 0x80, self.a, 0x0D, 0x0A]
                self.ser.write(bytearray(UPPER_to_PCU))


def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    executor = MultiThreadedExecutor()

    try:
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
