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


#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from steadylab.msg import ReadCar
from steadylab.msg import WriteCar

class Serial(Node):  # Node 클래스를 상속받습니다.
    def __init__(self):
        super().__init__('main_status')  # Node의 __init__ 메서드를 호출하여 노드 이름을 설정합니다.
        
        qos = QoSProfile(depth=1)
        self.speed = 0
        self.steer = 0

        self.erp_sub = self.create_subscription(WriteCar, 'from_bev', self.callback, qos)
        self.erp_pub = self.create_publisher(WriteCar, 'write_car', qos)
        self.erp = WriteCar()

    def callback(self, data):
        self.speed = data.read_speed
        self.steer = data.read_steer

        return self.speed, self.steer

    
    def pub_serial(self, speed, steer):
        self.erp.write_speed = speed
        self.erp.write_steer = steer

        self.erp_pub.publish(self.erp)

def main(args=None):
    rclpy.init(args=args)  # rclpy를 초기화합니다.

    s = Serial()

    rate = s.create_rate(10)  # Rate를 생성합니다.

    while rclpy.ok():  # rclpy.is_shutdown() 대신 rclpy.ok()를 사용

        speed, steer = s.callback()
        print("speed", speed)
        print("steer", steer)
        s.pub_serial(speed, steer)

        rate.sleep()

    s.destroy_node()
    rclpy.shutdown()  # rclpy를 종료합니다.

if __name__ == '__main__':
    main()
