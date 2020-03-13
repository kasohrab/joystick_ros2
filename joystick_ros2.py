# Copyright 2017 Muhammad Furqan Habibi
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

import platform
import time
from math import modf

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from sensor_msgs.msg import Joy
from std_msgs.msg import Header
from inputs import devices, UnpluggedError, GamePad

# Microsoft X-Box 360 pad
XINPUT_CODE_MAP = {
    'ABS_X': 0,
    'ABS_Y': 1,
    'ABS_Z': 2,
    'ABS_RX': 3,
    'ABS_RY': 4,
    'ABS_RZ': 5,
    'ABS_HAT0X': 6,
    'ABS_HAT0Y': 7,
    'BTN_SOUTH': 0,
    'BTN_EAST': 1,
    'BTN_WEST': 2,
    'BTN_NORTH': 3,
    'BTN_TL': 4,
    'BTN_TR': 5,
    'BTN_START': 6,
    'BTN_SELECT': 7,
    'BTN_MODE': 8,
    'BTN_THUMBL': 9,
    'BTN_THUMBR':10
}

XINPUT_VALUE_MAP = {
    0: (-32768, 32767),
    1: (32767, -32768),
    2: (0, 255),
    3: (-32768, 32767),
    4: (32767, -32768),
    5: (0, 255),
    6: (-1, 1),
    7: (-1, 1)
}

# Sony Computer Entertainment Wireless Controller
PS4_CODE_MAP = {
    'ABS_X': 0,
    'ABS_Y': 1,
    'ABS_RX': 2,
    'ABS_Z': 3,
    'ABS_RZ': 4,
    'ABS_RY': 5,
    'ABS_HAT0X': 6,
    'ABS_HAT0Y': 7,
    'BTN_EAST': 0,
    'BTN_C': 1,
    'BTN_SOUTH': 2,
    'BTN_NORTH': 3,
    'BTN_WEST': 4,
    'BTN_Z': 5,
    'BTN_TL2': 6,
    'BTN_TR2': 7,
    'BTN_MODE': 8,
    'BTN_SELECT': 9,
    'BTN_START':10
}

PS4_VALUE_MAP = {
    0: (0, 255),
    1: (0, 255),
    2: (0, 255),
    3: (0, 255),
    4: (0, 255),
    5: (0, 255),
    6: (-1, 1),
    7: (-1, 1)
}

# Logitech Gamepad F710
F710_CODE_MAP = {
    'ABS_X': 0,
    'ABS_Y': 1,
    'ABS_Z': 2,
    'ABS_RX': 3,
    'ABS_RY': 4,
    'ABS_RZ': 5,
    'ABS_HAT0X': 6,
    'ABS_HAT0Y': 7,
    'BTN_SOUTH': 0,
    'BTN_EAST': 1,
    'BTN_NORTH': 2,
    'BTN_WEST': 3,
    'BTN_TL': 4,
    'BTN_TR': 5,
    'BTN_SELECT': 6,
    'BTN_START': 7,
    'BTN_MODE': 8,
    'BTN_THUMBL': 9,
    'BTN_THUMBR':10
}

F710_VALUE_MAP = {
    0: (-32768, 32767),
    1: (-32768, 32767),
    2: (0, 255),
    3: (-32768, 32767),
    4: (-32768, 32767),
    5: (0, 255),
    6: (-1, 1),
    7: (-1, 1)
}

# Microsoft X-Box One pad
XONE_CODE_MAP = {
    'ABS_X': 0,
    'ABS_Y': 1,
    'ABS_Z': 2,
    'ABS_RX': 3,
    'ABS_RY': 4,
    'ABS_RZ': 5,
    'ABS_HAT0X': 6,
    'ABS_HAT0Y': 7,
    'BTN_SOUTH': 0,
    'BTN_EAST': 1,
    'BTN_NORTH': 2,
    'BTN_WEST': 3,
    'BTN_TL': 4,
    'BTN_TR': 5,
    'BTN_SELECT': 6,
    'BTN_START': 7,
    'BTN_MODE': 8,
    'BTN_THUMBL': 9,
    'BTN_THUMBR':10
}

XONE_VALUE_MAP = {
    0: (-32768, 32767),
    1: (-32768, 32767),
    2: (0, 1023),
    3: (-32768, 32767),
    4: (-32768, -32767),
    5: (0, 1023),
    6: (-1, 1),
    7: (-1, 1)
}

# Microsoft X-Box One pad
XONE_S_CODE_MAP = XONE_CODE_MAP

XONE_S_VALUE_MAP = {
    0: (-32768, 32767),
    1: (-32768, 32767),
    2: (-1023, 1023),
    3: (-32768, 32767),
    4: (-32768, 32767),
    5: (-1023, 1023),
    6: (-1, 1),
    7: (-1, 1)
}

JOYSTICK_CODE_VALUE_MAP = {
    'Microsoft X-Box 360 pad': (XINPUT_CODE_MAP, XINPUT_VALUE_MAP),
    'Sony Computer Entertainment Wireless Controller': (PS4_CODE_MAP, PS4_VALUE_MAP),
    'Logitech Gamepad F710': (F710_CODE_MAP, F710_VALUE_MAP),
    'Microsoft X-Box One pad': (XONE_CODE_MAP, XONE_VALUE_MAP),
    'Microsoft X-Box One S pad': (XONE_S_CODE_MAP, XONE_S_VALUE_MAP),
}

class JoystickRos2(Node):

    DEFAULT_DEADZONE: float = 0.05
    DEFAULT_AUTOREPEAT_RATE: float = 0.0
    DEFAULT_COALESCE_INTERVAL: float = 0.001
    DEFAULT_SLEEP_TIME: float = 0.01

    def __init__(self):
        super().__init__('joystick_ros2')

        self.declare_parameters(None, [
            ("deadzone", self.DEFAULT_DEADZONE),
            ("autorepeat_rate", self.DEFAULT_AUTOREPEAT_RATE),
            ("coalesce_interval", self.DEFAULT_COALESCE_INTERVAL),
        ])

        # Joy message
        self.joy = Joy()
        self.joy.header = Header()
        self.joy.header.frame_id = ''
        self.joy.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joy.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        # Joy publisher
        self.publisher_ = self.create_publisher(Joy, 'joy', 10)

        # logic params
        self.last_event = None
        self.last_publish_time = 0

        # Timers
        self.device_timers = self.create_timer(1.0, self.detect_devices)
        self.run_timer = self.create_timer(0.01, self.run_one)

    @property
    def deadzone(self) -> float:
        return self.get_parameter_or('deadzone', self.DEFAULT_DEADZONE).value

    @property
    def autorepeat_rate(self) -> float:
        return self.get_parameter_or('autorepeat_rate', self.DEFAULT_AUTOREPEAT_RATE).value

    @property
    def coalesce_interval(self) -> float:
        return self.get_parameter_or('coalesce_interval', self.DEFAULT_COALESCE_INTERVAL).value

    def publish_joy(self):
        current_time = modf(time.time())
        self.joy.header.stamp.sec = int(current_time[1])
        self.joy.header.stamp.nanosec = int(current_time[0] * 1000000000) & 0xffffffff
        self.publisher_.publish(self.joy)
        self.last_publish_time = time.time()

    def normalize_key_value(self, key_value_min, key_value_max, key_value):
        normalized = ((key_value - key_value_min) / (key_value_max - key_value_min) * (-2)) + 1
        if (abs(normalized) > self.deadzone):
            return normalized
        else:
            return 0.0

    def detect_devices(self):
        devices.find_devices()

        if len(devices.gamepads) == 0:
            self.get_logger().warning('No joystick was found. Rescanning')
            return

        if devices.gamepads[0].name not in JOYSTICK_CODE_VALUE_MAP:
            self.get_logger().warning('Sorry, joystick type not supported yet! Please plug in supported joystick')
            return


    def run_one(self):
        try:
            gamepad: GamePad = devices.gamepads[0]
        except IndexError:
            return

        try:
            for event in gamepad:
                if (event.code in JOYSTICK_CODE_VALUE_MAP[event.device.name][0]):
                    key_code = JOYSTICK_CODE_VALUE_MAP[event.device.name][0][event.code]
                    if (event.ev_type == 'Key'):
                        self.joy.buttons[key_code] = event.state
                        self.publish_joy()
                        self.last_event = event
                    elif (event.ev_type == 'Absolute'):
                        value_range = JOYSTICK_CODE_VALUE_MAP[event.device.name][1][key_code]
                        self.joy.axes[key_code] = self.normalize_key_value(value_range[0], value_range[1], event.state)
                        if (self.last_event is None) or (self.last_event.code != event.code) or (time.time() - self.last_publish_time > self.coalesce_interval):
                            self.publish_joy()
                        self.last_event = event
        except OSError:
            pass

        if ((self.autorepeat_rate > 0.0) and (time.time() - self.last_publish_time > 1/self.autorepeat_rate)):
            self.publish_joy()


def main(args=None):
    rclpy.init(args=args)

    try:
        joystick_ros2 = JoystickRos2()
        executor = SingleThreadedExecutor()
        executor.add_node(joystick_ros2)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            joystick_ros2.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
