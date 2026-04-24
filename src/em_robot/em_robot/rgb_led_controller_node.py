#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.node import Node
from std_msgs.msg import ColorRGBA

from em_robot.diagnostic_utils import build_diagnostic_array, build_diagnostic_status
from em_robot.led_utils import resolve_color_pins, scale_color

try:
    from gpiozero import RGBLED
except ImportError:  # pragma: no cover - depends on robot image
    RGBLED = None


class MockRGBLED:
    def __init__(self):
        self.color = (0.0, 0.0, 0.0)

    def off(self):
        self.color = (0.0, 0.0, 0.0)

    def close(self):
        self.off()


@dataclass
class LedConfig:
    name: str
    topic: str
    pins: tuple[int, int, int]
    color_order: str
    current_color: tuple[float, float, float] = (0.0, 0.0, 0.0)


class RgbLedControllerNode(Node):
    def __init__(self):
        super().__init__("rgb_led_controller")

        self.declare_parameter("backend", "mock")
        self.declare_parameter("active_low", False)
        self.declare_parameter("brightness", 1.0)
        self.declare_parameter("diagnostics_rate_hz", 1.0)
        self.declare_parameter("all_topic", "/leds/all/color")

        self.declare_parameter("front_name", "front")
        self.declare_parameter("front_topic", "/leds/front/color")
        self.declare_parameter("front_pins", [23, 24, 25])
        self.declare_parameter("front_color_order", "rgb")

        self.declare_parameter("rear_name", "rear")
        self.declare_parameter("rear_topic", "/leds/rear/color")
        self.declare_parameter("rear_pins", [4, 17, 27])
        self.declare_parameter("rear_color_order", "rgb")

        self.backend = str(self.get_parameter("backend").value)
        self.active_low = bool(self.get_parameter("active_low").value)
        self.brightness = float(self.get_parameter("brightness").value)
        diagnostics_rate_hz = float(self.get_parameter("diagnostics_rate_hz").value)
        self.all_topic = str(self.get_parameter("all_topic").value)

        self.led_configs = {
            "front": LedConfig(
                name=str(self.get_parameter("front_name").value),
                topic=str(self.get_parameter("front_topic").value),
                pins=tuple(int(pin) for pin in self.get_parameter("front_pins").value),
                color_order=str(self.get_parameter("front_color_order").value),
            ),
            "rear": LedConfig(
                name=str(self.get_parameter("rear_name").value),
                topic=str(self.get_parameter("rear_topic").value),
                pins=tuple(int(pin) for pin in self.get_parameter("rear_pins").value),
                color_order=str(self.get_parameter("rear_color_order").value),
            ),
        }

        self.devices = {}
        for key, config in self.led_configs.items():
            self.devices[key] = self._create_device(config)

        self.subscriptions = [
            self.create_subscription(
                ColorRGBA,
                self.led_configs["front"].topic,
                lambda msg: self.led_callback("front", msg),
                10,
            ),
            self.create_subscription(
                ColorRGBA,
                self.led_configs["rear"].topic,
                lambda msg: self.led_callback("rear", msg),
                10,
            ),
            self.create_subscription(
                ColorRGBA,
                self.all_topic,
                self.all_leds_callback,
                10,
            ),
        ]

        self.diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        timer_period = 1.0 / diagnostics_rate_hz if diagnostics_rate_hz > 0.0 else 1.0
        self.diagnostics_timer = self.create_timer(timer_period, self.publish_diagnostics)

        self.get_logger().info(
            f"RGB LED controller started with backend='{self.backend}', active_low={self.active_low}"
        )

    def _create_device(self, config: LedConfig):
        red_pin, green_pin, blue_pin = resolve_color_pins(config.pins, config.color_order)

        if self.backend == "mock":
            return MockRGBLED()

        if self.backend == "gpiozero":
            if RGBLED is None:
                raise RuntimeError(
                    "gpiozero is not available for the gpiozero LED backend"
                )

            return RGBLED(
                red=red_pin,
                green=green_pin,
                blue=blue_pin,
                active_high=not self.active_low,
                initial_value=(0.0, 0.0, 0.0),
                pwm=True,
            )

        raise ValueError(f"Unsupported LED backend: {self.backend}")

    def _msg_to_color(self, msg: ColorRGBA) -> tuple[float, float, float]:
        alpha = float(msg.a)
        brightness = 1.0 if alpha <= 0.0 else alpha
        return scale_color(
            (float(msg.r), float(msg.g), float(msg.b)),
            brightness=self.brightness * brightness,
        )

    def _apply_color(self, led_key: str, color: tuple[float, float, float]):
        self.devices[led_key].color = color
        self.led_configs[led_key].current_color = color

    def led_callback(self, led_key: str, msg: ColorRGBA):
        self._apply_color(led_key, self._msg_to_color(msg))

    def all_leds_callback(self, msg: ColorRGBA):
        color = self._msg_to_color(msg)
        for led_key in self.led_configs:
            self._apply_color(led_key, color)

    def publish_diagnostics(self):
        statuses = []
        for config in self.led_configs.values():
            statuses.append(
                build_diagnostic_status(
                    f"em_robot/leds/{config.name}",
                    DiagnosticStatus.OK,
                    f"{config.name} LED controller active",
                    hardware_id="raspberry_pi_gpio",
                    values={
                        "backend": self.backend,
                        "topic": config.topic,
                        "pins": ",".join(str(pin) for pin in config.pins),
                        "color_order": config.color_order,
                        "active_low": str(self.active_low).lower(),
                        "r": f"{config.current_color[0]:.2f}",
                        "g": f"{config.current_color[1]:.2f}",
                        "b": f"{config.current_color[2]:.2f}",
                    },
                )
            )

        self.diagnostics_pub.publish(
            build_diagnostic_array(statuses, self.get_clock().now().to_msg())
        )

    def shutdown_leds(self):
        for device in self.devices.values():
            device.off()
            device.close()


def main(args=None):
    rclpy.init(args=args)
    node = RgbLedControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down RGB LED controller...")
    finally:
        node.shutdown_leds()
        node.destroy_node()
        rclpy.shutdown()
