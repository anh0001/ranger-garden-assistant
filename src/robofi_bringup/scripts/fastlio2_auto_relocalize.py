#!/usr/bin/env python3

"""
Automatically trigger FASTLIO2's relocalizer with a saved PCD map so the map->odom
TF is ready before Nav2 accepts navigation goals.
"""

import os
from typing import Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.task import Future

from interface.srv import Relocalize


class FastLioAutoRelocalize(Node):
    """Call /localizer/relocalize once when a valid map_path is supplied."""

    def __init__(self) -> None:
        super().__init__("fastlio2_auto_relocalize")

        # Parameters provided from the launch file.
        self.declare_parameter("map_path", "")
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_z", 0.0)
        self.declare_parameter("initial_roll", 0.0)
        self.declare_parameter("initial_pitch", 0.0)
        self.declare_parameter("initial_yaw", 0.0)
        self.declare_parameter("timeout_sec", 30.0)

        self._map_path: str = self.get_parameter("map_path").get_parameter_value().string_value
        self._initial_x: float = float(self.get_parameter("initial_x").value)
        self._initial_y: float = float(self.get_parameter("initial_y").value)
        self._initial_z: float = float(self.get_parameter("initial_z").value)
        self._initial_roll: float = float(self.get_parameter("initial_roll").value)
        self._initial_pitch: float = float(self.get_parameter("initial_pitch").value)
        self._initial_yaw: float = float(self.get_parameter("initial_yaw").value)
        self._timeout = Duration(seconds=float(self.get_parameter("timeout_sec").value))

        if not self._map_path:
            self.get_logger().info("map_path not provided; skipping automatic relocalization.")
            self._shutdown_soon()
            return

        if not os.path.isfile(self._map_path):
            self.get_logger().warning(f"map_path '{self._map_path}' not found; skipping relocalization.")
            self._shutdown_soon()
            return

        self.get_logger().info(
            (
                "Auto-relocalizing FASTLIO2 using %s "
                "(x=%.2f y=%.2f z=%.2f roll=%.2f pitch=%.2f yaw=%.2f)."
            )
            % (
                self._map_path,
                self._initial_x,
                self._initial_y,
                self._initial_z,
                self._initial_roll,
                self._initial_pitch,
                self._initial_yaw,
            )
        )

        self._client = self.create_client(Relocalize, "/localizer/relocalize")
        self._future: Optional[Future] = None
        self._start_time = self.get_clock().now()
        self._last_wait_log = self.get_clock().now()

        self._timer = self.create_timer(0.5, self._on_timer)

    def _on_timer(self) -> None:
        # Handle result if we already fired off a request.
        if self._future:
            if self._future.done():
                try:
                    response = self._future.result()
                except Exception as exc:  # pylint: disable=broad-except
                    self.get_logger().error(f"Relocalization service call failed: {exc}")
                else:
                    if response.success:
                        self.get_logger().info(f"Relocalization succeeded: {response.message}")
                    else:
                        self.get_logger().error(f"Relocalization failed: {response.message}")
                self._shutdown_soon()
            return

        # Abort if we timed out waiting for the service.
        if self.get_clock().now() - self._start_time > self._timeout:
            timeout_sec = self._timeout.nanoseconds / 1e9
            self.get_logger().error(
                f"Timed out waiting for /localizer/relocalize after {timeout_sec:.1f}s, giving up."
            )
            self._shutdown_soon()
            return

        if not self._client.wait_for_service(timeout_sec=0.0):
            if (self.get_clock().now() - self._last_wait_log).nanoseconds > 3_000_000_000:
                self.get_logger().info("Waiting for /localizer/relocalize service...")
                self._last_wait_log = self.get_clock().now()
            return

        request = Relocalize.Request()
        request.pcd_path = self._map_path
        request.x = float(self._initial_x)
        request.y = float(self._initial_y)
        request.z = float(self._initial_z)
        request.yaw = float(self._initial_yaw)
        request.pitch = float(self._initial_pitch)
        request.roll = float(self._initial_roll)

        self.get_logger().info(f"Calling /localizer/relocalize with map '{self._map_path}'.")
        self._future = self._client.call_async(request)

    def _shutdown_soon(self) -> None:
        if hasattr(self, "_timer"):
            self._timer.cancel()
        self.destroy_node()
        rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = FastLioAutoRelocalize()
    try:
        rclpy.spin(node)
    except RuntimeError:
        # Node already destroyed (e.g., map path missing); nothing else to do.
        pass


if __name__ == "__main__":
    main()
