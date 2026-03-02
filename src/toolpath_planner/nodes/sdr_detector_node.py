#!/usr/bin/env python3
"""RTL-SDR radio traffic detector for aviation CTAF monitoring.

Tunes an RTL-SDR USB dongle to the configured CTAF frequency (default 122.8 MHz),
monitors for AM voice transmissions via squelch detection, and publishes radio
activity status. When a transmission is detected, e_stop is asserted to lock
twist_mux and halt the mower immediately.

Publications:
    /sdr/radio_active (std_msgs/Bool): True when transmission detected (5 Hz)
    /sdr/detection (moxl/msg/RadioDetection): Detailed detection info (1 Hz)
    /e_stop (std_msgs/Bool): True when radio active — locks twist_mux

Parameters:
    frequency_mhz (float): CTAF frequency to monitor (default 122.8)
    sample_rate_hz (int): RTL-SDR sample rate (default 250000)
    squelch_threshold_db (float): dB above noise floor to trigger (default 10.0)
    holdoff_sec (float): Stay active after last detection (default 5.0)
    quiet_period_sec (float): Seconds quiet before mission can resume (default 300.0)
    enabled (bool): Set false when no SDR hardware (default true)
    device_index (int): RTL-SDR USB device index (default 0)
"""

import math
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from moxl.msg import RadioDetection


class SdrDetectorNode(Node):
    def __init__(self):
        super().__init__("sdr_detector_node")

        # Parameters
        self.declare_parameter("frequency_mhz", 122.8)
        self.declare_parameter("sample_rate_hz", 250000)
        self.declare_parameter("squelch_threshold_db", 10.0)
        self.declare_parameter("holdoff_sec", 5.0)
        self.declare_parameter("quiet_period_sec", 300.0)
        self.declare_parameter("enabled", True)
        self.declare_parameter("device_index", 0)

        self._frequency_mhz = self.get_parameter("frequency_mhz").value
        self._sample_rate = self.get_parameter("sample_rate_hz").value
        self._squelch_threshold_db = self.get_parameter("squelch_threshold_db").value
        self._holdoff_sec = self.get_parameter("holdoff_sec").value
        self._quiet_period_sec = self.get_parameter("quiet_period_sec").value
        self._enabled = self.get_parameter("enabled").value
        self._device_index = self.get_parameter("device_index").value

        # State
        self._lock = threading.Lock()
        self._raw_detection = False  # instantaneous power above squelch
        self._active = False  # includes holdoff
        self._last_detection_time = 0.0  # monotonic time of last raw detection
        self._last_quiet_start = time.monotonic()  # when active went False
        self._signal_power_dbm = -120.0
        self._noise_floor_db = -100.0  # rolling estimate
        self._sdr_thread = None
        self._shutdown = False

        # Publishers
        self._radio_active_pub = self.create_publisher(Bool, "/sdr/radio_active", 10)
        self._detection_pub = self.create_publisher(
            RadioDetection, "/sdr/detection", 10
        )
        self._estop_pub = self.create_publisher(Bool, "/e_stop", 10)

        # Timers
        self.create_timer(0.2, self._publish_radio_active)  # 5 Hz
        self.create_timer(1.0, self._publish_detection)  # 1 Hz

        if not self._enabled:
            self.get_logger().warn(
                "SDR detector disabled (enabled=false) — "
                "radio_active will always be False"
            )
        else:
            self._start_sdr_thread()

        self.get_logger().info(
            f"SDR detector ready: {self._frequency_mhz} MHz, "
            f"enabled={self._enabled}"
        )

    def _start_sdr_thread(self):
        """Start the background SDR sampling thread."""
        self._sdr_thread = threading.Thread(
            target=self._sdr_loop, daemon=True, name="sdr_sampler"
        )
        self._sdr_thread.start()

    def _sdr_loop(self):
        """Background thread: read IQ samples and compute signal power."""
        try:
            from rtlsdr import RtlSdr
        except ImportError:
            self.get_logger().error(
                "pyrtlsdr not installed — run: pip install pyrtlsdr. "
                "SDR detector will publish radio_active=False."
            )
            return

        sdr = None
        try:
            sdr = RtlSdr(device_index=self._device_index)
            sdr.sample_rate = self._sample_rate
            sdr.center_freq = self._frequency_mhz * 1e6
            sdr.gain = "auto"

            self.get_logger().info(
                f"RTL-SDR opened: {sdr.center_freq/1e6:.3f} MHz, "
                f"{sdr.sample_rate/1e3:.0f} kHz sample rate"
            )

            num_samples = 256 * 1024  # ~1 second of samples at 250 kHz
            # Exponential moving average for noise floor
            alpha = 0.01

            while not self._shutdown:
                try:
                    samples = sdr.read_samples(num_samples)
                except Exception as e:
                    self.get_logger().error(f"SDR read error: {e}")
                    time.sleep(1.0)
                    continue

                # Compute power: 10*log10(mean(|IQ|^2))
                power = np.mean(np.abs(samples) ** 2)
                if power > 0:
                    power_db = 10.0 * math.log10(power)
                else:
                    power_db = -120.0

                with self._lock:
                    # Update noise floor with slow EMA (only when not detecting)
                    if not self._raw_detection:
                        self._noise_floor_db = (
                            alpha * power_db + (1 - alpha) * self._noise_floor_db
                        )

                    self._signal_power_dbm = power_db

                    # Squelch: signal above noise floor + threshold
                    above_squelch = (
                        power_db > self._noise_floor_db + self._squelch_threshold_db
                    )

                    now = time.monotonic()

                    if above_squelch:
                        self._raw_detection = True
                        self._last_detection_time = now
                        if not self._active:
                            self._active = True
                            self.get_logger().warn(
                                f"Radio transmission detected on "
                                f"{self._frequency_mhz} MHz "
                                f"(power={power_db:.1f} dB, "
                                f"floor={self._noise_floor_db:.1f} dB)"
                            )
                    else:
                        self._raw_detection = False
                        # Check holdoff
                        if self._active:
                            elapsed = now - self._last_detection_time
                            if elapsed >= self._holdoff_sec:
                                self._active = False
                                self._last_quiet_start = now
                                self.get_logger().info(
                                    "Radio transmission ended — quiet timer started"
                                )

        except Exception as e:
            self.get_logger().error(
                f"SDR thread failed: {e}. "
                "Detector will publish radio_active=False (fail-open)."
            )
        finally:
            if sdr is not None:
                try:
                    sdr.close()
                except Exception:
                    pass

    def _publish_radio_active(self):
        """Publish radio_active Bool and e_stop at 5 Hz."""
        with self._lock:
            active = self._active

        msg = Bool(data=active)
        self._radio_active_pub.publish(msg)
        self._estop_pub.publish(msg)

    def _publish_detection(self):
        """Publish detailed RadioDetection at 1 Hz."""
        with self._lock:
            active = self._active
            power = self._signal_power_dbm
            if active:
                quiet = 0.0
            else:
                quiet = time.monotonic() - self._last_quiet_start

        msg = RadioDetection()
        msg.stamp = self.get_clock().now().to_msg()
        msg.frequency_mhz = self._frequency_mhz
        msg.signal_power_dbm = power
        msg.active = active
        msg.quiet_duration_sec = quiet
        self._detection_pub.publish(msg)

    def destroy_node(self):
        self._shutdown = True
        if self._sdr_thread is not None:
            self._sdr_thread.join(timeout=3.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SdrDetectorNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
