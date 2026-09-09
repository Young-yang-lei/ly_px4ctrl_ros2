#!/usr/bin/env python3

import csv
import time
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import AttitudeTarget
from rclpy.node import Node
from sensor_msgs.msg import BatteryState


class ThrustCalibration(Node):
    def __init__(self):
        super().__init__("thrust_calibration")

        self.time_average_interval = float(
            self.declare_parameter("time_interval", 1.0).value
        )
        self.vbat_min = float(
            self.declare_parameter("min_battery_voltage", 13.2).value
        )
        self.mass_kg = float(self.declare_parameter("mass_kg", 1.0).value)

        self.volt_buf = []
        self.volt_records = []
        self.cmd_buf = []
        self.cmd_records = []
        self.last_records_t = self.get_clock().now()
        self.count_start_trigger_received = 0
        self.stopped = False

        self.bat_sub = self.create_subscription(
            BatteryState, "/mavros/battery", self.battery_voltage_cb, 10
        )
        self.create_subscription(
            AttitudeTarget,
            "/mavros/setpoint_raw/attitude",
            self.thrust_commands_cb,
            10,
        )
        self.create_subscription(
            PoseStamped, "/traj_start_trigger", self.start_trigger_cb, 10
        )

    def battery_voltage_cb(self, msg):
        if self.stopped or self.count_start_trigger_received == 0:
            return

        self.volt_buf.append(float(msg.voltage))
        current_time = self.get_clock().now()
        if (current_time - self.last_records_t).nanoseconds / 1e9 > self.time_average_interval:
            self.last_records_t = current_time
            if self.volt_buf and self.cmd_buf:
                self.volt_records.append(float(np.mean(self.volt_buf)))
                self.cmd_records.append(float(np.mean(self.cmd_buf)))
                self.volt_buf.clear()
                self.cmd_buf.clear()
                self.get_logger().info(
                    f"volt={self.volt_records[-1]:.3f} "
                    f"thr={self.cmd_records[-1]:.3f}"
                )

        if self.count_start_trigger_received >= 2 or (
            self.volt_records and self.volt_records[-1] < self.vbat_min
        ):
            self.cal_and_save_data()
            self.stopped = True
            self.destroy_subscription(self.bat_sub)

    def thrust_commands_cb(self, msg):
        if not self.stopped and self.count_start_trigger_received > 0:
            self.cmd_buf.append(float(msg.thrust))

    def start_trigger_cb(self, _msg):
        self.count_start_trigger_received += 1
        if self.count_start_trigger_received == 1:
            self.get_logger().info("Start recording.")
        else:
            self.get_logger().info("Stop recording.")

    def cal_and_save_data(self):
        if not self.cmd_records or not self.volt_records:
            self.get_logger().warning("No complete calibration samples to store.")
            return

        package_dir = Path(get_package_share_directory("px4ctrl"))
        file_path = package_dir / "thrust_calibrate_scripts" / "data.csv"
        file_path.parent.mkdir(parents=True, exist_ok=True)

        self.get_logger().info("Data storing.")
        with file_path.open("a", newline="") as data_file:
            writer = csv.writer(data_file)
            writer.writerow(
                (
                    time.strftime("%Y-%m-%d %H:%M:%S"),
                    "mass(kg):",
                    self.mass_kg,
                    "commands",
                    "voltage",
                )
            )
            writer.writerows(zip(self.cmd_records, self.volt_records))

        self.get_logger().info(f"Stored to {file_path}")


def main(args=None):
    rclpy.init(args=args)
    node = ThrustCalibration()
    try:
        node.get_logger().info("Waiting for trigger.")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
