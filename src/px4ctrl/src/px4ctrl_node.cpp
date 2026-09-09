#include <chrono>
#include <csignal>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "PX4CtrlFSM.h"
#include "PX4CtrlParam.h"
#include "controller.h"
#include "input.h"

namespace
{
void sigint_handler(int)
{
  RCLCPP_INFO(rclcpp::get_logger("px4ctrl"), "[PX4Ctrl] exit...");
  rclcpp::shutdown();
}
}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("px4ctrl");

  std::signal(SIGINT, sigint_handler);
  rclcpp::sleep_for(std::chrono::seconds(1));

  Parameter_t param;
  param.config_from_ros_handle(node);

  LinearControl controller(param, node);
  PX4CtrlFSM fsm(param, controller, node);

  auto state_sub = node->create_subscription<mavros_msgs::msg::State>(
    "/mavros/state", 10,
    std::bind(&State_Data_t::feed, &fsm.state_data, std::placeholders::_1));

  auto extended_state_sub = node->create_subscription<mavros_msgs::msg::ExtendedState>(
    "/mavros/extended_state", 10,
    std::bind(&ExtendedState_Data_t::feed, &fsm.extended_state_data, std::placeholders::_1));

  auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
    "odom", rclcpp::SensorDataQoS(),
    std::bind(&Odom_Data_t::feed, &fsm.odom_data, std::placeholders::_1));

  auto cmd_sub = node->create_subscription<quadrotor_msgs::msg::PositionCommand>(
    "cmd", rclcpp::QoS(100),
    std::bind(&Command_Data_t::feed, &fsm.cmd_data, std::placeholders::_1));

  auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
    "/mavros/imu/data", rclcpp::SensorDataQoS(),
    std::bind(&Imu_Data_t::feed, &fsm.imu_data, std::placeholders::_1));

  rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_sub;
  if (!param.takeoff_land.no_RC) {
    rc_sub = node->create_subscription<mavros_msgs::msg::RCIn>(
      "/mavros/rc/in", 10,
      std::bind(&RC_Data_t::feed, &fsm.rc_data, std::placeholders::_1));
  }

  auto bat_sub = node->create_subscription<sensor_msgs::msg::BatteryState>(
    "/mavros/battery", rclcpp::SensorDataQoS(),
    std::bind(&Battery_Data_t::feed, &fsm.bat_data, std::placeholders::_1));

  auto takeoff_land_sub = node->create_subscription<quadrotor_msgs::msg::TakeoffLand>(
    "takeoff_land", rclcpp::QoS(100),
    std::bind(&Takeoff_Land_Data_t::feed, &fsm.takeoff_land_data, std::placeholders::_1));

  fsm.ctrl_FCU_pub = node->create_publisher<mavros_msgs::msg::AttitudeTarget>(
    "/mavros/setpoint_raw/attitude", rclcpp::SensorDataQoS());
  fsm.traj_start_trigger_pub =
    node->create_publisher<geometry_msgs::msg::PoseStamped>("/traj_start_trigger", 10);
  fsm.debug_pub =
    node->create_publisher<quadrotor_msgs::msg::Px4ctrlDebug>("/debugPx4ctrl", 10);

  fsm.set_FCU_mode_client =
    node->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
  fsm.arming_client =
    node->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
  fsm.reboot_FCU_client =
    node->create_client<mavros_msgs::srv::CommandLong>("/mavros/cmd/command");

  if (param.takeoff_land.no_RC) {
    RCLCPP_WARN(node->get_logger(), "[PX4CTRL] Remote controller disabled, be careful!");
  } else {
    RCLCPP_INFO(node->get_logger(), "[PX4CTRL] Waiting for RC");
    while (rclcpp::ok()) {
      rclcpp::spin_some(node);
      if (fsm.rc_is_received(node->now())) {
        RCLCPP_INFO(node->get_logger(), "[PX4CTRL] RC received.");
        break;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
  }

  int trials = 0;
  while (rclcpp::ok() && !fsm.state_data.current_state.connected) {
    rclcpp::spin_some(node);
    rclcpp::sleep_for(std::chrono::seconds(1));
    if (trials++ > 5) {
      RCLCPP_ERROR(node->get_logger(), "Unable to connect to PX4!!!");
    }
  }

  rclcpp::Rate rate(param.ctrl_freq_max);
  while (rclcpp::ok()) {
    rate.sleep();
    rclcpp::spin_some(node);
    fsm.process();
  }

  return 0;
}
