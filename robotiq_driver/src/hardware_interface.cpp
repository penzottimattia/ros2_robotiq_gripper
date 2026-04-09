// Copyright (c) 2022 PickNik, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <robotiq_driver/default_driver_factory.hpp>
#include <robotiq_driver/hardware_interface.hpp>

#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <rclcpp/rclcpp.hpp>

const auto kLogger = rclcpp::get_logger("RobotiqGripperHardwareInterface");

constexpr uint8_t kGripperMinPos = 3;
constexpr uint8_t kGripperMaxPos = 230;
constexpr double kGripperMaxSpeed = 0.150;  // mm/s
constexpr double kGripperMaxforce = 235;    // N
constexpr uint8_t kGripperRange = kGripperMaxPos - kGripperMinPos;

constexpr auto kGripperCommsLoopPeriod = std::chrono::milliseconds{ 10 };

namespace robotiq_driver
{
RobotiqGripperHardwareInterface::RobotiqGripperHardwareInterface()
{
  driver_factory_ = std::make_unique<DefaultDriverFactory>();

  // Initialize communication flags
  communication_thread_is_running_.store(false);
  reactivate_gripper_async_cmd_.store(false);
  reactivate_gripper_async_response_.store(std::nullopt);

  // Initialize async activation/deactivation flags
  activate_async_cmd_.store(false);
  activate_async_response_.store(std::nullopt);
  deactivate_async_cmd_.store(false);
  deactivate_async_response_.store(std::nullopt);
}

RobotiqGripperHardwareInterface::~RobotiqGripperHardwareInterface()
{
  communication_thread_is_running_.store(false);
  if (communication_thread_.joinable())
  {
    communication_thread_.join();
  }
}

// This constructor is use for testing only.
RobotiqGripperHardwareInterface::RobotiqGripperHardwareInterface(std::unique_ptr<DriverFactory> driver_factory)
  : driver_factory_{ std::move(driver_factory) }
{
}

hardware_interface::CallbackReturn
RobotiqGripperHardwareInterface::on_init(const hardware_interface::HardwareInfo& info)
{
  RCLCPP_DEBUG(kLogger, "on_init");

  // Call base class on_init which expects a HardwareInfo
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Read parameters.
  gripper_closed_pos_ = stod(info_.hardware_parameters.at("gripper_closed_position"));
  gripper_max_speed_ = info_.hardware_parameters.count("gripper_max_speed") ?
                           stod(info_.hardware_parameters.at("gripper_max_speed")) :
                           kGripperMaxSpeed;
  gripper_max_force_ = info_.hardware_parameters.count("gripper_max_force") ?
                           stod(info_.hardware_parameters.at("gripper_max_force")) :
                           kGripperMaxforce;
  gripper_position_ = std::numeric_limits<double>::quiet_NaN();
  gripper_velocity_ = std::numeric_limits<double>::quiet_NaN();
  gripper_position_command_ = std::numeric_limits<double>::quiet_NaN();
  reactivate_gripper_cmd_ = NO_NEW_CMD_;
  reactivate_gripper_async_cmd_.store(false);

  const hardware_interface::ComponentInfo& joint = info_.joints.at(0);

  // There is one command interface: position.
  if (joint.command_interfaces.size() != 1)
  {
    RCLCPP_FATAL(kLogger, "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                 joint.command_interfaces.size());
    return CallbackReturn::ERROR;
  }

  if (joint.command_interfaces.at(0).name != hardware_interface::HW_IF_POSITION)
  {
    RCLCPP_FATAL(kLogger, "Joint '%s' has %s command interfaces found. '%s' expected.", joint.name.c_str(),
                 joint.command_interfaces.at(0).name.c_str(), hardware_interface::HW_IF_POSITION);
    return CallbackReturn::ERROR;
  }

  // There are two state interfaces: position and velocity.
  if (joint.state_interfaces.size() != 2)
  {
    RCLCPP_FATAL(kLogger, "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                 joint.state_interfaces.size());
    return CallbackReturn::ERROR;
  }

  for (int i = 0; i < 2; ++i)
  {
    if (!(joint.state_interfaces[i].name == hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[i].name == hardware_interface::HW_IF_VELOCITY))
    {
      RCLCPP_FATAL(kLogger, "Joint '%s' has %s state interface. Expected %s or %s.", joint.name.c_str(),
                   joint.state_interfaces.at(i).name.c_str(), hardware_interface::HW_IF_POSITION,
                   hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
  }

  try
  {
    driver_ = driver_factory_->create(info_);
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(kLogger, "Failed to create a driver: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotiqGripperHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_DEBUG(kLogger, "on_configure");
  try
  {
    if (hardware_interface::SystemInterface::on_configure(previous_state) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    // Open the serial port and handshake.
    bool connected = driver_->connect();
    if (!connected)
    {
      RCLCPP_ERROR(kLogger, "Cannot connect to the Robotiq gripper");
      return CallbackReturn::ERROR;
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(kLogger, "Cannot configure the Robotiq gripper: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotiqGripperHardwareInterface::export_state_interfaces()
{
  RCLCPP_DEBUG(kLogger, "export_state_interfaces");

  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(
      hardware_interface::StateInterface(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &gripper_position_));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &gripper_velocity_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotiqGripperHardwareInterface::export_command_interfaces()
{
  RCLCPP_DEBUG(kLogger, "export_command_interfaces");

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[0].name, hardware_interface::HW_IF_POSITION, &gripper_position_command_));

  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(info_.joints[0].name, "set_gripper_max_velocity", &gripper_speed_));
  gripper_speed_ = kGripperMaxSpeed * (info_.hardware_parameters.count("gripper_speed_multiplier") ?
                                           std::stod(info_.hardware_parameters.at("gripper_speed_multiplier")) :
                                           1.0);

  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(info_.joints[0].name, "set_gripper_max_effort", &gripper_force_));
  gripper_force_ = kGripperMaxforce * (info_.hardware_parameters.count("gripper_force_multiplier") ?
                                           std::stod(info_.hardware_parameters.at("gripper_force_multiplier")) :
                                           1.0);

  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(info_.gpios[0].name, "reactivate_gripper_cmd", &reactivate_gripper_cmd_));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.gpios[0].name, "reactivate_gripper_response", &reactivate_gripper_response_));

  return command_interfaces;
}

hardware_interface::CallbackReturn
RobotiqGripperHardwareInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_DEBUG(kLogger, "on_activate");

  // set some default values for joints
  if (std::isnan(gripper_position_))
  {
    gripper_position_ = 0;
    gripper_velocity_ = 0;
    gripper_position_command_ = 0;
  }

  // Activate the gripper asynchronously to avoid blocking the lifecycle thread.
  try
  {
    // Start communication thread first so it can process the async activation request.
    communication_thread_is_running_.store(true);
    communication_thread_ = std::thread([this] { this->background_task(); });

    // Request activation in the background thread.
    activate_async_response_.store(std::nullopt);
    activate_async_cmd_.store(true);
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(kLogger, "Failed to communicate with the Robotiq gripper: %s", e.what());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(kLogger, "Robotiq Gripper activation requested (background).");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
RobotiqGripperHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_DEBUG(kLogger, "on_deactivate");

  // Request deactivation to be performed in the background thread so we don't
  // block the lifecycle thread calling driver_->deactivate() directly.
  deactivate_async_response_.store(std::nullopt);
  deactivate_async_cmd_.store(true);

  // Wait for the background thread to complete the deactivation up to a timeout.
  const auto timeout = std::chrono::seconds{5};
  const auto start = std::chrono::steady_clock::now();
  while (!deactivate_async_response_.load().has_value())
  {
    if (std::chrono::steady_clock::now() - start > timeout)
    {
      RCLCPP_WARN(kLogger, "Timeout waiting for background deactivation; proceeding to stop communication thread.");
      break;
    }
    // Sleep a bit to avoid busy waiting.
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  // Stop the communication thread and join it.
  communication_thread_is_running_.store(false);
  if (communication_thread_.joinable())
  {
    communication_thread_.join();
  }

  try
  {
    if (deactivate_async_response_.load().has_value() && deactivate_async_response_.load().value())
    {
      RCLCPP_INFO(kLogger, "Robotiq Gripper successfully deactivated (background).");
      return CallbackReturn::SUCCESS;
    }
    else
    {
      // If background deactivation failed or timed out, try a final cleanup call
      // to ensure driver is deactivated. Do it in a try/catch to avoid throwing.
      try
      {
        driver_->deactivate();
      }
      catch (const std::exception& e)
      {
        RCLCPP_ERROR(kLogger, "Failed to deactivate the Robotiq gripper: %s", e.what());
        return CallbackReturn::ERROR;
      }
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(kLogger, "Failed during deactivation cleanup: %s", e.what());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(kLogger, "Robotiq Gripper successfully deactivated!");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobotiqGripperHardwareInterface::read(const rclcpp::Time& /*time*/,
                                                                      const rclcpp::Duration& /*period*/)
{
  gripper_position_ = gripper_closed_pos_ * (gripper_current_state_.load() - kGripperMinPos) / kGripperRange;

  if (!std::isnan(reactivate_gripper_cmd_))
  {
    RCLCPP_INFO(kLogger, "Sending gripper reactivation request.");
    reactivate_gripper_async_cmd_.store(true);
    reactivate_gripper_cmd_ = NO_NEW_CMD_;
  }

  if (reactivate_gripper_async_response_.load().has_value())
  {
    reactivate_gripper_response_ = reactivate_gripper_async_response_.load().value();
    reactivate_gripper_async_response_.store(std::nullopt);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotiqGripperHardwareInterface::write(const rclcpp::Time& /*time*/,
                                                                       const rclcpp::Duration& /*period*/)
{
  double gripper_pos = (gripper_position_command_ / gripper_closed_pos_) * kGripperRange + kGripperMinPos;
  gripper_pos = std::max(std::min(gripper_pos, 255.0), 0.0);
  write_command_.store(uint8_t(gripper_pos));
  const auto gripper_speed_multiplier = std::clamp(fabs(gripper_speed_) / gripper_max_speed_, 0.0, 1.0);
  write_speed_.store(uint8_t(gripper_speed_multiplier * 0xFF));
  const auto gripper_force_multiplier = std::clamp(fabs(gripper_force_) / gripper_max_force_, 0.0, 1.0);
  write_force_.store(uint8_t(gripper_force_multiplier * 0xFF));

  return hardware_interface::return_type::OK;
}

void RobotiqGripperHardwareInterface::background_task()
{
  while (communication_thread_is_running_.load())
  {
    try
    {
      // Process async activation request
      if (activate_async_cmd_.load())
      {
        try
        {
          this->driver_->deactivate();
          this->driver_->connect();
          this->driver_->set_speed(write_speed_.load());
          this->driver_->set_force(write_force_.load());
          this->driver_->activate();
          activate_async_response_.store(std::optional<bool>(true));
          RCLCPP_INFO(kLogger, "Robotiq Gripper activated (background).");
        }
        catch (const std::exception& e)
        {
          activate_async_response_.store(std::optional<bool>(false));
          RCLCPP_ERROR(kLogger, "Background activation failed: %s", e.what());
        }
        activate_async_cmd_.store(false);
      }

      // Process async deactivation request
      if (deactivate_async_cmd_.load())
      {
        try
        {
          this->driver_->deactivate();
          deactivate_async_response_.store(std::optional<bool>(true));
          RCLCPP_INFO(kLogger, "Robotiq Gripper deactivated (background).");
        }
        catch (const std::exception& e)
        {
          deactivate_async_response_.store(std::optional<bool>(false));
          RCLCPP_ERROR(kLogger, "Background deactivation failed: %s", e.what());
        }
        deactivate_async_cmd_.store(false);
      }

      // Re-activate the gripper
      // (this can be used, for example, to re-run the auto-calibration).
      if (reactivate_gripper_async_cmd_.load())
      {
        this->driver_->deactivate();
        this->driver_->activate();
        reactivate_gripper_async_cmd_.store(false);
        reactivate_gripper_async_response_.store(true);
      }

      // Write the latest command to the gripper.
      this->driver_->set_gripper_position(write_command_.load());
      this->driver_->set_speed(write_speed_.load());
      this->driver_->set_force(write_force_.load());

      // Read the state of the gripper.
      gripper_current_state_.store(this->driver_->get_gripper_position());
    }
    catch (std::exception& e)
    {
      RCLCPP_ERROR(kLogger, "Error: %s", e.what());
    }

    std::this_thread::sleep_for(kGripperCommsLoopPeriod);
  }
}

}  // namespace robotiq_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(robotiq_driver::RobotiqGripperHardwareInterface, hardware_interface::SystemInterface)
