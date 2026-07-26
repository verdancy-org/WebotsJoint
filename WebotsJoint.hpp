#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <string_view>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>

#include "libxr.hpp"

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Webots joint wrapper for a motor, optional position sensor, and joint command state
constructor_args: []
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

namespace zeq::hardware
{

class WebotsJoint
{
 public:
  WebotsJoint(std::string_view name, double torque_limit)
      : name_(name), torque_limit_(torque_limit)
  {
  }

  WebotsJoint(std::string_view name, float fixed_position)
      : name_(name), fixed_(true), position_(fixed_position)
  {
  }

  bool UpdateStatus(webots::Robot* robot, int sensor_period_ms, uint64_t now_us,
                    std::string& error)
  {
    if (fixed_)
    {
      velocity_ = 0.0F;
      effort_ = 0.0F;
      return true;
    }

    if (!EnsureDevice(robot, sensor_period_ms, now_us, error))
    {
      return false;
    }

    const float position = static_cast<float>(sensor_->getValue());
    const uint64_t delta_us = now_us > last_sample_us_ ? now_us - last_sample_us_ : 0;
    if (delta_us > 0)
    {
      velocity_ =
          (position - previous_position_) / (static_cast<float>(delta_us) * 1.0e-6F);
    }

    position_ = position;
    previous_position_ = position;
    last_sample_us_ = now_us;
    return true;
  }

  bool CommandVelocity(webots::Robot* robot, int sensor_period_ms, float value,
                       std::string& error)
  {
    if (fixed_)
    {
      return true;
    }

    if (!std::isfinite(value))
    {
      return true;
    }

    if (!EnsureDevice(robot, sensor_period_ms, NowUs(), error))
    {
      return false;
    }

    const double max_velocity = motor_->getMaxVelocity();
    const double velocity =
        std::isfinite(max_velocity) && max_velocity > 0.0
            ? std::clamp(static_cast<double>(value), -max_velocity, max_velocity)
            : static_cast<double>(value);
    motor_->setPosition(std::numeric_limits<double>::infinity());
    motor_->setAvailableTorque(torque_limit_);
    motor_->setVelocity(velocity);
    return true;
  }

  bool CommandEffort(webots::Robot* robot, int sensor_period_ms, float value,
                     std::string& error)
  {
    if (fixed_)
    {
      return true;
    }

    if (!std::isfinite(value))
    {
      return true;
    }

    if (!EnsureDevice(robot, sensor_period_ms, NowUs(), error))
    {
      return false;
    }

    const double torque =
        std::clamp(static_cast<double>(value), -torque_limit_, torque_limit_);
    motor_->setTorque(torque);
    effort_ = static_cast<float>(torque);
    return true;
  }

  [[nodiscard]] float Angle() const { return position_; }
  [[nodiscard]] float Velocity() const { return velocity_; }

 private:
  static uint64_t NowUs()
  {
    return static_cast<uint64_t>(LibXR::Timebase::GetMicroseconds());
  }

  static std::string SensorName(std::string_view motor_name)
  {
    std::string name(motor_name);
    name += "_sensor";
    return name;
  }

  bool EnsureDevice(webots::Robot* robot, int sensor_period_ms, uint64_t sample_us,
                    std::string& error)
  {
    if (robot == nullptr)
    {
      error = "Webots robot handle is not initialized";
      return false;
    }

    if (motor_ == nullptr)
    {
      motor_ = robot->getMotor(name_);
      if (motor_ == nullptr)
      {
        error = "missing Webots motor device: " + name_;
        return false;
      }

      motor_->setPosition(std::numeric_limits<double>::infinity());
      motor_->setVelocity(0.0);
      motor_->setAvailableTorque(torque_limit_);
    }

    if (sensor_ == nullptr)
    {
      const std::string position_sensor_name = SensorName(name_);
      sensor_ = robot->getPositionSensor(position_sensor_name);
      if (sensor_ == nullptr)
      {
        error = "missing Webots position sensor device: " + position_sensor_name;
        return false;
      }

      sensor_->enable(sensor_period_ms);
      position_ = static_cast<float>(sensor_->getValue());
      previous_position_ = position_;
      last_sample_us_ = sample_us;
    }

    return true;
  }

  std::string name_;
  double torque_limit_ = 0.0;
  bool fixed_ = false;
  webots::Motor* motor_ = nullptr;
  webots::PositionSensor* sensor_ = nullptr;
  float position_ = 0.0F;
  float previous_position_ = 0.0F;
  float velocity_ = 0.0F;
  float effort_ = 0.0F;
  uint64_t last_sample_us_ = 0;
};

}  // namespace zeq::hardware
