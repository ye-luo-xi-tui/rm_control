//
// Created by yezi on 2021/11/30.
//

#pragma once

#include <rm_hw/transmission/double_joint_transmission.h>
#include <transmission_interface/transmission_loader.h>

namespace transmission_interface
{
class DoubleJointTransmissionLoader : public TransmissionLoader
{
public:
  TransmissionSharedPtr load(const TransmissionInfo& transmission_info) override;

private:
  static bool getActuatorConfig(const TransmissionInfo& transmission_info, double& actuator_reduction);
  static bool getJointConfig(const TransmissionInfo& transmission_info, std::vector<double>& joint_reduction,
                             std::vector<double>& joint_offset);
};
}  // namespace transmission_interface
