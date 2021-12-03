//
// Created by yezi on 2021/11/30.
//

#pragma once

#include <cassert>
#include <vector>
#include <transmission_interface/transmission.h>
#include <transmission_interface/transmission_interface_exception.h>

namespace transmission_interface
{
class DoubleJointTransmission : public Transmission
{
public:
  /** \brief Check whether it has two joints and one actuator, throw error if not. Check whether Transmission reduction
   * ratios are zero, throw error if true.
   *
   * @param actuator_reduction Actuator's reduction.
   * @param joint_reduction Joint's reduction.
   * @param joint_offset Joint's offset.
   */
  DoubleJointTransmission(double actuator_reduction, std::vector<double> joint_reduction,
                          std::vector<double> joint_offset = { 0.0, 0.0 });
  /** \brief Set conversion from actuator to joint on effort.
   *
   * @param act_data Data of actuator.
   * @param jnt_data Data of joint.
   */
  void actuatorToJointEffort(const ActuatorData& act_data, JointData& jnt_data) override;
  /** \brief Set conversion from actuator to joint on velocity.
   *
   * @param act_data Data of actuator.
   * @param jnt_data Data of joint.
   */
  void actuatorToJointVelocity(const ActuatorData& act_data, JointData& jnt_data) override;
  /** \brief Set conversion from actuator to joint on position.
   *
   * @param act_data Data of actuator.
   * @param jnt_data Data of joint.
   */
  void actuatorToJointPosition(const ActuatorData& act_data, JointData& jnt_data) override;
  /** \brief Set conversion from joint to actuator on effort.
   *
   * @param act_data Data of actuator.
   * @param jnt_data Data of joint.
   */
  void jointToActuatorEffort(const JointData& jnt_data, ActuatorData& act_data) override;
  void jointToActuatorVelocity(const JointData& jnt_data, ActuatorData& act_data) override{};
  void jointToActuatorPosition(const JointData& jnt_data, ActuatorData& act_data) override{};

  std::size_t numActuators() const override
  {
    return 1;
  }
  std::size_t numJoints() const override
  {
    return 2;
  }

  const double getActuatorReduction() const
  {
    return act_reduction_;
  }
  const std::vector<double> getJointReduction() const
  {
    return jnt_reduction_;
  }
  const std::vector<double> getJointOffset()
  {
    return joint_offset_;
  }

protected:
  double act_reduction_;
  std::vector<double> jnt_reduction_;
  std::vector<double> joint_offset_;
};

DoubleJointTransmission::DoubleJointTransmission(double actuator_reduction, std::vector<double> joint_reduction,
                                                 std::vector<double> joint_offset)
  : act_reduction_(actuator_reduction)
  , jnt_reduction_(std::move(joint_reduction))
  , joint_offset_(std::move(joint_offset))
{
  if (numActuators() != 1 || jnt_reduction_.size() != numJoints())
  {
    throw TransmissionInterfaceException(
        "Joint reduction and offset vectors of a double transmission must have size 1, actuator must size 2");
  }
  if (0.0 == act_reduction_ || 0.0 == jnt_reduction_[0] || 0.0 == jnt_reduction_[1])
  {
    throw TransmissionInterfaceException("Transmission reduction ratios cannot be zero.");
  }
}

void DoubleJointTransmission::actuatorToJointEffort(const ActuatorData& act_data, JointData& jnt_data)
{
  assert(numActuators() == act_data.effort.size() && numJoints() == jnt_data.effort.size());
  assert(act_data.effort[0] && jnt_data.effort[0] && jnt_data.effort[1]);

  *jnt_data.effort[0] = *act_data.effort[0] * jnt_reduction_[0] * act_reduction_;
  *jnt_data.effort[1] = *act_data.effort[0] * jnt_reduction_[1] * act_reduction_;
}

void DoubleJointTransmission::actuatorToJointPosition(const ActuatorData& act_data, JointData& jnt_data)
{
  assert(numActuators() == act_data.position.size() && numJoints() == jnt_data.position.size());
  assert(act_data.position[0] && jnt_data.position[0] && jnt_data.position[1]);

  *jnt_data.position[0] = *act_data.position[0] / jnt_reduction_[0] / act_reduction_;
  *jnt_data.position[1] = *act_data.position[0] / jnt_reduction_[1] / act_reduction_;
}

void DoubleJointTransmission::actuatorToJointVelocity(const ActuatorData& act_data, JointData& jnt_data)
{
  assert(numActuators() == act_data.velocity.size() && numJoints() == jnt_data.velocity.size());
  assert(act_data.velocity[0] && jnt_data.velocity[0] && jnt_data.velocity[1]);

  *jnt_data.velocity[0] = *act_data.velocity[0] / jnt_reduction_[0] / act_reduction_;
  *jnt_data.velocity[1] = *act_data.velocity[0] / jnt_reduction_[1] / act_reduction_;
}

void DoubleJointTransmission::jointToActuatorEffort(const JointData& jnt_data, ActuatorData& act_data)
{
  assert(numActuators() == act_data.effort.size() && numJoints() == jnt_data.effort.size());
  assert(act_data.effort[0] && jnt_data.effort[0] && jnt_data.effort[1]);

  *act_data.effort[0] = *jnt_data.effort[0] / jnt_reduction_[0] / act_reduction_;
}

}  // namespace transmission_interface
