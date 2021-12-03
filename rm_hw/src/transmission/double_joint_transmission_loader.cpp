//
// Created by yezi on 2021/11/30.
//

#include <rm_hw/transmission/double_joint_transmission_loader.h>
#include <rm_hw/transmission/double_joint_transmission.h>
#include <hardware_interface/internal/demangle_symbol.h>

#include <pluginlib/class_list_macros.hpp>

namespace transmission_interface
{
TransmissionSharedPtr DoubleJointTransmissionLoader::load(const TransmissionInfo& transmission_info)
{
  // Transmission should contain only one actuator/joint
  if (!checkActuatorDimension(transmission_info, 1))
  {
    return TransmissionSharedPtr();
  }
  if (!checkJointDimension(transmission_info, 2))
  {
    return TransmissionSharedPtr();
  }

  double act_reduction;
  const bool act_config_ok = getActuatorConfig(transmission_info, act_reduction);
  if (!act_config_ok)
    return TransmissionSharedPtr();

  std::vector<double> jnt_reduction;
  std::vector<double> jnt_offset;
  const bool jnt_config_ok = getJointConfig(transmission_info, jnt_reduction, jnt_offset);
  if (!jnt_config_ok)
  {
    return TransmissionSharedPtr();
  }
  // Transmission instance
  try
  {
    TransmissionSharedPtr transmission(new DoubleJointTransmission(act_reduction, jnt_reduction, jnt_offset));
    return transmission;
  }
  catch (const TransmissionInterfaceException& ex)
  {
    using hardware_interface::internal::demangledTypeName;
    ROS_ERROR_STREAM_NAMED("parser", "Failed to construct transmission '"
                                         << transmission_info.name_ << "' of type '"
                                         << demangledTypeName<DoubleJointTransmission>() << "'. " << ex.what());
    return TransmissionSharedPtr();
  }
}

bool DoubleJointTransmissionLoader::getActuatorConfig(const TransmissionInfo& transmission_info,
                                                      double& actuator_reduction)
{
  TiXmlElement actuator_el = loadXmlElement(transmission_info.actuators_.front().xml_element_);
  const bool reduction_status = getActuatorReduction(actuator_el, transmission_info.actuators_.front().name_,
                                                     transmission_info.name_, true, actuator_reduction);
  return reduction_status;
}

bool DoubleJointTransmissionLoader::getJointConfig(const TransmissionInfo& transmission_info,
                                                   std::vector<double>& joint_reduction,
                                                   std::vector<double>& joint_offset)
{
  const std::string join_t1_role = "joint1";
  const std::string join_t2_role = "joint2";

  std::vector<TiXmlElement> jnt_elements(2, "");
  std::vector<std::string> jnt_names(2);
  std::vector<std::string> jnt_roles(2);

  for (unsigned int i = 0; i < 2; ++i)
  {
    // Joint name
    jnt_names[i] = transmission_info.joints_[i].name_;

    // Joint xml element
    jnt_elements[i] = loadXmlElement(transmission_info.joints_[i].xml_element_);

    // Populate role string
    std::string& jnt_role = jnt_roles[i];
    const bool jnt_role_status = getJointRole(jnt_elements[i], jnt_names[i], transmission_info.name_,
                                              true,  // Required
                                              jnt_role);
    if (!jnt_role_status)
    {
      return false;
    }

    // Validate role string
    if (join_t1_role != jnt_role && join_t2_role != jnt_role)
    {
      ROS_ERROR_STREAM_NAMED("parser", "Joint '" << jnt_names[i] << "' of transmission '" << transmission_info.name_
                                                 << "' does not specify a valid <role> element. Got '" << jnt_role
                                                 << "', expected '" << join_t1_role << "' or '" << join_t2_role
                                                 << "'.");
      return false;
    }
  }
  // Roles must be different
  if (jnt_roles[0] == jnt_roles[1])
  {
    ROS_ERROR_STREAM_NAMED("parser", "Joints '" << jnt_names[0] << "' and '" << jnt_names[1] << "' of transmission '"
                                                << transmission_info.name_
                                                << "' must have different roles. Both specify '" << jnt_roles[0]
                                                << "'.");
    return false;
  }
  // Indices sorted according to role
  std::vector<unsigned int> id_map(2);
  if (join_t1_role == jnt_roles[0])
  {
    id_map[0] = 0;
    id_map[1] = 1;
  }
  else
  {
    id_map[0] = 1;
    id_map[1] = 0;
  }
  // Joint configuration
  joint_reduction.resize(2, 1.0);
  joint_offset.resize(2, 0.0);
  for (unsigned int i = 0; i < 2; ++i)
  {
    const unsigned int id = id_map[i];

    // Parse optional mechanical reductions. Even though it's optional --and to avoid surprises-- we fail if the element
    // is specified but is of the wrong type
    const bool reduction_status = getJointReduction(jnt_elements[id], jnt_names[id], transmission_info.name_,
                                                    false,  // Optional
                                                    joint_reduction[i]);
    if (!reduction_status)
    {
      return false;
    }

    // Parse optional joint offset. Even though it's optional --and to avoid surprises-- we fail if the element is
    // specified but is of the wrong type
    const bool offset_status = getJointOffset(jnt_elements[id], jnt_names[id], transmission_info.name_,
                                              false,  // Optional
                                              joint_offset[i]);
    if (!offset_status)
    {
      return false;
    }
  }

  return true;
}
}  // namespace transmission_interface
PLUGINLIB_EXPORT_CLASS(transmission_interface::DoubleJointTransmissionLoader, transmission_interface::TransmissionLoader)
