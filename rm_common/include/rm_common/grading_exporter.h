//
// Created by yezi on 22-5-21.
//

#pragma once
#include <vector>
#include <ros/ros.h>

namespace rm_common
{
class GradingExporter
{
public:
  GradingExporter() = default;
  void init(XmlRpc::XmlRpcValue& grading_config)
  {
    ROS_ASSERT(grading_config.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int i = 0; i < grading_config.size(); i++)
    {
      ROS_ASSERT(grading_config[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
      ROS_ASSERT(grading_config[i].size() == 2);
      ROS_ASSERT(grading_config[i][0].getType() == XmlRpc::XmlRpcValue::TypeDouble ||
                 grading_config[i][0].getType() == XmlRpc::XmlRpcValue::TypeInt);
      ROS_ASSERT(grading_config[i][1].getType() == XmlRpc::XmlRpcValue::TypeDouble ||
                 grading_config[i][1].getType() == XmlRpc::XmlRpcValue::TypeInt);
      if (!grade_.empty())
        if ((double)grading_config[i][0] < grade_.back())
        {
          ROS_ERROR("Please sort the grades from smallest to largest. %lf < %lf", (double)grading_config[i][0],
                    grade_.back());
          return;
        }
      grade_.push_back(grading_config[i][0]);
      grade_export_value_.push_back(grading_config[i][1]);
    }
  }
  double output(double grade)
  {
    if (grade > grade_.back())
      return grade_export_value_.back();
    else if (grade < grade_.front())
      return grade_export_value_.front();
    for (int i = 0; i < (int)grade_.size(); i++)
    {
      if (grade >= grade_[i] && grade <= grade_[i + 1])
        return grade_export_value_[i] +
               ((grade_export_value_[i + 1] - grade_export_value_[i]) / (grade_[i + 1] - grade_[i])) *
                   (grade - grade_[i]);
    }
    ROS_ERROR("The grades aren't sorted from smallest to largest.");
    return 0;
  }

private:
  std::vector<double> grade_;
  std::vector<double> grade_export_value_;
};
}  // namespace rm_common
