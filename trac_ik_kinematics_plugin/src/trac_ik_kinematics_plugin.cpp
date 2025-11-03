/********************************************************************************
Copyright (c) 2015, TRACLabs, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software
       without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/


#include <urdf/model.h>
#include <urdf_parser/urdf_parser.h>
#include <tf2_kdl/tf2_kdl.hpp>
#include <algorithm>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <trac_ik/trac_ik.hpp>
#include <trac_ik/trac_ik_kinematics_plugin.hpp>
#include <moveit/robot_model/robot_model.h>
#include <limits>
#include <rclcpp/logging.hpp>
#include <sstream>

namespace trac_ik_kinematics_plugin
{

// bool TRAC_IKKinematicsPlugin::initialize(const std::string &robot_description,
//     const std::string& group_name,
//     const std::string& base_name,
//     const std::string& tip_name,
//     double search_discretization)
// {
//   std::vector<std::string> tip_names = {tip_name};
//   setValues(robot_description, group_name, base_name, tip_names, search_discretization);

//   urdf::Model robot_model;
//   std::string xml_string;

//   // In ROS2, robot_description is passed directly
//   xml_string = robot_description;

//   if (!robot_model.initString(xml_string))
//   {
//     return false;
//   }

//   KDL::Tree tree;

//   if (!kdl_parser::treeFromUrdfModel(robot_model, tree))
//   {
//     return false;
//   }

//   if (!tree.getChain(base_name, tip_name, chain))
//   {
//     return false;
//   }

//   num_joints_ = chain.getNrOfJoints();

//   std::vector<KDL::Segment> chain_segs = chain.segments;

//   urdf::JointConstSharedPtr joint;

//   std::vector<double> l_bounds, u_bounds;

//   joint_min.resize(num_joints_);
//   joint_max.resize(num_joints_);

//   uint joint_num = 0;
//   for (unsigned int i = 0; i < chain_segs.size(); ++i)
//   {

//     link_names_.push_back(chain_segs[i].getName());
//     joint = robot_model.getJoint(chain_segs[i].getJoint().getName());
//     if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
//     {
//       joint_num++;
//       assert(joint_num <= num_joints_);
//       float lower, upper;
//       int hasLimits;
//       joint_names_.push_back(joint->name);
//       if (joint->type != urdf::Joint::CONTINUOUS)
//       {
//         if (joint->safety)
//         {
//           lower = std::max(joint->limits->lower, joint->safety->soft_lower_limit);
//           upper = std::min(joint->limits->upper, joint->safety->soft_upper_limit);
//         }
//         else
//         {
//           lower = joint->limits->lower;
//           upper = joint->limits->upper;
//         }
//         hasLimits = 1;
//       }
//       else
//       {
//         hasLimits = 0;
//       }
//       if (hasLimits)
//       {
//         joint_min(joint_num - 1) = lower;
//         joint_max(joint_num - 1) = upper;
//       }
//       else
//       {
//         joint_min(joint_num - 1) = std::numeric_limits<float>::lowest();
//         joint_max(joint_num - 1) = std::numeric_limits<float>::max();
//       }
//     }
//   }

//   // Note: In ROS2 MoveIt, parameter lookup would be done through the node
//   // For now, use default values
//   position_ik_ = false;
//   solve_type = "Speed";

//   active_ = true;
//   return true;
// }

// MoveIt 2.5.x initialize method (current API) - this is what MoveIt calls
bool TRAC_IKKinematicsPlugin::initialize(const rclcpp::Node::SharedPtr& node,
    const moveit::core::RobotModel& robot_model,
    const std::string& group_name,
    const std::string& base_name,
    const std::vector<std::string>& tip_frames,
    double search_discretization)
{
  // Use the first tip frame if multiple are provided
  if (tip_frames.empty())
  {
    RCLCPP_ERROR(kinematics::KinematicsBase::LOGGER,
                 "TRAC-IK initialize (MoveIt 2.5 API) failed: tip_frames is empty for group '%s'",
                 group_name.c_str());
    return false;
  }
  std::string tip_name = tip_frames[0];
  RCLCPP_INFO(kinematics::KinematicsBase::LOGGER,
              "TRAC-IK initialize (MoveIt 2.5 API): group='%s' base='%s' tip='%s' tips=%zu search_discretization=%.4f",
              group_name.c_str(), base_name.c_str(), tip_name.c_str(), tip_frames.size(), search_discretization);
  
  setValues("", group_name, base_name, tip_frames, search_discretization);

  KDL::Tree tree;

  // Get URDF model from the RobotModel
  const auto& urdf_model = robot_model.getURDF();
  if (!urdf_model || !kdl_parser::treeFromUrdfModel(*urdf_model, tree))
  {
    RCLCPP_ERROR(kinematics::KinematicsBase::LOGGER,
                 "TRAC-IK initialize: Failed to load KDL tree from URDF for group '%s'",
                 group_name.c_str());
    return false;
  }

  if (!tree.getChain(base_name, tip_name, chain))
  {
    RCLCPP_ERROR(kinematics::KinematicsBase::LOGGER,
                 "TRAC-IK initialize: Failed to extract KDL chain base='%s' -> tip='%s'",
                 base_name.c_str(), tip_name.c_str());
    return false;
  }

  num_joints_ = chain.getNrOfJoints();
  RCLCPP_INFO(kinematics::KinematicsBase::LOGGER,
              "TRAC-IK: KDL chain ready with %u joints", num_joints_);

  std::vector<KDL::Segment> chain_segs = chain.segments;

  urdf::JointConstSharedPtr joint;

  std::vector<double> l_bounds, u_bounds;

  joint_min.resize(num_joints_);
  joint_max.resize(num_joints_);

  uint joint_num = 0;
  for (unsigned int i = 0; i < chain_segs.size(); ++i)
  {
    link_names_.push_back(chain_segs[i].getName());
    joint = urdf_model->getJoint(chain_segs[i].getJoint().getName());
    if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
    {
      joint_num++;
      assert(joint_num <= num_joints_);
      float lower, upper;
      int hasLimits;
      joint_names_.push_back(joint->name);
      if (joint->type != urdf::Joint::CONTINUOUS)
      {
        if (joint->safety)
        {
          lower = std::max(joint->limits->lower, joint->safety->soft_lower_limit);
          upper = std::min(joint->limits->upper, joint->safety->soft_upper_limit);
        }
        else
        {
          lower = joint->limits->lower;
          upper = joint->limits->upper;
        }
        hasLimits = 1;
      }
      else
      {
        hasLimits = 0;
      }
      if (hasLimits)
      {
        joint_min(joint_num - 1) = lower;
        joint_max(joint_num - 1) = upper;
      }
      else
      {
        joint_min(joint_num - 1) = std::numeric_limits<float>::lowest();
        joint_max(joint_num - 1) = std::numeric_limits<float>::max();
      }
    }
  }

  {
    std::ostringstream oss;
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      if (i) oss << ", ";
      oss << joint_names_[i];
    }
    RCLCPP_INFO(kinematics::KinematicsBase::LOGGER,
                "TRAC-IK: joints=[%s]", oss.str().c_str());
  }

  // Try to get parameters from MoveIt (loaded under robot_description_kinematics.<group>.*)
  position_ik_ = false;
  solve_type = "Speed";

  auto try_param_bool = [&](const std::string& key, bool& out, bool def) -> bool {
    return lookupParam(node, key, out, def);
  };
  auto try_param_str = [&](const std::string& key, std::string& out, const std::string& def) -> bool {
    return lookupParam(node, key, out, def);
  };

  // Primary (MoveIt standard): robot_description_kinematics.<group>.<param>
  bool pos_tmp = position_ik_;
  std::string solve_tmp = solve_type;
  bool got_pos = try_param_bool("robot_description_kinematics." + group_name + ".position_only_ik", pos_tmp, position_ik_);
  bool got_solve = try_param_str("robot_description_kinematics." + group_name + ".solve_type", solve_tmp, solve_type);

  // Fallbacks
  if (!got_pos) got_pos = try_param_bool(group_name + ".position_only_ik", pos_tmp, position_ik_);
  if (!got_pos) got_pos = try_param_bool("position_only_ik", pos_tmp, position_ik_);
  if (!got_solve) got_solve = try_param_str(group_name + ".solve_type", solve_tmp, solve_type);
  if (!got_solve) got_solve = try_param_str("solve_type", solve_tmp, solve_type);

  if (got_pos) position_ik_ = pos_tmp;
  if (got_solve) solve_type = solve_tmp;

  RCLCPP_INFO(kinematics::KinematicsBase::LOGGER,
              "TRAC-IK: position_only_ik=%s solve_type=%s (group=%s)",
              position_ik_ ? "true" : "false", solve_type.c_str(), group_name.c_str());

  active_ = true;
  RCLCPP_INFO(kinematics::KinematicsBase::LOGGER,
              "TRAC-IK initialize: solver active for group '%s'", group_name.c_str());
  return true;
}

// Newer initialize method (future API)
bool TRAC_IKKinematicsPlugin::initialize(const std::string& robot_description,
    std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface,
    const std::string& param_namespace)
{
  RCLCPP_INFO(kinematics::KinematicsBase::LOGGER,
              "TRAC-IK initialize: ns='%s' robot_description size=%zu",
              param_namespace.c_str(), robot_description.size());
  // Parse URDF from robot_description string
  urdf::ModelInterfaceSharedPtr urdf_model;
  try
  {
    urdf_model = urdf::parseURDF(robot_description);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(kinematics::KinematicsBase::LOGGER,
                 "TRAC-IK initialize: URDF parse exception: %s", e.what());
    return false;
  }
  
  if (!urdf_model)
  {
    RCLCPP_ERROR(kinematics::KinematicsBase::LOGGER,
                 "TRAC-IK initialize: URDF parse returned null");
    return false;
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(*urdf_model, tree))
  {
    RCLCPP_ERROR(kinematics::KinematicsBase::LOGGER,
                 "TRAC-IK initialize: Failed to load KDL tree from URDF");
    return false;
  }

  // Get parameters from NodeParametersInterface
  std::string group_name;
  std::string base_name;
  std::string tip_name;
  double search_discretization = 0.01;

  try
  {
    if (parameters_interface)
    {
      auto desc = rcl_interfaces::msg::ParameterDescriptor{};
      
      // Get group_name
      if (parameters_interface->has_parameter(param_namespace + ".group_name"))
      {
        auto param = parameters_interface->get_parameter(param_namespace + ".group_name");
        group_name = param.as_string();
      }
      
      // Get base_name
      if (parameters_interface->has_parameter(param_namespace + ".base_name"))
      {
        auto param = parameters_interface->get_parameter(param_namespace + ".base_name");
        base_name = param.as_string();
      }
      
      // Get tip_name
      if (parameters_interface->has_parameter(param_namespace + ".tip_name"))
      {
        auto param = parameters_interface->get_parameter(param_namespace + ".tip_name");
        tip_name = param.as_string();
      }
      
      // Get search_discretization
      if (parameters_interface->has_parameter(param_namespace + ".search_discretization"))
      {
        auto param = parameters_interface->get_parameter(param_namespace + ".search_discretization");
        search_discretization = param.as_double();
      }
    }
  }
  catch (const std::exception& e)
  {
    // Use defaults if parameter retrieval fails
    RCLCPP_WARN(kinematics::KinematicsBase::LOGGER,
                "TRAC-IK initialize: parameter retrieval failed: %s", e.what());
  }

  // Validate we have required parameters
  if (group_name.empty() || base_name.empty() || tip_name.empty())
  {
    RCLCPP_ERROR(kinematics::KinematicsBase::LOGGER,
                 "TRAC-IK initialize: missing required params group='%s' base='%s' tip='%s'",
                 group_name.c_str(), base_name.c_str(), tip_name.c_str());
    return false;
  }

  // Set up the kinematics chain
  std::vector<std::string> tip_frames = {tip_name};
  setValues(robot_description, group_name, base_name, tip_frames, search_discretization);

  if (!tree.getChain(base_name, tip_name, chain))
  {
    RCLCPP_ERROR(kinematics::KinematicsBase::LOGGER,
                 "TRAC-IK initialize: Failed to extract KDL chain base='%s' -> tip='%s'",
                 base_name.c_str(), tip_name.c_str());
    return false;
  }

  num_joints_ = chain.getNrOfJoints();
  RCLCPP_INFO(kinematics::KinematicsBase::LOGGER,
              "TRAC-IK: KDL chain ready with %u joints", num_joints_);

  std::vector<KDL::Segment> chain_segs = chain.segments;

  urdf::JointConstSharedPtr joint;

  joint_min.resize(num_joints_);
  joint_max.resize(num_joints_);

  uint joint_num = 0;
  for (unsigned int i = 0; i < chain_segs.size(); ++i)
  {
    link_names_.push_back(chain_segs[i].getName());
    joint = urdf_model->getJoint(chain_segs[i].getJoint().getName());
    if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
    {
      joint_num++;
      assert(joint_num <= num_joints_);
      float lower, upper;
      int hasLimits;
      joint_names_.push_back(joint->name);
      if (joint->type != urdf::Joint::CONTINUOUS)
      {
        if (joint->safety)
        {
          lower = std::max(joint->limits->lower, joint->safety->soft_lower_limit);
          upper = std::min(joint->limits->upper, joint->safety->soft_upper_limit);
        }
        else
        {
          lower = joint->limits->lower;
          upper = joint->limits->upper;
        }
        hasLimits = 1;
      }
      else
      {
        hasLimits = 0;
      }
      if (hasLimits)
      {
        joint_min(joint_num - 1) = lower;
        joint_max(joint_num - 1) = upper;
      }
      else
      {
        joint_min(joint_num - 1) = std::numeric_limits<float>::lowest();
        joint_max(joint_num - 1) = std::numeric_limits<float>::max();
      }
    }
  }

  {
    std::ostringstream oss;
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      if (i) oss << ", ";
      oss << joint_names_[i];
    }
    RCLCPP_INFO(kinematics::KinematicsBase::LOGGER,
                "TRAC-IK: joints=[%s]", oss.str().c_str());
  }

  // Get IK solver parameters
  position_ik_ = false;
  solve_type = "Speed";
  
  try
  {
    if (parameters_interface)
    {
      if (parameters_interface->has_parameter(param_namespace + ".position_only_ik"))
      {
        auto param = parameters_interface->get_parameter(param_namespace + ".position_only_ik");
        position_ik_ = param.as_bool();
      }
      
      if (parameters_interface->has_parameter(param_namespace + ".solve_type"))
      {
        auto param = parameters_interface->get_parameter(param_namespace + ".solve_type");
        solve_type = param.as_string();
      }
    }
  }
  catch (const std::exception& e)
  {
    // Silently ignore parameter errors and use defaults
    RCLCPP_WARN(kinematics::KinematicsBase::LOGGER,
                "TRAC-IK: parameter read exception: %s", e.what());
  }

  active_ = true;
  RCLCPP_INFO(kinematics::KinematicsBase::LOGGER,
              "TRAC-IK initialize: solver active for group '%s'", group_name.c_str());
  return true;
}


int TRAC_IKKinematicsPlugin::getKDLSegmentIndex(const std::string &name) const
{
  int i = 0;
  while (i < (int)chain.getNrOfSegments())
  {
    if (chain.getSegment(i).getName() == name)
    {
      return i + 1;
    }
    i++;
  }
  return -1;
}


bool TRAC_IKKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
    const std::vector<double> &joint_angles,
    std::vector<geometry_msgs::msg::Pose> &poses) const
{
  if (!active_)
  {
    return false;
  }
  poses.resize(link_names.size());
  if (joint_angles.size() != num_joints_)
  {
    return false;
  }

  KDL::Frame p_out;
  geometry_msgs::msg::Pose pose;

  KDL::JntArray jnt_pos_in(num_joints_);
  for (unsigned int i = 0; i < num_joints_; i++)
  {
    jnt_pos_in(i) = joint_angles[i];
  }

  KDL::ChainFkSolverPos_recursive fk_solver(chain);

  bool valid = true;
  for (unsigned int i = 0; i < poses.size(); i++)
  {
    if (fk_solver.JntToCart(jnt_pos_in, p_out, getKDLSegmentIndex(link_names[i])) >= 0)
    {
      poses[i] = tf2::toMsg(p_out);
    }
    else
    {
      valid = false;
    }
  }

  return valid;
}


bool TRAC_IKKinematicsPlugin::getPositionIK(const geometry_msgs::msg::Pose &ik_pose,
    const std::vector<double> &ik_seed_state,
    std::vector<double> &solution,
    moveit_msgs::msg::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const
{
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          default_timeout_,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool TRAC_IKKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose &ik_pose,
    const std::vector<double> &ik_seed_state,
    double timeout,
    std::vector<double> &solution,
    moveit_msgs::msg::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const
{
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool TRAC_IKKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose &ik_pose,
    const std::vector<double> &ik_seed_state,
    double timeout,
    const std::vector<double> &consistency_limits,
    std::vector<double> &solution,
    moveit_msgs::msg::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const
{
  const IKCallbackFn solution_callback = 0;
  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool TRAC_IKKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose &ik_pose,
    const std::vector<double> &ik_seed_state,
    double timeout,
    std::vector<double> &solution,
    const IKCallbackFn &solution_callback,
    moveit_msgs::msg::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const
{
  std::vector<double> consistency_limits;
  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool TRAC_IKKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose &ik_pose,
    const std::vector<double> &ik_seed_state,
    double timeout,
    const std::vector<double> &consistency_limits,
    std::vector<double> &solution,
    const IKCallbackFn &solution_callback,
    moveit_msgs::msg::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const
{
  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool TRAC_IKKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose &ik_pose,
    const std::vector<double> &ik_seed_state,
    double timeout,
    std::vector<double> &solution,
    const IKCallbackFn &solution_callback,
    moveit_msgs::msg::MoveItErrorCodes &error_code,
    const std::vector<double> &consistency_limits,
    const kinematics::KinematicsQueryOptions &options) const
{
  if (!active_)
  {
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  if (ik_seed_state.size() != num_joints_)
  {
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  KDL::Frame frame;
  tf2::fromMsg(ik_pose, frame);

  KDL::JntArray in(num_joints_), out(num_joints_);

  for (uint z = 0; z < num_joints_; z++)
    in(z) = ik_seed_state[z];

  KDL::Twist bounds = KDL::Twist::Zero();

  if (position_ik_)
  {
    bounds.rot.x(std::numeric_limits<float>::max());
    bounds.rot.y(std::numeric_limits<float>::max());
    bounds.rot.z(std::numeric_limits<float>::max());
  }

  double epsilon = 1e-5;  //Same as MoveIt's KDL plugin

  TRAC_IK::SolveType solvetype;

  if (solve_type == "Manipulation1")
    solvetype = TRAC_IK::Manip1;
  else if (solve_type == "Manipulation2")
    solvetype = TRAC_IK::Manip2;
  else if (solve_type == "Distance")
    solvetype = TRAC_IK::Distance;
  else
  {
    if (solve_type != "Speed")
    {
      // Log warning: solve_type is not valid
    }
    solvetype = TRAC_IK::Speed;
  }

  TRAC_IK::TRAC_IK ik_solver(chain, joint_min, joint_max, timeout, epsilon, solvetype);

  int rc = ik_solver.CartToJnt(in, frame, out, bounds);


  solution.resize(num_joints_);

  if (rc >= 0)
  {
    for (uint z = 0; z < num_joints_; z++)
      solution[z] = out(z);

    // check for collisions if a callback is provided
    if (solution_callback)
    {
      solution_callback(ik_pose, solution, error_code);
      if (error_code.val == error_code.SUCCESS)
      {
        return true;
      }
      else
      {
        // Solution had an error code
        return false;
      }
    }
    else
      return true; // no collision check callback provided
  }

  error_code.val = error_code.NO_IK_SOLUTION;
  return false;
}



} // end namespace

// Register TRAC_IKKinematicsPlugin as a KinematicsBase implementation
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(trac_ik_kinematics_plugin::TRAC_IKKinematicsPlugin, kinematics::KinematicsBase);
