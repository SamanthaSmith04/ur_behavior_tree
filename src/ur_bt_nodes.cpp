#include <ur_behavior_tree/ur_bt_nodes.h>

template <typename T>
T getBTInput(const BT::TreeNode* node, const std::string& port)
{
  BT::Expected<T> input = node->getInput<T>(port);
  if (!input)
    throw BT::RuntimeError("Failed to get required input value: '" + input.error() + "'");

  return input.value();
}

template <typename T>
T get_parameter(rclcpp::Node::SharedPtr node, const std::string& key)
{
  T val;
  if (!node->get_parameter(key, val))
    throw std::runtime_error("Failed to get '" + key + "' parameter");
  return val;
}

namespace ur_behavior_tree
{

bool SetIONode::setRequest(typename Request::SharedPtr& request)
{
  request->fun = getBTInput<uint32_t>(this, FUNCTION_KEY);
  request->pin = getBTInput<uint32_t>(this, PIN_KEY);
  request->state = getBTInput<float_t>(this, STATE_KEY);
  return true;
}

BT::NodeStatus SetIONode::onResponseReceived(const typename Response::SharedPtr& response)
{
  if (!response->success)
  {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

bool SetRobotModeRunningNode::setGoal(RosActionNode::Goal& goal) 
{
  goal.target_robot_mode = ur_dashboard_msgs::msg::RobotMode::RUNNING;
  goal.stop_program = true;
  goal.play_program = false;
  return true;
}

BT::NodeStatus SetRobotModeRunningNode::onResultReceived(const WrappedResult& wr)
{
  if (!wr.result->success)
  {
    config().blackboard->set(ERROR_MESSAGE_KEY, "Failed to set robot mode to RUNNING: " + wr.result->message);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

bool GetRobotModeNode::setRequest(typename Request::SharedPtr& request)
{
  // No parameters to set for this service
  (void)request;
  return true;
}

BT::NodeStatus GetRobotModeNode::onResponseReceived(const typename Response::SharedPtr& response)
{
  if (response->robot_mode.mode != ur_dashboard_msgs::msg::RobotMode::RUNNING)
  {
    config().blackboard->set(ERROR_MESSAGE_KEY, "Robot mode is not RUNNING: " + response->answer);
    return BT::NodeStatus::FAILURE;
  }

  setOutput(ROBOT_MODE_OUTPUT_PORT_KEY, response->robot_mode);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ReadSingleIONode::onTick(const typename ur_msgs::msg::IOStates::SharedPtr& last_msg)
{
  try
  {
    // Check
    if (last_msg == nullptr)
      throw std::runtime_error("No IO state message acquired yet");

    // Find the index of the requested address in the last state message
    if (getBTInput<std::string>(this, IO_STATES_TYPE_KEY) == "digital_in_states")
    {
      auto pin_it = std::find_if(
          last_msg->digital_in_states.begin(), last_msg->digital_in_states.end(),
          [this](const ur_msgs::msg::Digital& state) {
            return state.pin == getBTInput<uint32_t>(this, IO_STATES_PIN_KEY);
          });
      if (pin_it == last_msg->digital_in_states.end())
        throw std::runtime_error("Requested digital input pin not found in the last IO state message");
      auto idx = static_cast<std::size_t>(std::distance(last_msg->digital_in_states.begin(), pin_it));
      setOutput(IO_STATES_OUTPUT_PORT_KEY, last_msg->digital_in_states.at(idx).state);
    }
    else if (getBTInput<std::string>(this, IO_STATES_TYPE_KEY) == "digital_out_states")
    {
      auto pin_it = std::find_if(
          last_msg->digital_out_states.begin(), last_msg->digital_out_states.end(),
          [this](const ur_msgs::msg::Digital& state) {
            return state.pin == getBTInput<uint32_t>(this, IO_STATES_PIN_KEY);
          });
      if (pin_it == last_msg->digital_out_states.end())
        throw std::runtime_error("Requested digital output pin not found in the last IO state message");
      auto idx = static_cast<std::size_t>(std::distance(last_msg->digital_out_states.begin(), pin_it));
      setOutput(IO_STATES_OUTPUT_PORT_KEY, last_msg->digital_out_states.at(idx).state);
    }
    else if (getBTInput<std::string>(this, IO_STATES_TYPE_KEY) == "analog_in_states")
    {
      auto pin_it = std::find_if(
          last_msg->analog_in_states.begin(), last_msg->analog_in_states.end(),
          [this](const ur_msgs::msg::Analog& state) {
            return state.pin == getBTInput<uint32_t>(this, IO_STATES_PIN_KEY);
          });
      if (pin_it == last_msg->analog_in_states.end())
        throw std::runtime_error("Requested analog input pin not found in the last IO state message");
      auto idx = static_cast<std::size_t>(std::distance(last_msg->analog_in_states.begin(), pin_it));
      setOutput(IO_STATES_OUTPUT_PORT_KEY, last_msg->analog_in_states.at(idx).state);
    }
    else if (getBTInput<std::string>(this, IO_STATES_TYPE_KEY) == "analog_out_states")
    {
      auto pin_it = std::find_if(
          last_msg->analog_out_states.begin(), last_msg->analog_out_states.end(),
          [this](const ur_msgs::msg::Analog& state) {
            return state.pin == getBTInput<uint32_t>(this, IO_STATES_PIN_KEY);
          });
      if (pin_it == last_msg->analog_out_states.end())
        throw std::runtime_error("Requested analog output pin not found in the last IO state message");
      auto idx = static_cast<std::size_t>(std::distance(last_msg->analog_out_states.begin(), pin_it));
      setOutput(IO_STATES_OUTPUT_PORT_KEY, last_msg->analog_out_states.at(idx).state);
    }
    else
    {
      throw std::runtime_error("Unsupported IO states type: " + getBTInput<std::string>(this, IO_STATES_TYPE_KEY));
    }
  }
  catch (const std::exception& ex)
  {
    config().blackboard->set(ERROR_MESSAGE_KEY, ex.what());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus AddJointsToTrajectoryNode::onTick(const typename sensor_msgs::msg::JointState::SharedPtr& last_msg)
{
  try
  {
    // Check
    if (last_msg == nullptr)
      throw std::runtime_error("No joint state message acquired yet");

    BT::Expected<trajectory_msgs::msg::JointTrajectory> input =
        getInput<trajectory_msgs::msg::JointTrajectory>(TRAJECTORY_INPUT_PORT_KEY);
    if (!input)
    {
      std::stringstream ss;
      ss << "Failed to get required input value: '" << input.error() << "'";
      throw std::runtime_error(ss.str());
    }
    trajectory_msgs::msg::JointTrajectory trajectory = input.value();

    // Get the controller joint names from parameter
    std::vector<std::string> controller_joint_names =
        get_parameter<std::vector<std::string>>(node_, CONTROLLER_JOINT_NAMES_PARAM);

    // Find the index of each controller joint in the current trajectory; substitute with -1 if the joint is not found
    std::vector<int> joint_idx_in_trajectory;
    std::transform(controller_joint_names.begin(), controller_joint_names.end(),
                   std::back_inserter(joint_idx_in_trajectory), [&trajectory](const std::string& joint) {
                     auto it = std::find(trajectory.joint_names.begin(), trajectory.joint_names.end(), joint);
                     if (it == trajectory.joint_names.end())
                       return -1;
                     return static_cast<int>(std::distance(trajectory.joint_names.begin(), it));
                   });

    // Find the index of each controller joint in the last state message; substitute with -1 if the joint is not found
    std::vector<int> joint_idx_in_last_msg;
    std::transform(controller_joint_names.begin(), controller_joint_names.end(),
                   std::back_inserter(joint_idx_in_last_msg), [&last_msg](const std::string& joint) {
                     auto it = std::find(last_msg->name.begin(), last_msg->name.end(), joint);
                     if (it == last_msg->name.end())
                       return -1;
                     return static_cast<int>(std::distance(last_msg->name.begin(), it));
                   });

    // Reconstruct the trajectory
    trajectory_msgs::msg::JointTrajectory out_traj;
    out_traj.header = trajectory.header;
    out_traj.joint_names = controller_joint_names;
    out_traj.points.reserve(trajectory.points.size());
    for (const trajectory_msgs::msg::JointTrajectoryPoint& pt : trajectory.points)
    {
      trajectory_msgs::msg::JointTrajectoryPoint new_pt;
      new_pt.time_from_start = pt.time_from_start;

      for (std::size_t i = 0; i < controller_joint_names.size(); ++i)
      {
        // Look for the joint first in the trajectory
        if (joint_idx_in_trajectory.at(i) >= 0)
        {
          auto idx = static_cast<std::size_t>(joint_idx_in_trajectory.at(i));
          new_pt.positions.push_back(pt.positions.at(idx));
          new_pt.velocities.push_back(pt.velocities.at(idx));
          if (!pt.accelerations.empty())
            new_pt.accelerations.push_back(pt.accelerations.at(idx));
          if (!pt.effort.empty())
            new_pt.effort.push_back(pt.effort.at(idx));
        }
        // Then look in the current state message
        else if (joint_idx_in_last_msg.at(i) >= 0)
        {
          auto idx = static_cast<std::size_t>(joint_idx_in_last_msg.at(i));
          new_pt.positions.push_back(last_msg->position.at(idx));
          new_pt.velocities.push_back(0.0);
          if (!pt.accelerations.empty())
            new_pt.accelerations.push_back(0.0);
          if (!pt.effort.empty())
            new_pt.effort.push_back(0.0);
        }
        // Finally return an error if the joint cannot be found in either the trajectory or the last joint state
        else
        {
          std::stringstream ss;
          ss << "Failed to find controller joint name '" << controller_joint_names.at(i)
             << "' in input trajectory and current state";
          throw std::runtime_error(ss.str());
        }
      }

      // Append the trajectory point
      out_traj.points.push_back(new_pt);
    }

    BT::Result output = setOutput(TRAJECTORY_OUTPUT_PORT_KEY, out_traj);
    if (!output)
    {
      std::stringstream ss;
      ss << "Failed to set required output value: '" << output.error() << "'";
      throw std::runtime_error(ss.str());
    }
  }
  catch (const std::exception& ex)
  {
    config().blackboard->set(ERROR_MESSAGE_KEY, ex.what());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace ur_behavior_tree

BTCPP_EXPORT void BT_RegisterRosNodeFromPlugin(BT::BehaviorTreeFactory& factory, const BT::RosNodeParams& params)
{
  factory.registerNodeType<ur_behavior_tree::SetIONode>("SetIO", params);

  if (!params.nh->has_parameter(ur_behavior_tree::AddJointsToTrajectoryNode::CONTROLLER_JOINT_NAMES_PARAM))
  {
    params.nh->declare_parameter<std::vector<std::string>>(
        ur_behavior_tree::AddJointsToTrajectoryNode::CONTROLLER_JOINT_NAMES_PARAM, {});
  }
  factory.registerNodeType<ur_behavior_tree::AddJointsToTrajectoryNode>("AddJointsToTrajectory", params);

  factory.registerNodeType<ur_behavior_tree::ReadSingleIONode>("ReadSingleIO", params);

  factory.registerNodeType<ur_behavior_tree::GetRobotModeNode>("GetRobotMode", params);
  factory.registerNodeType<ur_behavior_tree::SetRobotModeRunningNode>("SetRobotModeRunning", params);
}
