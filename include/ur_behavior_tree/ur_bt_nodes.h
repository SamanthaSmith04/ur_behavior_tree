#pragma once

#include <behaviortree_ros2/bt_service_node.hpp>
#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <ur_msgs/srv/set_io.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace ur_behavior_tree
{
inline static const std::string ERROR_MESSAGE_KEY = "error_message";

template <typename T>
class RosServiceNode : public BT::RosServiceNode<T>
{
public:
  using BT::RosServiceNode<T>::RosServiceNode;

  inline BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override
  {
    std::stringstream ss;
    ss << "Service '" << BT::RosServiceNode<T>::prev_service_name_ << "'";

    switch (error)
    {
      case BT::SERVICE_UNREACHABLE:
        ss << " is unreachable";
        break;
      case BT::SERVICE_TIMEOUT:
        ss << " timed out";
        break;
      case BT::INVALID_REQUEST:
        ss << " was sent an invalid request";
        break;
      case BT::SERVICE_ABORTED:
        ss << " was aborted";
        break;
      default:
        break;
    }

    this->config().blackboard->set(ERROR_MESSAGE_KEY, ss.str());

    return BT::NodeStatus::FAILURE;
  }
};

class SetIONode : public RosServiceNode<ur_msgs::srv::SetIO>
{
public:
  inline static std::string FUNCTION_KEY = "function";
  inline static std::string PIN_KEY = "pin";
  inline static std::string STATE_KEY = "state";
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort(FUNCTION_KEY), BT::InputPort(PIN_KEY), BT::InputPort(STATE_KEY) });
  }
  using RosServiceNode<ur_msgs::srv::SetIO>::RosServiceNode;

  bool setRequest(typename Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override;
};


class AddJointsToTrajectoryNode : public BT::RosTopicSubNode<sensor_msgs::msg::JointState>
{
public:
  inline static std::string TRAJECTORY_INPUT_PORT_KEY = "input";
  inline static std::string TRAJECTORY_OUTPUT_PORT_KEY = "output";
  inline static std::string CONTROLLER_JOINT_NAMES_PARAM = "controller_joint_names";
  inline static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort<trajectory_msgs::msg::JointTrajectory>(TRAJECTORY_INPUT_PORT_KEY),
                                BT::OutputPort<trajectory_msgs::msg::JointTrajectory>(TRAJECTORY_OUTPUT_PORT_KEY) });
  }
  using BT::RosTopicSubNode<sensor_msgs::msg::JointState>::RosTopicSubNode;

  BT::NodeStatus onTick(const typename sensor_msgs::msg::JointState::SharedPtr& last_msg) override;
};

}  // namespace ur_behavior_tree
