#ifndef TEMPLATE_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_
#define TEMPLATE_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_

#include <map>
#include <memory>
#include <chrono>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

namespace template_lifecycle_manager
{

class LifecycleManager : public rclcpp::Node
{
public:
  LifecycleManager(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
  );
  ~LifecycleManager();

protected:
  void callbackManager(
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<lifecycle_msgs::srv::ChangeState::Request> req,
    const std::shared_ptr<lifecycle_msgs::srv::ChangeState::Response> res
  );
  std::uint8_t getState(const std::string & node_name);
  bool changeState(const std::string & node_name, std::uint8_t transition);
  bool checkStateForAllNodes(
    std::uint8_t transition
  );
  bool changeStateForAllNodes(
    std::uint8_t transition
  );
  void shutdownAllNodes();
  void autoStartUp();

  rclcpp::Service<lifecycle_msgs::srv::ChangeState>::SharedPtr srv_;
  std::vector<std::string> node_names_;
  bool autostart_;
  std::chrono::seconds timeout_sec_;
};

} // namespace template_lifecycle_manager

#endif // TEMPLATE_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_
