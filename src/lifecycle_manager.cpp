#include "template_lifecycle_manager/lifecycle_manager.hpp"

#define ANSI_COLOR_RESET "\033[0m"
#define ANSI_COLOR_BOLD_BLUE "\033[1;34m"

namespace template_lifecycle_manager
{

// ========== コンストラクタ ========== //
LifecycleManager::LifecycleManager(
  const rclcpp::NodeOptions& options
): Node("lifecycle_manager", options)
{
  using namespace std::placeholders;
  using namespace std::chrono_literals;

  RCLCPP_INFO(this->get_logger(), "Creating");

  /*** パラメータの準備 ***/
  node_names_ = this->declare_parameter<std::vector<std::string>>("node_names", std::vector<std::string>(0,""));
  autostart_ = this->declare_parameter<bool>("autostart", true);
  double timeout_ = this->declare_parameter<double>("timeout", 10.0);
  this->get_parameter("node_names", node_names_);
  this->get_parameter("autostart", autostart_);
  this->get_parameter("timeout", timeout_);
  
  timeout_sec_ = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::duration<double>(timeout_));
  
  /*** 自動スタートでないときは、各状態のサービスを受けたときのみその状態に遷移する ***/
  if(!autostart_)
  {
    srv_ = this->create_service<lifecycle_msgs::srv::ChangeState>(
      get_name() + std::string("/manage"),
      std::bind(&LifecycleManager::callbackManager, this, _1, _2, _3),
      rmw_qos_profile_services_default
    );
  }
  /*** 自動スタートのときは、自動でactive状態まで遷移する ***/
  else if(autostart_)
  {
    autoStartUp();
  }
  
}

// ========== デストラクタ ========== //
LifecycleManager::~LifecycleManager()
{
  RCLCPP_INFO(this->get_logger(), "Destroying %s", get_name());
}

// ========== コールバック関数 ========== //
void LifecycleManager::callbackManager(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<lifecycle_msgs::srv::ChangeState::Request> req,
  const std::shared_ptr<lifecycle_msgs::srv::ChangeState::Response> res)
{
  bool success_ = true;
  
  RCLCPP_INFO(this->get_logger(), "Service was called.");
  
  for(auto & node_name_ : node_names_)
  {
    if(!changeState(node_name_, req->transition.id))
    {
      success_ = false;
    }
  }
  
  res->success = success_;

  if(checkStateForAllNodes(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE))
  {
    RCLCPP_INFO(this->get_logger(), ANSI_COLOR_BOLD_BLUE "All requested nodes are active." ANSI_COLOR_RESET);
  }
}

// ========== 1つのノードの状態を取得する関数 ========== //
std::uint8_t LifecycleManager::getState(const std::string & node_name)
{
  auto client = this->create_client<lifecycle_msgs::srv::GetState>(
    node_name + std::string("get_state")
  );
  
  if(!client->wait_for_service(timeout_sec_))
  {
    RCLCPP_ERROR(this->get_logger(), "Service %s/get_state do not appear...", node_name.c_str());
    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  }
  
  auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  auto result = client->async_send_request(request);
  
  if(rclcpp::spin_until_future_complete(this->shared_from_this(), result, timeout_sec_) == rclcpp::FutureReturnCode::SUCCESS)
  {
    return result.get()->current_state.id;
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to call the service");
    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  }
}

// ========== 1つのノードをある状態に遷移させる関数 ========== //
bool LifecycleManager::changeState(const std::string & node_name, std::uint8_t transition)
{
  auto client = this->create_client<lifecycle_msgs::srv::ChangeState>(
    node_name + std::string("change_state")
  );
  
  if(!client->wait_for_service(timeout_sec_))
  {
    RCLCPP_ERROR(this->get_logger(), "Service %s/change_state do not appear...", node_name.c_str());
    return false;
  }
  
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;
  auto result = client->async_send_request(request);
  
  if(rclcpp::spin_until_future_complete(this->shared_from_this(), result, timeout_sec_) == rclcpp::FutureReturnCode::SUCCESS)
  {
    return result.get()->success;
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to call the service");
    return false;
  }
}

// ========== 全ノードの状態がある状態に一致しているか確認する関数 ========== //
bool LifecycleManager::checkStateForAllNodes(
  std::uint8_t transition)
{
  return std::all_of(
    node_names_.begin(), node_names_.end(),
    [&](const std::string & node_name_)
    {
      return getState(node_name_) == transition;
    }
  );
}

// ========== 全ノードをある状態に遷移させる関数 ========== //
bool LifecycleManager::changeStateForAllNodes(
  std::uint8_t transition)
{
  return std::all_of(
    node_names_.begin(), node_names_.end(),
    [&](const std::string & node_name_)
    {
      return changeState(node_name_, transition);
    }
  );
}

// ========== 全ノードを強制シャットダウンする関数 ========== //
void LifecycleManager::shutdownAllNodes()
{
  for(auto & node_name_ : node_names_)
  {
    if(getState(node_name_) == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      if(!changeState(node_name_, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
      {
        continue;
      }
    }
    if(getState(node_name_) == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
      if(!changeState(node_name_, lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP))
      {
        continue;
      }
    }
    if(getState(node_name_) == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
    {
      if(!changeState(node_name_, lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN))
      {
        continue;
      }
    }
  }
}

// ========== 自動起動する関数 ========== //
void LifecycleManager::autoStartUp()
{
  using namespace std::chrono_literals;
  
  if(node_names_.size() == 0)
  {
    RCLCPP_INFO(this->get_logger(), "Lifecycle node is none.");
    return;
  }
  
  /*** 全ノードがUnconfigured状態か確認 ***/
  if(checkStateForAllNodes(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED))
  {
    RCLCPP_INFO(this->get_logger(), "Checked unconfigured nodes.");
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to check unconfigured nodes.");
    return;
  }
  
  /*** 全ノードをConfigured状態に遷移 ***/
  if(changeStateForAllNodes(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE))
  {
    RCLCPP_INFO(this->get_logger(), "Change to configured nodes");
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to change configured nodes");
    return;
  }
  
  /*** 全ノードをActivated状態に遷移 ***/
  if(changeStateForAllNodes(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
  {
    RCLCPP_INFO(this->get_logger(), "Change to activated nodes");
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to change activated nodes");
    return;
  }

  RCLCPP_INFO(this->get_logger(), ANSI_COLOR_BOLD_BLUE "All requested nodes are active." ANSI_COLOR_RESET);
  
  while(rclcpp::ok())
  {
    rclcpp::sleep_for(10s);

    /*** 全ノードがActivated状態か確認 ***/
    if(checkStateForAllNodes(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE))
    {
      RCLCPP_INFO(this->get_logger(), "Checked activated nodes.");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to check activated nodes.");
      break;
    }
  }
  
  /*** 全ノードをシャットダウン ***/
  shutdownAllNodes();
}


} // template_lifecycle_manager

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(template_lifecycle_manager::LifecycleManager)
