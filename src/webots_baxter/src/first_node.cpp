#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace webots_baxter
// Namespace should match the package name
{
class FirstNode : public rclcpp::Node
// New class FirstNode is created which inherits from rclcpp::Node
{
public:
  explicit FirstNode(const rclcpp::NodeOptions& options) : rclcpp::Node("first_node", options)
  // This is a constructors which calls the base class constructor giving it same default node name "first_node"
  {
    timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&FirstNode::TimerCallback, this));
    RCLCPP_INFO(get_logger(), "First node is ready!");
    // Then continuing the constructor we logged a message for the user
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;

  void TimerCallback()
  {
    RCLCPP_INFO(get_logger(), "Hello!");
  }
};

}  // namespace webots_baxter

RCLCPP_COMPONENTS_REGISTER_NODE(webots_baxter::FirstNode)