#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);

  // Instantiate a node
  rclcpp::Node::SharedPtr node =
      rclcpp::Node::make_shared("executor_example_1_node");
    
  RCLCPP_INFO(node->get_logger(), "Bacon pancakes, making bacon pancakes");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}