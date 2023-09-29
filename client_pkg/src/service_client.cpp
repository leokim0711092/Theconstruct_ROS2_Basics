#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("service_client");
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client =
    node->create_client<std_srvs::srv::Empty>("moving");

  auto request = std::make_shared<std_srvs::srv::Empty::Request>();

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result_future = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result_future) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    auto result = result_future.get();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The robot is moving");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service /moving");
  }

  rclcpp::shutdown();
  return 0;
}