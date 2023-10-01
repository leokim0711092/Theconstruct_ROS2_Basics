#include "rclcpp/client.hpp"
#include "rclcpp/node.hpp"
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <services_quiz_srv/srv/spin.hpp>

#include <chrono>

int main(int argc, char**argv){
    rclcpp::init(argc,argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("rotate_client");
    rclcpp::Client<services_quiz_srv::srv::Spin>::SharedPtr client = node->create_client<services_quiz_srv::srv::Spin>("/rotate");
    


  auto request = std::make_shared<services_quiz_srv::srv::Spin::Request>();
  request->direction = "right";
  request->angular_velocity = 0.2;
  request->time = 10;


    while (!client->wait_for_service(std::chrono::seconds(1))) {
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
    if (result->success == true)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service returned success");
    }
    else if (result->success == false)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service returned false");
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service /rotate");
  }

  rclcpp::shutdown();
  return 0;
}

