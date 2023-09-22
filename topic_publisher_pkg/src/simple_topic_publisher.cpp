// Import all the necessary ROS libraries and import the Int32 message from the std_msgs package
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Initiate a Node named 'simple_publisher'
  auto node = rclcpp::Node::make_shared("simple_publisher");
  // Create a Publisher object that will publish on the /counter topic, messages of the type Int32
  // The 10 indicates the size of the queue
  auto publisher = node->create_publisher<std_msgs::msg::Int32>("counter", 10);
  // Create a variable named 'message' of type Int32
  auto message = std::make_shared<std_msgs::msg::Int32>();
  // Initialize the 'message' variable
  message->data = 0;
  // Set a publish rate of 2 Hz
  rclcpp::WallRate loop_rate(2);

  // Create a loop that will go until someone stops the program execution
  while (rclcpp::ok()) {
    
    // Publish the message within the 'message' variable
    publisher->publish(*message);
    // Increment the 'message' variable
    message->data++;
    rclcpp::spin_some(node);
    // Make sure the publish rate maintains at 2 Hz
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}