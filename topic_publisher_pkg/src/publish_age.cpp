#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/age.hpp"
#include <chrono>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class PublishAge : public rclcpp::Node
{
public:
  PublishAge()
  : Node("move_robot")
  {
    publisher_ = this->create_publisher<custom_interfaces::msg::Age>("age", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&PublishAge::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = custom_interfaces::msg::Age();
    message.years = 4;
    message.months = 11;
    message.days = 21;
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<custom_interfaces::msg::Age>::SharedPtr publisher_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PublishAge>());
  rclcpp::shutdown();
  return 0;
}