#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
using std::placeholders::_1;

class OdomSubscriber : public rclcpp::Node
{
public:
  OdomSubscriber()
  : Node("simple_subscriber")
  {
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&OdomSubscriber::odom_callback, this, _1));
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->pose.pose.position.x);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomSubscriber>());
  rclcpp::shutdown();
  return 0;
}