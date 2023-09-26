#include "rcl/publisher.h"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/timer.hpp"
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class Detect_obstacle : public rclcpp::Node {
public:
  Detect_obstacle() : Node("topics_quiz_node") {
    publisher =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    sub =
        this->create_subscription<sensor_msgs::msg::LaserScan>("/scan",10,std::bind(&Detect_obstacle::avoidance_callback,this,std::placeholders::_1));
  }

private:
  void avoidance_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    rclcpp::WallRate lr(1);
    if(msg->ranges[360]<1){
        message.linear.x = 0;
        message.angular.z = 0.3;
        publisher->publish(message);
        lr.sleep();
        RCLCPP_INFO(this->get_logger(), "Middle < 1");
    }else if (msg->ranges[539]<1) {
        message.linear.x = 0;
        message.angular.z = 0.3;
        publisher->publish(message);
        lr.sleep();
        RCLCPP_INFO(this->get_logger(), "Right < 1");
    }else if (msg->ranges[139]<1) {
        message.linear.x = 0;
        message.angular.z = -0.3;
        publisher->publish(message);
        lr.sleep();
        RCLCPP_INFO(this->get_logger(), "Left < 1");
    }else {
        message.linear.x = 0.3;
        message.angular.z = 0;
        publisher->publish(message);
        lr.sleep();
        RCLCPP_INFO(this->get_logger(), "Middle>1");
    }

  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub;
  geometry_msgs::msg::Twist message;
  
};

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Detect_obstacle>());
  rclcpp::shutdown();
}