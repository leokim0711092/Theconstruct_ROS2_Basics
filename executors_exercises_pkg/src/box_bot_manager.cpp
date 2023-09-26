#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include <geometry_msgs/msg/point.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <unistd.h>

// For the parameters, templates of seconds or ms
using namespace std::chrono_literals;

class BoxBotManager : public rclcpp::Node {
public:
  BoxBotManager(std::string odom_topic_name1, std::string odom_topic_name2,
                std::string odom_topic_name3, geometry_msgs::msg::Point goal1,
                geometry_msgs::msg::Point goal2,
                geometry_msgs::msg::Point goal3)
      : Node("box_bot_manager_node") {

    this->goal1_ = goal1;
    this->goal2_ = goal2;
    this->goal3_ = goal3;

    RCLCPP_INFO(this->get_logger(), "INIT GOAL 1 >>>>>>>>>>>>>>>['%f','%f']",
                this->goal1_.x, this->goal1_.y);
    RCLCPP_INFO(this->get_logger(), "INIT GOAL 2 >>>>>>>>>>>>>>>['%f','%f']",
                this->goal2_.x, this->goal2_.y);
    RCLCPP_INFO(this->get_logger(), "INIT  GOAL 3 >>>>>>>>>>>>>>>['%f','%f']",
                this->goal3_.x, this->goal3_.y);

    timer_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    odom1_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    odom2_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    odom3_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options1;
    options1.callback_group = odom1_callback_group_;
    rclcpp::SubscriptionOptions options2;
    options2.callback_group = odom2_callback_group_;
    rclcpp::SubscriptionOptions options3;
    options3.callback_group = odom3_callback_group_;

    subscription1_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_name1, 10,
        std::bind(&BoxBotManager::topic1_callback, this, std::placeholders::_1),
        options1);

    subscription2_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_name2, 10,
        std::bind(&BoxBotManager::topic2_callback, this, std::placeholders::_1),
        options2);

    subscription3_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_name3, 10,
        std::bind(&BoxBotManager::topic3_callback, this, std::placeholders::_1),
        options3);

    this->wait_time = 1.0;
    timer_ = this->create_wall_timer(
        500ms, std::bind(&BoxBotManager::timer_callback, this),
        timer_callback_group_);

    // Publishers
    publisher1_ = this->create_publisher<std_msgs::msg::String>(
        "/reached_goal/box_bot_1", 1);
    publisher2_ = this->create_publisher<std_msgs::msg::String>(
        "/reached_goal/box_bot_2", 1);
    publisher3_ = this->create_publisher<std_msgs::msg::String>(
        "/reached_goal/box_bot_3", 1);
  }

private:
  void topic1_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    this->box_bot_1_reached_goal =
        check_reached_goal(this->goal1_, msg->pose.pose.position, "box_bot1");
  }
  void topic2_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    this->box_bot_2_reached_goal =
        check_reached_goal(this->goal2_, msg->pose.pose.position, "box_bot2");
  }
  void topic3_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    this->box_bot_3_reached_goal =
        check_reached_goal(this->goal3_, msg->pose.pose.position, "box_bot3");
  }

  void timer_callback() {
    RCLCPP_INFO(this->get_logger(), "Wating...TIMER CALLBACK");
    // Check if someone has reached the goal
    if (this->box_bot_1_reached_goal) {
      std_msgs::msg::String message;
      message.data =
          "BOX BOT 1 REACHED GOAL =" + std::to_string(this->goal1_.x) + "," +
          std::to_string(this->goal1_.y);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      this->publisher1_->publish(message);
    }

    if (this->box_bot_2_reached_goal) {
      std_msgs::msg::String message;
      message.data =
          "BOX BOT 2 REACHED GOAL =" + std::to_string(this->goal2_.x) + "," +
          std::to_string(this->goal2_.y);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      this->publisher2_->publish(message);
    }

    if (this->box_bot_3_reached_goal) {
      std_msgs::msg::String message;
      message.data =
          "BOX BOT 3 REACHED GOAL =" + std::to_string(this->goal3_.x) + "," +
          std::to_string(this->goal3_.y);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      this->publisher3_->publish(message);
    }

    sleep(this->wait_time);
    RCLCPP_INFO(this->get_logger(), "End TIMER CALLBACK");
  }

  void print_2Dposition(const geometry_msgs::msg::Point pos, std::string name) {

    RCLCPP_INFO(this->get_logger(), "%s=>[x,y]=['%f','%f']", name.c_str(),
                pos.x, pos.y);
  }

  bool check_reached_goal(const geometry_msgs::msg::Point goal,
                          const geometry_msgs::msg::Point current_pos,
                          std::string name, float delta_error = 0.1) {

    // print_2Dposition(goal, "GOAL");
    // print_2Dposition(current_pos, "CURRENT_POS");

    float delta_x = goal.x - current_pos.x;
    float delta_y = goal.y - current_pos.y;
    bool result = false;

    if ((delta_x <= delta_error) && (delta_x >= -1 * delta_error)) {
      if ((delta_y <= delta_error) && (delta_y >= -1 * delta_error)) {
        RCLCPP_INFO(this->get_logger(),
                    "IN GOAL[dx,dy,error]=['%f','%f','%f']=%s", delta_x,
                    delta_y, delta_error, name.c_str());
        result = true;
      } else {
        RCLCPP_INFO(this->get_logger(), "[DY,ERROR]=['%f','%f']=%s", delta_y,
                    delta_error, name.c_str());
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "[DX,ERROR]=['%f','%f']=%s", delta_x,
                  delta_error, name.c_str());
    }

    return result;
  }

  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

  rclcpp::CallbackGroup::SharedPtr odom1_callback_group_;
  rclcpp::CallbackGroup::SharedPtr odom2_callback_group_;
  rclcpp::CallbackGroup::SharedPtr odom3_callback_group_;

  rclcpp::TimerBase::SharedPtr timer_;
  float wait_time;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription1_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription2_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription3_;

  geometry_msgs::msg::Point goal1_;
  geometry_msgs::msg::Point goal2_;
  geometry_msgs::msg::Point goal3_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher1_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher2_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher3_;

  bool box_bot_1_reached_goal = false;
  bool box_bot_2_reached_goal = false;
  bool box_bot_3_reached_goal = false;
};

int main(int argc, char *argv[]) {
  // Some initialization.
  rclcpp::init(argc, argv);

  // Instantiate a Node.

  geometry_msgs::msg::Point goal1;
  geometry_msgs::msg::Point goal2;
  geometry_msgs::msg::Point goal3;

  goal1.x = 2.498409;
  goal1.y = -1.132045;

  goal2.x = 0.974281;
  goal2.y = -1.132045;

  goal3.x = -0.507990;
  goal3.y = -1.132045;

  std::shared_ptr<BoxBotManager> box_manager_node =
      std::make_shared<BoxBotManager>("/box_bot_1/odom", "/box_bot_2/odom",
                                      "/box_bot_3/odom", goal1, goal2, goal3);

  // This is the same as a print in ROS
  RCLCPP_INFO(box_manager_node->get_logger(), "BoxBotManagerNode INFO...");

  // Start and spin executor SingleThreadded
  //   rclcpp::spin(node);

  // Same code, but in steps
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(box_manager_node);
  executor.spin();

  // Shut down and exit.
  rclcpp::shutdown();
  return 0;
}