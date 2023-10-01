#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/my_custom_service_message.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <memory>

using MyCustomServiceMessage = custom_interfaces::srv::MyCustomServiceMessage;
using std::placeholders::_1;
using std::placeholders::_2;

class ServerNode : public rclcpp::Node
{
public:
  ServerNode()
  : Node("movement_server")
  {

    srv_ = create_service<MyCustomServiceMessage>("movement", std::bind(&ServerNode::moving_callback, this, _1, _2));
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  }

private:
  rclcpp::Service<MyCustomServiceMessage>::SharedPtr srv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  void moving_callback(
      const std::shared_ptr<MyCustomServiceMessage::Request> request,
      const std::shared_ptr<MyCustomServiceMessage::Response>
          response) 
    {

        auto message = geometry_msgs::msg::Twist();
    
        if (request->move == "Turn Right")
        {   
            // Send velocities to move the robot to the right
            message.linear.x = 0.2;
            message.angular.z = -0.2;
            publisher_->publish(message);

            // Set the response success variable to true
            response->success = true;
        }
        else if (request->move == "Turn Left")
        {
            // Send velocities to stop the robot
            message.linear.x = 0.2;
            message.angular.z = 0.2;
            publisher_->publish(message);

            // Set the response success variable to false
            response->success = true;
        }
        else if (request->move == "Stop")
        {
            // Send velocities to stop the robot
            message.linear.x = 0.0;
            message.angular.z = 0.0;
            publisher_->publish(message);

            // Set the response success variable to false
            response->success = true;
        }
        else {
            response->success = false;
        }
                
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServerNode>());
  rclcpp::shutdown();
  return 0;
}