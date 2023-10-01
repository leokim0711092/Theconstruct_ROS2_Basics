#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/utilities.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <services_quiz_srv/srv/spin.hpp>
#include <geometry_msgs/msg/twist.hpp>

class ServerNode : public rclcpp::Node{

    public:
        ServerNode(): Node("services_server"){
            srv_ = create_service<services_quiz_srv::srv::Spin>("/rotate",std::bind(&ServerNode::rotate_callback, this, std::placeholders::_1, std::placeholders::_2));
            pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
        }
    
    private:
        rclcpp::Service<services_quiz_srv::srv::Spin>::SharedPtr srv_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        int t=0;
        geometry_msgs::msg::Twist mes;
        void rotate_callback(const std::shared_ptr<services_quiz_srv::srv::Spin::Request> req, 
        const std::shared_ptr<services_quiz_srv::srv::Spin::Response> res){
            
            if(req->direction =="right") mes.angular.z = req->angular_velocity*-1;
            else if (req->direction =="left") mes.angular.z = req->angular_velocity;
           

            for (int i = 0; i <= req->time; i++) {
            this->pub_->publish(mes);
            std::this_thread::sleep_for(std::chrono::seconds(1));
            }

        // set velocity to zero to stop the robot
            mes.angular.z = 0.0;
            pub_->publish(mes);
             }   

            res->success = true;
};
int main(int argc, char** argv){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ServerNode>());
    rclcpp::shutdown();
    return 0;
}