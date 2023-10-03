#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/server.hpp"
#include "rclcpp_action/types.hpp"
#include <actions_quiz_msg/action/distance.hpp>
#include <functional>
#include <math.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <actions_quiz_msg/msg/distance.hpp>
#include <cmath>



class ActionServer : public rclcpp::Node{

    public:
        using Dist = actions_quiz_msg::action::Distance;
        using GoalHandleDist = rclcpp_action::ServerGoalHandle<Dist>;
        explicit ActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions() ): 
        Node("action_server_node",options){

            this->action_ser_ = rclcpp_action::create_server<actions_quiz_msg::action::Distance>(
                this,
                "/distance_as",
                std::bind(&ActionServer::goal_rep, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&ActionServer::goal_can, this, std::placeholders::_1),
                std::bind(&ActionServer::handle_accepted, this, std::placeholders::_1));
            
            pub_ = this->create_publisher<actions_quiz_msg::msg::Distance>("/total_distance",10);
            sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom",10, std::bind(&ActionServer::sub_callback,this,std::placeholders::_1));
        }
    private:
        rclcpp_action::Server<actions_quiz_msg::action::Distance>::SharedPtr action_ser_ ;
        rclcpp::Publisher<actions_quiz_msg::msg::Distance>::SharedPtr pub_ ;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_ ;
        std::shared_ptr<nav_msgs::msg::Odometry> odom_msg ;

        rclcpp_action::GoalResponse goal_rep(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Dist::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(), "Received goal request with secs %d", goal->seconds);
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; 
        }
        
        rclcpp_action::CancelResponse goal_can(const std::shared_ptr<GoalHandleDist> go_cancel)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            (void)go_cancel;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandleDist> goal_handle)
        {
            using namespace std::placeholders;
            // this needs to return quickly to avoid blocking the executor, so spin up a new thread
            std::thread{std::bind(&ActionServer::execute, this, _1), goal_handle}.detach();
        }
        void execute(const std::shared_ptr<GoalHandleDist> goal_h){
            RCLCPP_INFO(this->get_logger(), "Executing goal");
            const auto goal = goal_h->get_goal();
            auto feedback = std::make_shared<Dist::Feedback>();
            auto result = std::make_shared<Dist::Result>();
            actions_quiz_msg::msg::Distance total_distance;
            total_distance.dist = 0;
            rclcpp::Rate loop_rate(1);
            nav_msgs::msg::Odometry odom_msg_store_previous;

            for (int i = 0; (i < goal->seconds) && rclcpp::ok(); ++i) {

                if (goal_h->is_canceling()) {
                    goal_h->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "Goal canceled");
                    return;
                }
                // Move robot forward and send feedback
                if(i== 0) feedback->current_dist = std::sqrt(std::pow(odom_msg->pose.pose.position.x,2)+std::pow(odom_msg->pose.pose.position.y,2));
                else feedback->current_dist += calculate_odom(odom_msg, odom_msg_store_previous);

                odom_msg_store_previous = *odom_msg;
                
                goal_h->publish_feedback(feedback);
                RCLCPP_INFO(this->get_logger(), "Publish feedback: %f", feedback->current_dist);

                total_distance.dist = feedback->current_dist;
                loop_rate.sleep();
            }
            if(rclcpp::ok()){
                pub_->publish(total_distance);
                result->total_dist = total_distance.dist+calculate_odom(odom_msg, odom_msg_store_previous);
                result->status = true;
                if(result->status == true) RCLCPP_INFO(this->get_logger(), "Publish result: status: succeed, totoal distance: %f", result->total_dist);
                goal_h->succeed(result);
            }
        }

        void sub_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
            this->odom_msg = msg;
        }

        float calculate_odom(const nav_msgs::msg::Odometry::SharedPtr odom_now, nav_msgs::msg::Odometry odom_previous){

            float result = std::sqrt(std::pow(odom_now->pose.pose.position.x-odom_previous.pose.pose.position.x,2)+std::pow(odom_now->pose.pose.position.y-odom_previous.pose.pose.position.y,2));
            return result;
        }

      

};
int main(int argc, char** argv){
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<ActionServer>();
    
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;

}