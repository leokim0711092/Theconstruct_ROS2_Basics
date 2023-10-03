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
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <actions_quiz_msg/msg/distance.hpp>

class ActionClient : public rclcpp::Node{

    public:         
        using Dist = actions_quiz_msg::action::Distance;
        using GoalHandleDist = rclcpp_action::ClientGoalHandle<Dist>;

        explicit ActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
        : Node("action_client_node", node_options), goal_done_(false)
        {
            this->client_ptr_ = rclcpp_action::create_client<Dist>(
            this->get_node_base_interface(),
            this->get_node_graph_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "/distance_as");

                this->timer_ = this->create_wall_timer(
                std::chrono::milliseconds(500),
                std::bind(&ActionClient::send_goal, this));
        }

        bool is_goal_done() const
        {
            return this->goal_done_;
        }


        void send_goal()
        {
            using namespace std::placeholders;

            this->timer_->cancel();

            this->goal_done_ = false;

            if (!this->client_ptr_) {
            RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
            }

            if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            this->goal_done_ = true;
            return;
            }

            auto goal_msg = Dist::Goal();
            goal_msg.seconds = 20;

            RCLCPP_INFO(this->get_logger(), "Sending goal");

            auto send_goal_options = rclcpp_action::Client<Dist>::SendGoalOptions();
                        
            send_goal_options.goal_response_callback =
            std::bind(&ActionClient::goal_response_callback, this, _1);

            send_goal_options.feedback_callback =
            std::bind(&ActionClient::feedback_callback, this, _1, _2);

            send_goal_options.result_callback =
            std::bind(&ActionClient::result_callback, this, _1);
            
            auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
        }
    private:
        rclcpp_action::Client<Dist>::SharedPtr client_ptr_;
        rclcpp::TimerBase::SharedPtr timer_;
        bool goal_done_;

        void goal_response_callback(const std::shared_ptr<GoalHandleDist> goal_handle)
        {
            if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
            }
        }

        void feedback_callback(
            GoalHandleDist::SharedPtr,
            const std::shared_ptr<const Dist::Feedback> feedback)
        {
            RCLCPP_INFO(
            this->get_logger(), "Feedback received: %f", feedback->current_dist);
        }

        void result_callback(const GoalHandleDist::WrappedResult & result)
        {
            this->goal_done_ = true;
            switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Result received: status: succeed, total distance: %f", result.result->total_dist);

        }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<ActionClient>();
    
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_client);

  while (!action_client->is_goal_done()) {
    executor.spin();
  }

  rclcpp::shutdown();
  return 0;
}