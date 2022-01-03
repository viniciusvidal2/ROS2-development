#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_follow_wall/srv/find_wall.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_follow_wall/action/odom_record.hpp"

using std::placeholders::_1;

/// Node class
class RobotControlNode : public rclcpp::Node {
public:
    using mysrv = robot_follow_wall::srv::FindWall;
    using Orec = robot_follow_wall::action::OdomRecord;
    using GoalHandleOrec = rclcpp_action::ClientGoalHandle<Orec>;
        
    RobotControlNode() : Node("robot_controller") {
        rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);
        RCLCPP_INFO(this->get_logger(), "Starting control node ...");

        // Creating the client for the server to put the robot by the wall
        client_server_ = this->create_client<mysrv>("place_robot");
        auto request = std::make_shared<mysrv::Request>();
        while (!client_server_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

        auto result = this->send_request(client_server_, request);
        if (result) {
            auto result_str = result->wallfound ? "True" : "False";
            RCLCPP_INFO(this->get_logger(), "Result-Success : %s", result_str);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for response. Exiting.");
        }

        // Action client to start measuring the odometry and receive it at the end of the path
        this->client_action_ = rclcpp_action::create_client<Orec>(
                this->get_node_base_interface(),
                this->get_node_graph_interface(),
                this->get_node_logging_interface(),
                this->get_node_waitables_interface(),
                "action_server");

        this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
            std::bind(&RobotControlNode::send_goal, this));

        // Control node by itself        
        pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 100, std::bind(&RobotControlNode::wall_listener, this, _1));
    }

    void send_goal(){
        using namespace std::placeholders;

        this->timer_->cancel();
        this->goal_done_ = false;

        if (!this->client_action_) {
            RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
        }
        if (!this->client_action_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            this->goal_done_ = true;
            return;
        }

        auto goal_msg = Orec::Goal();
        goal_msg.goal = 1;
        RCLCPP_INFO(this->get_logger(), "Sending goal to action server ...");

        auto send_goal_options = rclcpp_action::Client<Orec>::SendGoalOptions();
        send_goal_options.goal_response_callback =
        std::bind(&RobotControlNode::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
        std::bind(&RobotControlNode::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
        std::bind(&RobotControlNode::result_callback, this, _1);
        auto goal_handle_future = this->client_action_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    mysrv::Response::SharedPtr send_request(rclcpp::Client<mysrv>::SharedPtr client, mysrv::Request::SharedPtr request) {
        auto result = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::executor::FutureReturnCode::SUCCESS){
            rclcpp::WallRate r(2);
            while(result.get()->wallfound != true){
                RCLCPP_DEBUG(this->get_logger(), "Waiting for service to be completed ...");
                r.sleep();
            }
        }        
        return result.get();
    }

    void wall_listener(sensor_msgs::msg::LaserScan::SharedPtr msg) {
        RCLCPP_DEBUG(this->get_logger(), "Measurements:  [0] %.2f   [90] %.2f   [180] %.2f   [270] %.2f", msg->ranges[0], msg->ranges[90],msg->ranges[180],msg->ranges[270]);
        // Check the distance to the wall
        RCLCPP_DEBUG(this->get_logger(), "Distance to the wall on the right: %.2f\n", msg->ranges[90]);
        // Populate the msg according to the distance
        float turn = 0.08, front = 0.045, angular = 0.45;
        if (msg->ranges[90] > 0.30) {
            msg_vel.angular.z = -angular;
            msg_vel.linear.x = turn;
        } else if (msg->ranges[90] < 0.20) {
            msg_vel.angular.z = angular;
            msg_vel.linear.x = turn/2;
        } else {
            if (msg->ranges[135] > msg->ranges[45]){
                msg_vel.angular.z = -angular;
            } else {
                msg_vel.angular.z = angular;
            }
            msg_vel.linear.x = front;
        }
        if(msg->ranges[180] < 0.3){
            msg_vel.linear.x = 0;
            msg_vel.angular.z = angular;
        }
        // Check if there is any obstacle that needs more rotation to be avoided
        if (*std::min_element(msg->ranges.begin() + 135, msg->ranges.begin() + 180) <= 0.15){
            msg_vel.linear.x = 0;
            msg_vel.angular.z = angular*3;
        }
        if (*std::min_element(msg->ranges.begin() + 180, msg->ranges.begin() + 225) <= 0.15){
            msg_vel.linear.x = 0;
            msg_vel.angular.z = -angular*3;
        }
        // Publish the message to control the robot
        pub->publish(msg_vel);
    }

    void goal_response_callback(std::shared_future<GoalHandleOrec::SharedPtr> future){
        auto goal_handle = future.get();
        if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(GoalHandleOrec::SharedPtr, const std::shared_ptr<const Orec::Feedback> feedback){
        RCLCPP_DEBUG(this->get_logger(), "Feedback received: %.2f", feedback->current_total);
    }

    void result_callback(const GoalHandleOrec::WrappedResult & result){
        this->goal_done_ = true;
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "GOAL SUCCEDED !");
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

        std::vector<geometry_msgs::msg::Point32> odom_list;
        odom_list = result.result->list_of_odoms;
        RCLCPP_INFO(this->get_logger(), "Result received! ");
        msg_vel.angular.z = 0;
        msg_vel.linear.x = 0;
        pub->publish(msg_vel);
        rclcpp::shutdown();
    }
        
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
    rclcpp::Client<mysrv>::SharedPtr client_server_;
    rclcpp_action::Client<Orec>::SharedPtr client_action_;
    geometry_msgs::msg::Twist msg_vel;
    rclcpp::TimerBase::SharedPtr timer_;
    bool goal_done_;
};

/// Main
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    rclcpp::spin(std::make_shared<RobotControlNode>());
    
    rclcpp::shutdown();
    return 0;
}