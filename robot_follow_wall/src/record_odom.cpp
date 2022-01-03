#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "robot_follow_wall/action/odom_record.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point32.hpp"

class OdomRecordActionServer : public rclcpp::Node
{
public:
    using Orec = robot_follow_wall::action::OdomRecord;
    using GoalHandleOrec = rclcpp_action::ServerGoalHandle<Orec>;

    explicit OdomRecordActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("odom_record_action_server", options)
    {
        using namespace std::placeholders;
        rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);

        this->action_server_ = rclcpp_action::create_server<Orec>(
        this,
        "action_server",
        std::bind(&OdomRecordActionServer::handle_goal, this, _1, _2),
        std::bind(&OdomRecordActionServer::handle_cancel, this, _1),
        std::bind(&OdomRecordActionServer::handle_accepted, this, _1));

        first_time = true;
        x_start = previous_odom.pose.pose.position.x, y_start = previous_odom.pose.pose.position.y;

        this->sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 100, std::bind(&OdomRecordActionServer::odom_callback, this, _1));

        total_walked = 0;
    }

private:
    void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg){
        if (first_time){
            previous_odom = *msg;
            first_time = false;
        }
        total_walked += std::sqrt(std::pow(msg->pose.pose.position.x - previous_odom.pose.pose.position.x, 2) + 
                                std::pow(msg->pose.pose.position.y - previous_odom.pose.pose.position.y, 2));
        previous_odom = *msg;
        float term1 = msg->pose.pose.orientation.w*msg->pose.pose.orientation.z;
        float term2 = msg->pose.pose.orientation.x*msg->pose.pose.orientation.y;
        float term3 = std::pow(msg->pose.pose.orientation.y, 2) + std::pow(msg->pose.pose.orientation.z, 2);
        orientation = std::atan2(2*(term1 + term2), 1 - 2*term3);
        // RCLCPP_INFO(this->get_logger(), "Our orientation: %.2f", orientation);
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Orec::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request to record odometry!");
        (void)uuid;
        (void)goal;

        x_start = previous_odom.pose.pose.position.x, y_start = previous_odom.pose.pose.position.y;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleOrec> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleOrec> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&OdomRecordActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleOrec> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal!");
        auto feedback = std::make_shared<Orec::Feedback>();
        auto result = std::make_shared<Orec::Result>();
        rclcpp::Rate loop_rate(1);

        while (first_time)
            loop_rate.sleep();
        RCLCPP_INFO(this->get_logger(), "Went past the first time, have origin set, now lets monitor the path!");
        x_start = previous_odom.pose.pose.position.x, y_start = previous_odom.pose.pose.position.y;
        geometry_msgs::msg::Point32 current_odom;
        while(rclcpp::ok()){

            // Add current odometry to the list
            // RCLCPP_INFO(this->get_logger(), "Recording odometry ...");
            current_odom.x = previous_odom.pose.pose.position.x;
            current_odom.y = previous_odom.pose.pose.position.y;
            current_odom.z = orientation;
            odom_list_.emplace_back(current_odom);
            // Publish how much we have already travelled
            feedback->current_total = total_walked;
            goal_handle->publish_feedback(feedback);        

            // Check if someone cancelled the execution
            if (goal_handle->is_canceling()) {
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            // Check if we have gotten to the start again
            float dist_start = std::sqrt(std::pow(previous_odom.pose.pose.position.x - x_start, 2) + \
                                    std::pow(previous_odom.pose.pose.position.y - y_start, 2));
            RCLCPP_DEBUG(this->get_logger(), "We are     at: x:  %.2f    y:  %.2f", previous_odom.pose.pose.position.x, previous_odom.pose.pose.position.y);
            RCLCPP_DEBUG(this->get_logger(), "We started at: x:  %.2f    y:  %.2f", x_start, y_start);
            if(dist_start < 0.3 && total_walked > 0.6){
                RCLCPP_INFO(this->get_logger(), "Reached the beginning again, finishing up ...");
                result->list_of_odoms = odom_list_;
                goal_handle->succeed(result);
            }
            
            loop_rate.sleep();

        }
    }

    // Variables
    rclcpp_action::Server<Orec>::SharedPtr action_server_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    float total_walked, orientation;
    nav_msgs::msg::Odometry previous_odom;
    std::vector<geometry_msgs::msg::Point32> odom_list_;
    bool first_time;
    float x_start, y_start;

};  // class OdomRecordActionServer

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto action_server = std::make_shared<OdomRecordActionServer>();

    rclcpp::spin(action_server);

    rclcpp::shutdown();
    return 0;
}