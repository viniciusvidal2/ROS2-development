#include "robot_follow_wall/srv/find_wall.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <inttypes.h>
#include <memory>
#include <thread>

using mysrv = robot_follow_wall::srv::FindWall;

rclcpp::Node::SharedPtr g_node = nullptr;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
float front_distance = 10;
int minimum_index = 1000;
int state_machine = 0;
int front_index = 180, right_index = 90;

int find_min(std::vector<float> arr) {
  float low = 10000;
  int ind = 1000;
  for (size_t i = 0; i < arr.size(); i++) {
    if (arr[i] < low) {
      ind = i;
      low = arr[i];
    }
  }

  return ind;
}

void operate_state_machine(){
    geometry_msgs::msg::Twist msg_out;
    rclcpp::WallRate r(10);
    state_machine = 0;
    while (state_machine == 0){
        RCLCPP_DEBUG(g_node->get_logger(), "Rotating to find wall in front : %d", minimum_index);
        if (std::abs(minimum_index - front_index) > 10) {
            msg_out.angular.z = 0.3;
        } else {
            msg_out.angular.z = 0;
            state_machine = 1;
        }
        pub->publish(msg_out);
        r.sleep();
    }
    while (state_machine == 1){
        RCLCPP_DEBUG(g_node->get_logger(), "The distance to the wall in front: %.2f", front_distance);
        if (front_distance > 0.3) {
            msg_out.linear.x = 0.1;
        } else {
            msg_out.linear.x = 0;
            state_machine = 2;
        }
        pub->publish(msg_out);
        r.sleep();
    }
    while (state_machine == 2){
        RCLCPP_DEBUG(g_node->get_logger(), "Rotating so the wall is on our right side: %d", minimum_index);
        if (std::abs(minimum_index - right_index) > 7) {
            msg_out.angular.z = 0.3;
        } else {
            msg_out.angular.z = 0;
            state_machine = 3;
        }
        pub->publish(msg_out);
        r.sleep();
    }
}

void my_handle_service(
    const std::shared_ptr<rmw_request_id_t> request_header, 
    mysrv::Request::SharedPtr request, mysrv::Response::SharedPtr response) {
    (void)request_header;
    (void)request;
    RCLCPP_INFO(g_node->get_logger(), "Moving Robot to the closest wall");

    // Create publisher here, destroy after so there is no conflict with the controller code
    pub = g_node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    // Work the state machine all the way to the end
    operate_state_machine();
    pub = nullptr;

    response->wallfound = true;

    RCLCPP_DEBUG(g_node->get_logger(), "END OF STATE MACHINE FOR ROBOT POSITIONING, ANSWERING THE CLIENT ...");
}

void laser_callback(sensor_msgs::msg::LaserScan::SharedPtr msg) {
    switch (state_machine) {
    case 0: // rotating so the wall is in front
        minimum_index = find_min(msg->ranges);
        break;
    case 1: // Moving forward
        front_distance = msg->ranges[front_index];
        break;
    case 2: // Rotating so the wall is at the right side
        minimum_index = find_min(msg->ranges);
        break;
    default:
        break;
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);   
 
    // Main node
    g_node = rclcpp::Node::make_shared("find_wall_server");
    auto server = g_node->create_service<mysrv>("place_robot", my_handle_service);
    rcutils_logging_set_logger_level(g_node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);

    // Thread parallel node for the subscriber not to be blocked
    rclcpp::executors::SingleThreadedExecutor exec_subscriber;
    auto node_subscriber = rclcpp::Node::make_shared("node_subscriber");
    auto sub = node_subscriber->create_subscription<sensor_msgs::msg::LaserScan>("scan", 100, &laser_callback);
    exec_subscriber.add_node(node_subscriber);
    // Spin the subscriber executor in a separate thread
    std::thread spinThread([&exec_subscriber]() {
        exec_subscriber.spin();
    });

    // Spin main thread
    rclcpp::spin(g_node);        
  
    rclcpp::shutdown();
    g_node = nullptr;
    return 0;
}