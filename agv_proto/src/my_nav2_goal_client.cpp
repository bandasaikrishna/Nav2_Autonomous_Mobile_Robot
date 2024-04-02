#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class NavigateToPoseClient : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit NavigateToPoseClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("navigate_to_pose_client", options)
  {
  
    
    goal_msg = NavigateToPose::Goal();
    
    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/my_goal_pose", 10, std::bind(&NavigateToPoseClient::goal_from_web_UI, this, std::placeholders::_1));
      
    subscription2_ = this->create_subscription<std_msgs::msg::UInt8>(
      "/cancel_from_web_UI", 10, std::bind(&NavigateToPoseClient::cancel_from_web_UI, this, std::placeholders::_1));
    
    // Wait for the action server to be available
    while (!client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_INFO(get_logger(), "Waiting for the 'navigate_to_pose' action server to be available...");
    }
  }

void sendGoal()
{
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&NavigateToPoseClient::goalResponseCallback, this, std::placeholders::_1);
    send_goal_options.result_callback =
        std::bind(&NavigateToPoseClient::resultCallback, this, std::placeholders::_1);

    auto future_goal_handle = client_->async_send_goal(goal_msg, send_goal_options);
    /*if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_goal_handle) !=
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "Failed to send goal");
    }*/
}


  void cancelGoal()
  {
    if (goal_handle_) {
      auto future_cancel = client_->async_cancel_goal(goal_handle_);
      if (/*rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_cancel) !=
        rclcpp::executor::FutureReturnCode::SUCCESS*/0)
      {
        RCLCPP_ERROR(get_logger(), "Failed to cancel goal");
      } else {
        RCLCPP_INFO(get_logger(), "Goal canceled");
      }
    } else {
      RCLCPP_INFO(get_logger(), "No active goal to cancel");
    }
    goal_handle_ = NULL;
  }

private:


  void goal_from_web_UI(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
  
    RCLCPP_INFO(this->get_logger(), "received a goal_pose");
    // Access goals using an index

    
        // Set your goal parameters here
        goal_msg.pose.header.frame_id = "map"; // Replace "map" with the desired frame_id
        goal_msg.pose.pose.position.x = msg->pose.position.x;    // Replace with your x coordinate
        goal_msg.pose.pose.position.y = msg->pose.position.y;    // Replace with your y coordinate
        goal_msg.pose.pose.position.z = msg->pose.position.z;    // Replace with your z coordinate
        goal_msg.pose.pose.orientation.x = msg->pose.orientation.x; // Replace with your quaternion orientation
        goal_msg.pose.pose.orientation.y = msg->pose.orientation.y; // Replace with your quaternion orientation
        goal_msg.pose.pose.orientation.z = msg->pose.orientation.z; // Replace with your quaternion orientation
        goal_msg.pose.pose.orientation.w = msg->pose.orientation.w; // Replace with your quaternion orientation
    
    
       this->sendGoal();
    
    
  }
  
  void cancel_from_web_UI(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    this->cancelGoal();
    
    RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->data);
  }
  
  void goalResponseCallback(std::shared_future<GoalHandleNavigateToPose::SharedPtr> future)
  {
    goal_handle_ = future.get();
    if (!goal_handle_) {
      RCLCPP_ERROR(get_logger(), "Goal was rejected by the action server");
    } else {
      RCLCPP_INFO(get_logger(), "Goal accepted by the action server");
    }
  }

  void resultCallback(const GoalHandleNavigateToPose::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Goal succeeded");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(), "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(get_logger(), "Goal was canceled");
        break;
      default:
        RCLCPP_ERROR(get_logger(), "Unknown result code");
        break;
    }
  }

  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  GoalHandleNavigateToPose::SharedPtr goal_handle_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscription2_;
  NavigateToPose::Goal goal_msg;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigateToPoseClient>();

  // Sending a goal
  //node->sendGoal();

  // Waiting for a moment (replace this with your application logic)
  //rclcpp::sleep_for(std::chrono::seconds(5));

  // Canceling the goal
  //node->cancelGoal();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
