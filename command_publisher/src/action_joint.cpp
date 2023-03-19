#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/client_goal_handle.hpp>
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include <future>
#include <functional>
#include <memory>
#include <string>
#include <sstream>


class JointActionPublisher : public rclcpp::Node {
    using FollowJointTraj =  control_msgs::action::FollowJointTrajectory;
    using GoalFollow = rclcpp_action::ClientGoalHandle<FollowJointTraj>;
    public:
    JointActionPublisher() : Node("joint_action_publisher")
    {
        action_client_ = rclcpp_action::create_client<FollowJointTraj>(this,"/joint_trajectory_controller/follow_joint_trajectory");
        goal_state_ = false;
    };

    void send_goal()
    {
        if (!action_client_->wait_for_action_server()) 
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available");
            rclcpp::shutdown();
        }

        auto goal = FollowJointTraj::Goal();

        std::vector<std::string> joint_names = {"dof_joint"};
        std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points;

        trajectory_msgs::msg::JointTrajectoryPoint point;

        point.time_from_start = rclcpp::Duration::from_seconds(0.0);  // start asap
        point.positions.resize(joint_names.size());
        point.positions[0] = 1.7;

        trajectory_msgs::msg::JointTrajectoryPoint point2;
        point2.time_from_start = rclcpp::Duration::from_seconds(1.0);
        point2.positions.resize(joint_names.size());
        point2.positions[0] = 3.14;

        trajectory_msgs::msg::JointTrajectoryPoint point3;
        point3.time_from_start = rclcpp::Duration::from_seconds(2.0);
        point3.positions.resize(joint_names.size());
        point3.positions[0] = 4.84;

        trajectory_msgs::msg::JointTrajectoryPoint point4;
        point4.time_from_start = rclcpp::Duration::from_seconds(3.0);
        point4.positions.resize(joint_names.size());
        point4.positions[0] = 6.28;

        points.push_back(point);
        points.push_back(point2);
        points.push_back(point3);
        points.push_back(point4);

        rclcpp_action::Client<FollowJointTraj>::SendGoalOptions opt;

        opt.goal_response_callback = std::bind(&JointActionPublisher::common_goal_response,this, std::placeholders::_1);
        opt.feedback_callback = std::bind(&JointActionPublisher::common_feedback, this, std::placeholders::_1, std::placeholders::_2);
        opt.result_callback = std::bind(&JointActionPublisher::common_result_response, this, std::placeholders::_1);

        goal.goal_time_tolerance = rclcpp::Duration::from_seconds(1.0);
        goal.trajectory.joint_names = joint_names;
        goal.trajectory.points = points;

        auto goal_handle_future = action_client_->async_send_goal(goal, opt);
    }

    bool is_goal_done()
    {
        return goal_state_;
    }
    private:
    rclcpp_action::Client<FollowJointTraj>::SharedPtr action_client_;
    bool goal_state_;

    void common_goal_response(std::shared_future<GoalFollow::SharedPtr> future)
    {
        RCLCPP_INFO(this->get_logger(),"Getting into this function");
        GoalFollow::SharedPtr goal_response = future.get();
        if (!goal_response)
        {
            RCLCPP_ERROR(this->get_logger(), "Nope");
        }
        else 
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted!");
        }
    };

    void common_result_response(const rclcpp_action::ClientGoalHandle<FollowJointTraj>::WrappedResult &result)
    {
        goal_state_ = true;
        if (result.result->error_code == control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL)
        {
            RCLCPP_INFO(this->get_logger(), "Result function is a success");
        }
        else 
        {
            RCLCPP_INFO(this->get_logger(), "Goal Could not be achieved, error code is : %d", result.result->error_code);
        }
    };

    void common_feedback(GoalFollow::SharedPtr , const std::shared_ptr<const FollowJointTraj::Feedback> feedback)
    {
        float current_position = feedback->actual.positions[0];
        float desired_position = feedback->desired.positions[0];
        float current_speed =  feedback->actual.velocities[0];
        float desired_speed = feedback->desired.velocities[0];
        RCLCPP_INFO(this->get_logger(), "Receiving feedback :");
        RCLCPP_INFO(this->get_logger(), "|          | Actual    | Desired   |");
        RCLCPP_INFO(this->get_logger(), "| Speed    | %f        | %f        |",current_speed,desired_speed);
        RCLCPP_INFO(this->get_logger(), "| Position | %f        | %f        |",current_position,desired_position);
    };

};

int main (int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    std::shared_ptr<JointActionPublisher> myNode = std::make_shared<JointActionPublisher>();
    myNode->send_goal();
    while (!myNode->is_goal_done())
    {
        rclcpp::spin_some(myNode);
    }
    return 0;
}
