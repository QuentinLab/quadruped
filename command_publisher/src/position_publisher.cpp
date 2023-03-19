#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>
#include <trajectory_msgs/msg/detail/joint_trajectory__struct.hpp>
#include <vector>

class PositionPublisher : public rclcpp::Node {
    public:
        PositionPublisher() :
            Node("position_publisher")
    {
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_trajectory_controller/joint_trajectory", 10);
    };

        void publishMessage()
        {
            trajectory_msgs::msg::JointTrajectory JtMsg;
            std::vector<trajectory_msgs::msg::JointTrajectoryPoint> JtPointTrajectory;
            trajectory_msgs::msg::JointTrajectoryPoint JtPoint;
            std::vector<double> positions;
            std_msgs::msg::Header header;

            // Set message header with correct timestamp
            header.set__stamp(this->get_clock()->now());
            header.set__frame_id("unknown");
            JtMsg.set__header(header);

            //Create a vector with one position
            positions.push_back(1.7);
            JtPoint.set__positions(positions);
            JtPoint.set__time_from_start(rclcpp::Duration(0,500000000));

            //Push position to position vector
            JtPointTrajectory.push_back(JtPoint);
            //Add to message
            JtMsg.set__points(JtPointTrajectory);
            JtMsg.set__joint_names({"dof_joint"});
            //Publish message
            publisher_->publish(JtMsg);

        }

        void publishVelocity()
        {

            auto publisher2 = this->create_publisher<std_msgs::msg::Float64MultiArray>(
                    "/velocity_controller/commands", 10);


            std_msgs::msg::Float64MultiArray commands;

            using namespace std::chrono_literals;
            commands.data.push_back(0);
            publisher2->publish(commands);
            std::this_thread::sleep_for(1s);

            commands.data[0] = 1;
            publisher2->publish(commands);
            std::this_thread::sleep_for(1s);

            commands.data[0] = -1;
            publisher2->publish(commands);
            std::this_thread::sleep_for(1s);

            commands.data[0] = 0;
            publisher2->publish(commands);
            std::this_thread::sleep_for(1s);
        }

    private:
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    PositionPublisher myNode;
    myNode.publishMessage();
}

