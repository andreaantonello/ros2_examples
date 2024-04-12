#include "rclcpp/rclcpp.hpp"
#include "custom_crab_interfaces/msg/joint_telemetry.hpp"
#include "custom_crab_interfaces/msg/robot_telemetry.hpp"


class CrabPublisher : public rclcpp::Node {
public:
    CrabPublisher() : Node("crab_publisher") {
        publisher_ = this->create_publisher<custom_crab_interfaces::msg::RobotTelemetry>("crab_publisher", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), 
                std::bind(&CrabPublisher::publishMessage, this));
        RCLCPP_INFO(this->get_logger(), "Crab Publisher has started");
    }

private:
    rclcpp::Publisher<custom_crab_interfaces::msg::RobotTelemetry>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publishMessage() {
        auto message = custom_crab_interfaces::msg::RobotTelemetry();
        message.timestamp = this->now();  // Set current ROS time as the timestamp

        // Simulated telemetry data for 6 sets of joint telemetry
        for (int i = 0; i < 6; i++) {
            custom_crab_interfaces::msg::JointTelemetry joint_data;
            joint_data.joint_number = i;
            joint_data.input_encoder_radians = getRandomFloat();
            joint_data.output_encoder_radians = getRandomFloat();
            joint_data.input_velocity = getRandomFloat();
            joint_data.output_velocity = getRandomFloat();
            joint_data.joint_current = getRandomFloat();

            message.data[i] = joint_data;
        }

        // Publish the message
        publisher_->publish(message);
    }

    float getTimestamp() {
        // Get current ROS time in seconds
        return this->now().seconds();
    }

    float getRandomFloat() {
        // Generate a random float for demonstration
        return static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 100.0;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CrabPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
