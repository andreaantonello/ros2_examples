#include "rclcpp/rclcpp.hpp"
#include "custom_crab_interfaces/msg/joint_telemetry.hpp"
#include "custom_crab_interfaces/msg/robot_telemetry.hpp"


class CrabPublisher : public rclcpp::Node {
public:
    CrabPublisher() : Node("crab_publisher") {
        publisher_ = this->create_publisher<custom_crab_interfaces::msg::RobotTelemetry>("crab_publisher", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(7), 
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
            joint_data.input_encoder_radians = getSinusoidalValue(i, "input_encoder", message.timestamp);
            joint_data.output_encoder_radians = getSinusoidalValue(i, "output_encoder", message.timestamp);
            joint_data.input_velocity = getSinusoidalValue(i, "input_velocity", message.timestamp);
            joint_data.output_velocity = getSinusoidalValue(i, "output_velocity", message.timestamp);
            joint_data.joint_current = getSinusoidalValue(i, "joint_current", message.timestamp);

            message.data[i] = joint_data;
        }

        // Publish the message
        publisher_->publish(message);
    }

    float getSinusoidalValue(int joint_number, const std::string &field_name, const builtin_interfaces::msg::Time &timestamp) {
        // Use sinusoidal functions to generate telemetry values based on joint number and field name
        double amplitude = 1.0; // Set amplitude of sinusoidal function
        double frequency = 0.5; // Set frequency of sinusoidal function (adjust as needed)

        // Calculate time in seconds
        double time_sec = timestamp.sec + 1e-9 * timestamp.nanosec;

        if (field_name == "input_encoder") {
            return amplitude * joint_number * std::sin(frequency * time_sec);
        }
        else if (field_name == "output_encoder") {
            return amplitude * joint_number * std::cos(frequency * time_sec);
        }
        else if (field_name == "input_velocity") {
            return amplitude * joint_number * std::sin(2 * frequency * time_sec);
        }
        else if (field_name == "output_velocity") {
            return amplitude * joint_number * std::cos(2 * frequency * time_sec);
        }
        else if (field_name == "joint_current") {
            return amplitude * joint_number *std::sin(0.5 * frequency * time_sec);
        }
        else {
            return 0.0; // Default value (should not occur)
        }
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
