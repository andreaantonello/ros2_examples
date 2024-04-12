#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/float32_multi_array.hpp"

class CrabPublisher : public rclcpp::Node {
public:
    CrabPublisher() : Node("crab_publisher") {
        publisher_ = this->create_publisher<example_interfaces::msg::Float32MultiArray>("crab_publisher", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(5), 
                std::bind(&CrabPublisher::publishMessage, this));
        RCLCPP_INFO(this->get_logger(), "Crab Publisher has started");
    }

private:
    rclcpp::Publisher<example_interfaces::msg::Float32MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publishMessage() {
        auto msg = example_interfaces::msg::Float32MultiArray();
        msg.data = {getTimestamp(), getRandomFloat()};
        publisher_->publish(msg);
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
