#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

class MyCustomNode: public rclcpp::Node { //MODIFY NAME

public:
    SmartphoneNode(): Node("smartphone")
    {
    }

private:
};

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<SmartphoneNode>(); //MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}