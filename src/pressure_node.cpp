#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "pressure_pkg/pressure_driver.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("Pressao", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&MinimalPublisher::timer_callback, this));
        
        // Inicializa o driver de pressão
        pressure_driver_ = std::make_shared<pressure_pkg::PressureDriver>();
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        float pressure_value = pressure_driver_->getPressure();
        message.data = "Pressão: " + std::to_string(pressure_value);
        
        RCLCPP_INFO(this->get_logger(), "Valor da pressão obtido: '%s'", message.data.c_str());
        publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    std::shared_ptr<pressure_pkg::PressureDriver> pressure_driver_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
