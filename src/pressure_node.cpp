#include "pressure_pkg/pressure_node.hpp"
#include "pressure_pkg/pressure_driver.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
    : Node("minimal_publisher"), count_(0), pressure_driver(std::make_shared<pressure_pkg::PressureDriver>())
  {
    // Corrigido para criar o publisher de Float64
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::Float64();
    
    // Chame getPressure() na instância de pressure_driver
    message.data = pressure_driver->getPressure();
    
    // Usar %f para imprimir o valor do tipo double
    RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.data);
    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
  std::shared_ptr<pressure_pkg::PressureDriver> pressure_driver;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
