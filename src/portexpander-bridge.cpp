#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
using std::placeholders::_1;

class Portexpander_I2C_Bridge : public rclcpp::Node
{
	std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> _subscriptions;
	uint8_t _portA, _portB;
	
  public:
    Portexpander_I2C_Bridge()
    : Node("portexpander_I2C_Bridge")
    {
		_subscriptions.reserve(2*8);
		const std::string base_name = "portexpander_I2C_bridge/";
		for(uint_fast8_t port = 0; port < 2; port++)
		{
			for(uint_fast8_t bit = 0; bit < 8; bit++)
			{
				std::string topic_name = base_name + (port == 0 ? "A" : "B") + std::to_string(bit);
				std::cout << "Registering topic " << topic_name << std::endl;
				std::function<void(const std_msgs::msg::Bool::SharedPtr)> kack = std::bind(&Portexpander_I2C_Bridge::topic_callback, this, _1, port, bit);
				_subscriptions.emplace_back(
										this->create_subscription<std_msgs::msg::Bool>(
											topic_name, 1, kack)
										);
			}
		}
		//TODO: Connect I2C addr 0x20
		// Set PA+PB to output
		// Set PA+PB to zero (Or read config for default value?)
    }

  private:
    void topic_callback(const std_msgs::msg::Bool::SharedPtr msg,
						const uint_fast8_t port, const uint_fast8_t bitpos)
    {
		assert(port < 2 && "Only Port A or B valid");
		RCLCPP_INFO(this->get_logger(), "Set port %s bit %u to %s",
				 port == 0 ? "A" : "B", bitpos, msg->data ? "1" : "0");
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Portexpander_I2C_Bridge>());
  rclcpp::shutdown();
  return 0;
}