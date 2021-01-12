#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
using std::placeholders::_1;


#ifdef __linux__ 
    #include <unistd.h>				//Needed for I2C port
	#include <fcntl.h>				//Needed for I2C port
	#include <sys/ioctl.h>			//Needed for I2C port
	#include <linux/i2c-dev.h>		//Needed for I2C port
#else
   #pragma message("warning: Windows lol, best I can do is mock-up")
	#define MOCKUP
#endif

class Portexpander_I2C_Bridge : public rclcpp::Node
{
	std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> _subscriptions;
	struct State
	{
		uint8_t addr;
		uint8_t ports[2];	//A and B
	} _state;
#ifndef MOCKUP
	FILE i2c_bus;
#endif
	
	static constexpr uint8_t IODIRA = 0x00;
	static constexpr uint8_t IODIRB = 0x01;
	static constexpr uint8_t GPIOPA = 0x14;
	static constexpr uint8_t GPIOPB = 0x15;
	
  public:
    Portexpander_I2C_Bridge()
    : Node("portexpander_I2C_Bridge")
    {
		memset(&_state, 0, sizeof(State));
#ifndef MOCKUP
		//----- OPEN THE I2C BUS -----
		std::string filename = "/dev/i2c-2";		//TODO: Read configuration
		if ((i2c_bus = open(filename, O_RDWR)) < 0)
		{
			perror("Failed to open the i2c bus");
			return;
		}

		int addr = 0x20;
		if (ioctl(i2c_bus, I2C_SLAVE, addr) < 0)
		{
			perror("Failed to acquire bus access and/or talk to slave.\n");
			return;
		}
		
		//TODO: Set PA+PB to zero (Or read config for default value?)
		
		// Set PA+PB to output
		_state.addr = IODIRA;
		if (write(i2c_bus, &_state, sizeof(State)) != sizeof(State))
		{
			/* ERROR HANDLING: i2c transaction failed */
			std::cerr << "Failed to write to the i2c bus (" << i2c_bus << ")" << std::endl;
		}
#endif
		
		
		_subscriptions.reserve(2*8);
		const std::string base_name = "portexpander_I2C_bridge/";
		for(uint_fast8_t port = 0; port < 2; port++)
		{
			for(uint_fast8_t bit = 0; bit < 8; bit++)
			{
				std::string topic_name = base_name + (port == 0 ? "A" : "B") + std::to_string(bit);
				//std::cout << "Registering topic " << topic_name << std::endl;
				std::function<void(const std_msgs::msg::Bool::SharedPtr)> kack = std::bind(&Portexpander_I2C_Bridge::topic_callback, this, _1, port, bit);
				_subscriptions.emplace_back(
										this->create_subscription<std_msgs::msg::Bool>(
											topic_name, 1, kack)
										);
			}
		}
    }

  private:
    void topic_callback(const std_msgs::msg::Bool::SharedPtr msg,
						const uint_fast8_t port, const uint_fast8_t bitpos)
    {
		assert(port < 2 && "Only Port A or B valid");
		
		RCLCPP_INFO(this->get_logger(), "Set port %s bit %u to %s",
				 port == 0 ? "A" : "B", bitpos, msg->data ? "1" : "0");
		if(msg->data)
			_state.ports[port] |= 1 << bitpos;
		else
			_state.ports[port] &= ~(1 << bitpos);
#ifndef MOCKUP
		_state.addr = GPIOPA;
		if (write(i2c_bus, &_state, sizeof(State)) != sizeof(State))
		{
			/* ERROR HANDLING: i2c transaction failed */
			std::cerr << "Failed to write to the i2c bus (" << i2c_bus << ")" << std::endl;
		}
#endif
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Portexpander_I2C_Bridge>());
  rclcpp::shutdown();
  return 0;
}