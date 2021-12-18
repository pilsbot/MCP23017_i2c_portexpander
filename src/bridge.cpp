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
		uint8_t port[2];	//A and B
	} _state;

	struct Message
	{
	    uint8_t addr;
	    uint8_t data;
	};
#ifndef MOCKUP
	int i2c_bus;
#endif
	
	static constexpr uint8_t IODIRA = 0x00;
	static constexpr uint8_t IODIRB = 0x01;
	static constexpr uint8_t GPIOPA = 0x14;
	static constexpr uint8_t GPIOPB = 0x15;
	
  public:
    Portexpander_I2C_Bridge()
    : Node("portexpander_i2c_bridge")
    {
		this->declare_parameter("i2c_device", "/dev/i2c-2");
		this->declare_parameter("mcp_addr", 0x20);
		std::string i2c_device = this->get_parameter("i2c_device").as_string();
		int addr = this->get_parameter("mcp_addr").as_int();
		RCLCPP_INFO(this->get_logger(), "using %s at addr 0x%x",
		    i2c_device.c_str(), addr);
		
		memset(&_state, 0, sizeof(State));
#ifndef MOCKUP
		//----- OPEN THE I2C BUS -----
		if ((i2c_bus = open(i2c_device.c_str(), O_RDWR)) < 0)
		{
		    RCLCPP_FATAL(this->get_logger(),
		        "Failed to open the i2c bus at %s", i2c_device.c_str());
			perror("I2C open");
			return;
		}

		if (ioctl(i2c_bus, I2C_SLAVE, addr) < 0)
		{
            RCLCPP_FATAL(this->get_logger(),
                "Failed to acquire bus access and/or talk to slave.");
			perror("ioctl");
			return;
		}
#endif
		
		
		// Set PA+PB to output (0)
		//todo: maybe read default values?
		Message output_dir{IODIRA, 0};
        if (!write_to_device(output_dir))
        {
            RCLCPP_WARN(this->get_logger(),
            "Failed to configure the i2c device (%d)", i2c_bus);
        }
        output_dir = {IODIRB, 0};
        if (!write_to_device(output_dir))
        {
            RCLCPP_WARN(this->get_logger(),
            "Failed to configure the i2c device (%d)", i2c_bus);
        }


		_subscriptions.reserve(2*8);
		const std::string base_name = "portexpander_i2c_bridge/";
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

		auto state = _state.port[port];

		if(msg->data)
			state |= 1 << bitpos;
		else
			state &= ~(1 << bitpos);

		Message pinchange{port == 0 ? GPIOPA : GPIOPB, state};
		if (!write_to_device(pinchange))
		{
			RCLCPP_ERROR(this->get_logger(),
			    "could not update pin!");
		}
    }

    bool write_to_device(Message& msg) {
#ifndef MOCKUP
        if (write(i2c_bus, &msg, sizeof(Message)) != sizeof(Message))
        {
            /* TODO: error handling */
            RCLCPP_WARN(this->get_logger(),
            "Failed to write to the i2c device (%d)", i2c_bus);
            perror("write");
            return false;
        }
#endif
        return true;
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Portexpander_I2C_Bridge>());
  rclcpp::shutdown();
  return 0;
}
