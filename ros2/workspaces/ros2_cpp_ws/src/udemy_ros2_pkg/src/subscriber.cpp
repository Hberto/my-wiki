#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional> 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <iostream>
using namespace std::chrono_literals;
using namespace std_msgs;
using namespace rclcpp;
using namespace std;


class HelloWorldSubNode : public rclcpp::Node
{
  public: 
    HelloWorldSubNode() : Node("hello_world_sub_node") 
		{
			subscription_ = this->create_subscription<msg::String>(
				"hello_world", 10, bind(&HelloWorldSubNode::sub_callback, this, placeholders::_1)
			);
		}

	private:
		void sub_callback(const msg::String & msg) const
		{
			std::cout << msg.data << std::endl;
			// ROS LOGS
			//RCLCPP_INFO(this-> get_logger(), msg.data.c_str());
			//RCLCPP_WARN(this-> get_logger(), msg.data.c_str());
			//RCLCPP_DEBUG(this-> get_logger(), msg.data.c_str());
			//RCLCPP_ERROR(this-> get_logger(), msg.data.c_str());
		}
		Subscription<msg::String>::SharedPtr subscription_;
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HelloWorldSubNode>());
  rclcpp::shutdown();
  
  return 0;
}