#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional> 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include <math.h>


#include <iostream>
using namespace std::chrono_literals;
using namespace std_msgs;
using namespace rclcpp;
using namespace std;

const float WHEEL_RADIUS = 12.5 / 100; /// centimeters to meters

class CalcSpeedNode : public rclcpp::Node
{
  public: 
    CalcSpeedNode() : Node("calc_speed_node") 
		{
      this->declare_parameter<float>("wheel_radius", WHEEL_RADIUS);
			rpm_subscription_ = this->create_subscription<msg::Float32>(
				"rpm", 10, bind(&CalcSpeedNode::calc_pub_speed, this, placeholders::_1)
			);
      speed_publisher_ = this->create_publisher<msg::Float32>("speed", 10);
      cout << "Subscribing to RPM Node and calculate speed" << endl;
		}

	private:
		void calc_pub_speed(const msg::Float32 & msg) const
		{
            auto message = msg::Float32();
            /// Speed [m/s] = RPM [rev/min] * Wheel_Circumference [meters/rev] / 60 [seconds/min]
            rclcpp::Parameter wheel_radius_param_obj = this->get_parameter("wheel_radius");
            message.data = msg.data * (2 * wheel_radius_param_obj.as_double() * M_PI) / 60;
            speed_publisher_->publish(message);

            
		}
		Subscription<msg::Float32>::SharedPtr rpm_subscription_;
        Publisher<msg::Float32>::SharedPtr speed_publisher_;
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CalcSpeedNode>());
  rclcpp::shutdown();
  
  return 0;
}