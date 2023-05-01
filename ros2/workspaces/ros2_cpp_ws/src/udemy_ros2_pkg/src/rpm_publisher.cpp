#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional> 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;
using namespace std_msgs;
using namespace rclcpp;
using namespace std;

const float RPM_DEFAULT_VALUE = 10.0;

//Erbt von rclcpp::Node 
class RPMPubNode : public Node {
	public:
		RPMPubNode() : Node("rpm_pub_node") 
		{   
            //this->declare_parameter<float>("rpm_val", rpm_val_param);
            this->declare_parameter<float>("rpm_val", RPM_DEFAULT_VALUE);
			publisher_ = this->create_publisher<msg::Float32>("rpm", 10);
			timer_ = this->create_wall_timer(1s, bind(&RPMPubNode::publish_rpm_value, this));
            cout << "RPM Node is running and publishing.."<< endl;
		}

	private:
		void publish_rpm_value()
        {
        auto message = msg::Float32();
        //this->get_parameter("rpm_val", rpm_val_param);
        rclcpp::Parameter rpm_val_param_obj = this->get_parameter("rpm_val");
        //message.data  = rpm_val_param;
        message.data = rpm_val_param_obj.as_double();
        publisher_->publish(message);
        }
	Publisher<msg::Float32>::SharedPtr publisher_;
    TimerBase::SharedPtr timer_;
    //float rpm_val_param = RPM_DEFAULT_VALUE;
    //this->set_parameter(rclcpp::Parameter("your_param_name", 5.0));

};


int main(int argc, char * argv[]) {
  init(argc, argv);
  spin(make_shared<RPMPubNode>());
  shutdown();

	return 0;
}