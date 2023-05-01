#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional> 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using namespace std::chrono_literals;
using namespace std_msgs;
using namespace rclcpp;
using namespace std;

//Erbt von rclcpp::Node 
class HelloWorldPubNode : public Node {
	public:
		HelloWorldPubNode() : Node("hello_world_pub_node") 
		{
			publisher_ = this->create_publisher<msg::String>("hello_world", 10);
			timer_ = this->create_wall_timer(1s, bind(&HelloWorldPubNode::publish_hello_world, this));
		}

	private:
		void publish_hello_world()
    {
      auto message = msg::String();
      message.data = "Hello, World  " + to_string(counter_);
      publisher_->publish(message);
      counter_++;
    }
	Publisher<msg::String>::SharedPtr publisher_;
    TimerBase::SharedPtr timer_;
    size_t counter_ = 0;

};


int main(int argc, char * argv[]) {
	init(argc, argv);
  spin(make_shared<HelloWorldPubNode>());
  shutdown();

	return 0;
}