#include "order_publisher.h"

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OrderPublisher>());
	rclcpp::shutdown();
	return 0;
}