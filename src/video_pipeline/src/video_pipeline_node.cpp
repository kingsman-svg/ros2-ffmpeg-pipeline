#include <memory>

#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("video_pipeline");
  RCLCPP_INFO(node->get_logger(), "Video pipeline node started");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
