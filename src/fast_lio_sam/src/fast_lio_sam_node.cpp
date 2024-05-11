#include "fast_lio_sam/main.h"


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr node_ptr = rclcpp::Node::make_shared("fast_lio_sam");

  FAST_LIO_SAM_CLASS fast_lio_sam_(node_ptr);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node_ptr);
  executor.spin();

  return 0;
}