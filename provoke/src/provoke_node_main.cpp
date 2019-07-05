
#include "provoke/provoke_node.hpp"

int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

  // Create single-threaded executor
  rclcpp::executors::SingleThreadedExecutor executor;

  // Create and add node
  rclcpp::NodeOptions options{};
  options.use_intra_process_comms(false); // TODO
  auto node = provoke::node_factory(options);

  executor.add_node(node);

  // Spin until rclcpp::ok() returns false
  executor.spin();

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
