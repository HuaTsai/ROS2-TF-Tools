#include "comm_node.h"

CommNode::CommNode() : Node("comm_node") {
  tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>("tf", 10);
  str_pub_ = create_publisher<std_msgs::msg::String>("str", 10);
  timer_ = create_wall_timer(std::chrono::milliseconds(1000),
                             std::bind(&CommNode::cb, this));
}

void CommNode::cb() {
  std_msgs::msg::String msg;
  static int cnt = 0;
  msg.data = "Hello, world! #" + std::to_string(++cnt);
  str_pub_->publish(msg);
  std::cout << "Published: " << msg.data << std::endl;
}

void CommNode::run() {
  rclcpp::spin(this->get_node_base_interface());
}
