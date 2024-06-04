#include "comm_node.h"

CommNode::CommNode() : Node("comm_node") {
  tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>("tf", 10);
}

void CommNode::set_tf_msg(const tf2_msgs::msg::TFMessage &msg) {
  tf_msg_ = msg;
}

void CommNode::StartPublish() {
  timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
    tf_msg_.transforms[0].header.stamp = now();
    tf_pub_->publish(tf_msg_);
  });
}

void CommNode::StopPublish() { timer_->cancel(); }

void CommNode::run() { rclcpp::spin(this->get_node_base_interface()); }
