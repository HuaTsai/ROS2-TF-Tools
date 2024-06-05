#include "comm_node.h"

CommNode::CommNode(int id) : Node("comm_node_" + std::to_string(id)) {
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  qos.transient_local();
  tf_static_pub_ = create_publisher<tf2_msgs::msg::TFMessage>("tf_static", qos);
  tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>("tf", 10);
}

void CommNode::set_tf_msg(const tf2_msgs::msg::TFMessage &msg) {
  tf_msg_ = msg;
}

void CommNode::PublishStaticTF() {
  timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
    tf_msg_.transforms[0].header.stamp = now();
    tf_static_pub_->publish(tf_msg_);
  });
}

void CommNode::PublishTF() {
  timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
    tf_msg_.transforms[0].header.stamp = now();
    tf_pub_->publish(tf_msg_);
  });
}

void CommNode::run() { rclcpp::spin(this->get_node_base_interface()); }
