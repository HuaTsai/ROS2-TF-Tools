#include <QThread>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

class CommNode : public rclcpp::Node, public QThread {

public:
  CommNode();
  void cb();
  void run() override;

private:
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr str_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
