#include <QThread>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

class CommNode : public rclcpp::Node, public QThread {

public:
  explicit CommNode(int id);
  void run() override;

  void set_tf_msg(const tf2_msgs::msg::TFMessage &msg);
  void PublishStaticTF();
  void PublishTF();

private:
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_msgs::msg::TFMessage tf_msg_;
};
