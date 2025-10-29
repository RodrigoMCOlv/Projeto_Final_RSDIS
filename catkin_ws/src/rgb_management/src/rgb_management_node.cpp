#include <cctype>
#include <cmath>
#include <limits>
#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>

class RgbManagementNode {
public:
  RgbManagementNode()
  : nh_(), pnh_("~")
  {
    pnh_.param<std::string>("target_param", target_param_, std::string("/preview_controller/base_speed"));
    pnh_.param("default_speed", default_speed_, 0.2);
    pnh_.param("red_speed", red_speed_, default_speed_);
    pnh_.param("green_speed", green_speed_, default_speed_);
    pnh_.param("blue_speed", blue_speed_, default_speed_);

    desired_speed = default_speed_;
    colour_sub_ = nh_.subscribe("/colour", 5, &RgbManagementNode::colourCallback, this);

    setSpeed(default_speed_, "initialisation");
  }

private:

  double desired_speed{0.0};
  std::string tag = "default";

  void colourCallback(const std_msgs::String::ConstPtr& msg) {
    if (!msg) {
      return;
    }

    if (!msg->data.empty()) {
      const char key = static_cast<char>(std::toupper(msg->data[0]));
      if (key == 'R') {
        desired_speed = red_speed_;
        tag = "red";
      } else if (key == 'G') {
        desired_speed = green_speed_;
        tag = "green";
      } else if (key == 'B') {
        desired_speed = blue_speed_;
        tag = "blue";
      }
    }

    setSpeed(desired_speed, tag);
  }

  void setSpeed(double value, const std::string& reason) {
    if (std::abs(value - last_speed_) < 1e-6) {
      return;
    }
    ros::param::set(target_param_, value);
    last_speed_ = value;
    ROS_INFO_STREAM("rgb_management: set " << target_param_ << " to " << value << " (" << reason << ")");
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber colour_sub_;

  std::string target_param_;
  double default_speed_{0.3};
  double red_speed_{0.2};
  double green_speed_{0.2};
  double blue_speed_{0.2};
  double last_speed_{std::numeric_limits<double>::quiet_NaN()};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "rgb_management");
  RgbManagementNode node;
  ros::spin();
  return 0;
}
