#include <algorithm>
#include <cmath>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

struct Gains {
  double base_speed{0.14};
  double speed_min{0.05};
  double speed_max{0.2};


  double angular_max{0.2};

  double kp{0.3};
  double ki{0.0};
  double kd{0.0};

  double boost{0.2};

  double error_timeout{0.3};
};

class PreviewControllerNode {
public:
  PreviewControllerNode()
  : nh_(), pnh_("~")
  {
    pnh_.param("base_speed", gains_.base_speed, gains_.base_speed);
    pnh_.param("speed_min", gains_.speed_min, gains_.speed_min);
    pnh_.param("speed_max", gains_.speed_max, gains_.speed_max);

    pnh_.param("angular_max", gains_.angular_max, gains_.angular_max);

    pnh_.param("boost", gains_.boost, gains_.boost);

    pnh_.param("kp", gains_.kp, gains_.kp);
    pnh_.param("ki", gains_.ki, gains_.ki);
    pnh_.param("kd", gains_.kd, gains_.kd);

    pnh_.param("error_timeout", gains_.error_timeout, gains_.error_timeout);

    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 25);

    error_ = nh_.subscribe<std_msgs::Float32>("/error", 10,
              &PreviewControllerNode::errorCb, this);
    timer_ = nh_.createTimer(ros::Duration(0.02), &PreviewControllerNode::controlLoop, this);
  }

private:
  static double clip(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
  }

  void refreshParameters() {
    double updated_base_speed = gains_.base_speed;
    pnh_.param("base_speed", updated_base_speed, gains_.base_speed);
    gains_.base_speed = updated_base_speed;
  }

    void errorCb(const std_msgs::Float32::ConstPtr& msg) {
    error_cb = msg->data;
    time_now = ros::Time::now();
    have_cb = true;
    }


  void controlLoop(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    refreshParameters();
    const bool fresh_error = have_cb &&
                             (now - time_now).toSec() <= gains_.error_timeout;

    double error_output = 0.0;

    
    double dt = last_time_.isZero() ? 0.02 : (now - last_time_).toSec();
    if (dt <= 0.0) dt = 0.02;

    
    if (fresh_error) {
      double error = static_cast<double>(error_cb);
      error_integral_ += error * dt;
      double error_derivative = 0.0;
      if (have_prev_error_) {
        error_derivative = (error - prev_error_) / dt;
      }
      error_output = gains_.kp * error +
                    gains_.ki * error_integral_ +
                    gains_.kd * error_derivative;
      prev_error_ = error;
      have_prev_error_ = true;
    } else {
      error_integral_ = 0.0;
      prev_error_ = 0.0;
      have_prev_error_ = false;
      have_cb = false;
    }


    geometry_msgs::Twist cmd;

    if (!fresh_error) {
      cmd.linear.x = gains_.speed_min;
      cmd.angular.z = 0.0;
    } else {

      if (error_output < 0.03 && error_output > -0.03)
      {
        double speed = clip(gains_.base_speed + gains_.boost, gains_.speed_min, gains_.speed_max);
        cmd.linear.x = speed;
      }
      else{
      double speed = clip(gains_.base_speed, gains_.speed_min, gains_.speed_max);
      cmd.linear.x = speed;
      }

      double angular_cmd = error_output;
      angular_cmd = clip(angular_cmd, -gains_.angular_max, gains_.angular_max);

      
      cmd.angular.z = angular_cmd;
    }

    last_time_ = now;
    cmd_pub_.publish(cmd);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher  cmd_pub_;
  ros::Subscriber error_;
  
  ros::Timer      timer_;

  Gains gains_;

  

  float error_cb{0.0f};
  ros::Time time_now;
  bool have_cb{false};



  double error_integral_{0.0};
  double prev_error_{0.0};
  bool have_prev_error_{false};
  
  
  ros::Time last_time_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "preview_controller");
  PreviewControllerNode node;
  ros::spin();
  return 0;
}
