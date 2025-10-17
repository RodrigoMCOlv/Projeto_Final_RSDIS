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


 /* 
  double kp_lat{0.0};
  double ki_lat{0.0};
  double kd_lat{0.0};



  double kp_head{0.6};
  double ki_head{0.0};
  double kd_head{0.05};
*/
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

    /*
    pnh_.param("kp_lat", gains_.kp_lat, gains_.kp_lat);
    pnh_.param("ki_lat", gains_.ki_lat, gains_.ki_lat);
    pnh_.param("kd_lat", gains_.kd_lat, gains_.kd_lat);

    pnh_.param("kp_head", gains_.kp_head, gains_.kp_head);
    pnh_.param("ki_head", gains_.ki_head, gains_.ki_head);
    pnh_.param("kd_head", gains_.kd_head, gains_.kd_head);
    */
    pnh_.param("kp", gains_.kp, gains_.kp);
    pnh_.param("ki", gains_.ki, gains_.ki);
    pnh_.param("kd", gains_.kd, gains_.kd);

    pnh_.param("error_timeout", gains_.error_timeout, gains_.error_timeout);

    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 25);
    /*sub_lat_ = nh_.subscribe<std_msgs::Float32>("/line_lateral_error", 10,
                    &PreviewControllerNode::latCb, this);
    sub_head_ = nh_.subscribe<std_msgs::Float32>("/line_heading_error", 10,
                    &PreviewControllerNode::headCb, this);
    */
    error_ = nh_.subscribe<std_msgs::Float32>("/error", 10,
              &PreviewControllerNode::errorCb, this);
    timer_ = nh_.createTimer(ros::Duration(0.02), &PreviewControllerNode::controlLoop, this);
  }

private:
  static double clip(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
  }

    void errorCb(const std_msgs::Float32::ConstPtr& msg) {
    error_cb = msg->data;
    time_now = ros::Time::now();
    have_cb = true;
    }
  /*
  void latCb(const std_msgs::Float32::ConstPtr& msg) {
    e_y_ = msg->data;
    t_e_y_ = ros::Time::now();
    have_e_y_ = true;
  }
  void headCb(const std_msgs::Float32::ConstPtr& msg) {
    e_psi_ = msg->data;
    t_e_psi_ = ros::Time::now();
    have_e_psi_ = true;
  }
    */


  void controlLoop(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
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



    /*const bool fresh_lat = have_e_y_ &&
                           (now - t_e_y_).toSec() <= gains_.error_timeout;
    const bool fresh_head = have_e_psi_ &&
                            (now - t_e_psi_).toSec() <= gains_.error_timeout;

    double dt = last_time_.isZero() ? 0.02 : (now - last_time_).toSec();
    if (dt <= 0.0) dt = 0.02;

    double lat_output = 0.0;
    if (fresh_lat) {
      double lat_error = static_cast<double>(e_y_);
      lat_integral_ += lat_error * dt;
      double lat_derivative = 0.0;
      if (have_prev_lat_) {
        lat_derivative = (lat_error - prev_lat_error_) / dt;
      }
      lat_output = gains_.kp_lat * lat_error +
                   gains_.ki_lat * lat_integral_ +
                   gains_.kd_lat * lat_derivative;
      prev_lat_error_ = lat_error;
      have_prev_lat_ = true;
    } else {
      lat_integral_ = 0.0;
      prev_lat_error_ = 0.0;
      have_prev_lat_ = false;
    }

    double head_output = 0.0;
    if (fresh_head) {
      double head_error = static_cast<double>(e_psi_);
      head_integral_ += head_error * dt;
      double head_derivative = 0.0;
      if (have_prev_head_) {
        head_derivative = (head_error - prev_head_error_) / dt;
      }
      head_output = gains_.kp_head * head_error +
                    gains_.ki_head * head_integral_ +
                    gains_.kd_head * head_derivative;
      prev_head_error_ = head_error;
      have_prev_head_ = true;
    } else {
      head_integral_ = 0.0;
      prev_head_error_ = 0.0;
      have_prev_head_ = false;
    }*/

    geometry_msgs::Twist cmd;
    /*if (!fresh_lat && !fresh_head) {
      cmd.linear.x = gains_.speed_min;
      cmd.angular.z = 0.0;
    } else {
      double base_speed = clip(gains_.base_speed, gains_.speed_min, gains_.speed_max);
      double angular_cmd = lat_output + head_output;


      angular_cmd = clip(angular_cmd, -gains_.angular_max, gains_.angular_max);

      cmd.linear.x = base_speed;
      cmd.angular.z = angular_cmd;
    }*/
    if (!fresh_error) {
      cmd.linear.x = gains_.speed_min;
      cmd.angular.z = 0.0;
    } else {
      double base_speed = clip(gains_.base_speed, gains_.speed_min, gains_.speed_max);
      double angular_cmd = error_output;


      angular_cmd = clip(angular_cmd, -gains_.angular_max, gains_.angular_max);

      cmd.linear.x = base_speed;
      cmd.angular.z = angular_cmd;
    }

    last_time_ = now;
    cmd_pub_.publish(cmd);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher  cmd_pub_;
  //ros::Subscriber sub_lat_;
  //ros::Subscriber sub_head_;
  ros::Subscriber error_;
  
  ros::Timer      timer_;

  Gains gains_;

  

  //float e_y_{0.0f};
  //float e_psi_{0.0f};
  float error_cb{0.0f};
  //ros::Time t_e_y_;
  //ros::Time t_e_psi_;
  ros::Time time_now;
  //bool have_e_y_{false};
  //bool have_e_psi_{false};
  bool have_cb{false};


  //double lat_integral_{0.0};
  //double head_integral_{0.0};
  double error_integral_{0.0};
  //double prev_lat_error_{0.0};
  //double prev_head_error_{0.0};
  double prev_error_{0.0};
  //bool have_prev_lat_{false};
  //bool have_prev_head_{false};
  bool have_prev_error_{false};
  
  
  ros::Time last_time_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "preview_controller");
  PreviewControllerNode node;
  ros::spin();
  return 0;
}
