#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#include <termios.h>
#include <unistd.h>

namespace line_statistics {

struct RunningStats {
  void add(double value) {
    ++count;
    const double delta = value - mean;
    mean += delta / static_cast<double>(count);
    const double delta2 = value - mean;
    m2 += delta * delta2;
    sum_sq += value * value;
    sum_abs += std::abs(value);
    if (value < min) {
      min = value;
    }
    if (value > max) {
      max = value;
    }
    const double abs_val = std::abs(value);
    if (abs_val > max_abs) {
      max_abs = abs_val;
    }
  }

  double variance() const {
    return (count > 1) ? m2 / static_cast<double>(count - 1) : 0.0;
  }

  double stddev() const {
    return std::sqrt(variance());
  }

  double rmse() const {
    return (count > 0) ? std::sqrt(sum_sq / static_cast<double>(count)) : 0.0;
  }

  double mean_abs_value() const {
    return (count > 0) ? sum_abs / static_cast<double>(count) : 0.0;
  }

  void reset() {
    count = 0;
    mean = 0.0;
    m2 = 0.0;
    sum_sq = 0.0;
    sum_abs = 0.0;
    min = std::numeric_limits<double>::infinity();
    max = -std::numeric_limits<double>::infinity();
    max_abs = 0.0;
  }

  std::size_t count{0};
  double mean{0.0};
  double m2{0.0};
  double sum_sq{0.0};
  double sum_abs{0.0};
  double min{std::numeric_limits<double>::infinity()};
  double max{-std::numeric_limits<double>::infinity()};
  double max_abs{0.0};
};

struct LapMetrics {
  std::size_t lap_index{0};
  ros::Duration duration{0.0};
  RunningStats error_stats;
  std::size_t velocity_samples{0};
  double linear_sum{0.0};
  double linear_abs_sum{0.0};
  double angular_sum{0.0};
  double angular_abs_sum{0.0};
  double linear_max{0.0};
  double angular_max_abs{0.0};
  ros::Time stamp;
};

class StatisticsNode {
public:
  StatisticsNode()
  : nh_()
  , pnh_("~") {
    pnh_.param<std::string>("error_topic", error_topic_, "/error");
    pnh_.param<std::string>("cmd_topic", cmd_topic_, "/cmd_vel");
    pnh_.param<std::string>("log_file", log_file_path_, std::string());
    pnh_.param("enable_keyboard", enable_keyboard_, true);
    pnh_.param("status_period", status_period_, 5.0);

    error_sub_ = nh_.subscribe(error_topic_, 200, &StatisticsNode::errorCb, this);

    if (!cmd_topic_.empty()) {
      cmd_sub_ = nh_.subscribe(cmd_topic_, 200, &StatisticsNode::cmdCb, this);
    }

    summary_pub_ = nh_.advertise<std_msgs::String>("lap_statistics", 10, true);

    if (status_period_ > 0.0) {
      status_timer_ = nh_.createTimer(ros::Duration(status_period_), &StatisticsNode::statusTimerCb, this);
    }

    if (!log_file_path_.empty()) {
      openLogFile();
    }

    if (enable_keyboard_) {
      keep_running_.store(true);
      keyboard_thread_ = std::thread(&StatisticsNode::keyboardLoop, this);
    }
  }

  ~StatisticsNode() {
    keep_running_.store(false);
    if (keyboard_thread_.joinable()) {
      keyboard_thread_.join();
    }
    if (log_file_.is_open()) {
      log_file_.close();
    }
  }

  void spin() {
    ros::spin();
  }

private:
  static std::string expandUserPath(const std::string& path) {
    if (path.empty() || path.front() != '~') {
      return path;
    }
    const char* home = std::getenv("HOME");
    if (!home) {
      return path;
    }
    return std::string(home) + path.substr(1);
  }

  void openLogFile() {
    const std::string expanded = expandUserPath(log_file_path_);
    std::ifstream existing(expanded);
    const bool has_content = existing.good() && existing.peek() != std::ifstream::traits_type::eof();
    existing.close();

    log_file_.open(expanded, std::ios::app);
    if (!log_file_.is_open()) {
      ROS_WARN_STREAM("statistics_node: failed to open log file at " << expanded);
      return;
    }

    log_file_path_ = expanded;
    if (!has_content) {
      log_file_ << "lap_index,stamp,laptime_sec,samples,mean_error,rmse,stddev,"
                   "mean_abs_error,max_abs_error,"
                   "avg_linear,avg_abs_linear,avg_angular,avg_abs_angular,"
                   "max_linear,max_abs_angular,trigger\n";
      log_file_.flush();
    }
  }

  void errorCb(const std_msgs::Float32::ConstPtr& msg) {
    const ros::Time stamp = ros::Time::now();
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (!lap_active_) {
      lap_active_ = true;
      lap_start_time_ = stamp;
    }

    error_stats_.add(static_cast<double>(msg->data));
  }

  void cmdCb(const geometry_msgs::Twist::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    ++velocity_samples_;
    linear_sum_ += static_cast<double>(msg->linear.x);
    linear_abs_sum_ += std::abs(static_cast<double>(msg->linear.x));
    angular_sum_ += static_cast<double>(msg->angular.z);
    angular_abs_sum_ += std::abs(static_cast<double>(msg->angular.z));
    if (msg->linear.x > linear_max_) {
      linear_max_ = msg->linear.x;
    }
    const double abs_ang = std::abs(static_cast<double>(msg->angular.z));
    if (abs_ang > angular_max_abs_) {
      angular_max_abs_ = abs_ang;
    }
  }

  void statusTimerCb(const ros::TimerEvent&) {
    LapMetrics snapshot;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!lap_active_ || error_stats_.count == 0) {
        return;
      }
      snapshot = collectMetricsLocked(ros::Time::now());
    }

    ROS_INFO_STREAM("[lap " << snapshot.lap_index << " running] "
                    << formatSummary(snapshot));
  }

  LapMetrics collectMetricsLocked(const ros::Time& now) const {
    LapMetrics metrics;
    metrics.lap_index = lap_index_;
    metrics.duration = lap_active_ ? (now - lap_start_time_) : ros::Duration(0.0);
    metrics.error_stats = error_stats_;
    metrics.velocity_samples = velocity_samples_;
    metrics.linear_sum = linear_sum_;
    metrics.linear_abs_sum = linear_abs_sum_;
    metrics.angular_sum = angular_sum_;
    metrics.angular_abs_sum = angular_abs_sum_;
    metrics.linear_max = linear_max_;
    metrics.angular_max_abs = angular_max_abs_;
    metrics.stamp = now;
    return metrics;
  }

  std::string formatSummary(const LapMetrics& metrics) const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "t=" << metrics.duration.toSec() << "s";
    if (metrics.error_stats.count > 0) {
      oss << " samples=" << metrics.error_stats.count
          << " mean=" << metrics.error_stats.mean
          << " rmse=" << metrics.error_stats.rmse()
          << " std=" << metrics.error_stats.stddev()
          << " mean|e|=" << metrics.error_stats.mean_abs_value()
          << " max|e|=" << metrics.error_stats.max_abs;
    } else {
      oss << " samples=0";
    }

    if (metrics.velocity_samples > 0) {
      const double s = static_cast<double>(metrics.velocity_samples);
      const double avg_linear = metrics.linear_sum / s;
      const double mean_abs_linear = metrics.linear_abs_sum / s;
      const double avg_angular = metrics.angular_sum / s;
      const double mean_abs_angular = metrics.angular_abs_sum / s;
      oss << " mean_v=" << avg_linear
          << " mean|v|=" << mean_abs_linear
          << " mean_w=" << avg_angular
          << " mean|w|=" << mean_abs_angular
          << " max_v=" << metrics.linear_max
          << " max|w|=" << metrics.angular_max_abs;
    }

    return oss.str();
  }

  void publishSummary(const LapMetrics& metrics, const std::string& trigger) {
    if (!summary_pub_) {
      return;
    }
    std_msgs::String msg;
    std::ostringstream oss;
    oss << "lap=" << metrics.lap_index
        << " trigger=" << trigger << " "
        << formatSummary(metrics);
    msg.data = oss.str();
    summary_pub_.publish(msg);
  }

  void writeLapToFile(const LapMetrics& metrics, const std::string& trigger) {
    if (!log_file_.is_open()) {
      return;
    }

    const double samples = static_cast<double>(metrics.velocity_samples);
    const double avg_linear = (samples > 0.0) ? metrics.linear_sum / samples : 0.0;
    const double mean_abs_linear = (samples > 0.0) ? metrics.linear_abs_sum / samples : 0.0;
    const double avg_angular = (samples > 0.0) ? metrics.angular_sum / samples : 0.0;
    const double mean_abs_angular = (samples > 0.0) ? metrics.angular_abs_sum / samples : 0.0;

    log_file_ << metrics.lap_index << ','
              << std::fixed << std::setprecision(6) << metrics.stamp.toSec() << ','
              << metrics.duration.toSec() << ','
              << metrics.error_stats.count << ','
              << metrics.error_stats.mean << ','
              << metrics.error_stats.rmse() << ','
              << metrics.error_stats.stddev() << ','
              << metrics.error_stats.mean_abs_value() << ','
              << metrics.error_stats.max_abs << ','
              << avg_linear << ','
              << mean_abs_linear << ','
              << avg_angular << ','
              << mean_abs_angular << ','
              << metrics.linear_max << ','
              << metrics.angular_max_abs << ','
              << trigger << '\n';
    log_file_.flush();
  }

  void resetLapLocked(const ros::Time& reset_stamp) {
    error_stats_.reset();
    velocity_samples_ = 0;
    linear_sum_ = 0.0;
    linear_abs_sum_ = 0.0;
    angular_sum_ = 0.0;
    angular_abs_sum_ = 0.0;
    linear_max_ = 0.0;
    angular_max_abs_ = 0.0;
    lap_active_ = false;
    lap_start_time_ = reset_stamp;
  }

  void markLapEnd(const std::string& trigger) {
    LapMetrics snapshot;
    std::size_t lap_index_finished = 0;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!lap_active_ || error_stats_.count == 0) {
        ROS_WARN_STREAM("statistics_node: lap end triggered by " << trigger
                         << " but no samples collected yet.");
        return;
      }

      const ros::Time now = ros::Time::now();
      snapshot = collectMetricsLocked(now);
      lap_index_finished = lap_index_;
      ++lap_index_;
      resetLapLocked(now);
    }

    snapshot.lap_index = lap_index_finished;

    const std::string summary = formatSummary(snapshot);
    ROS_INFO_STREAM("[lap " << snapshot.lap_index << " complete] "
                    << summary << " trigger=" << trigger);

    publishSummary(snapshot, trigger);
    writeLapToFile(snapshot, trigger);
  }

  char readKeyNonBlocking() const {
    struct termios oldt;
    if (tcgetattr(STDIN_FILENO, &oldt) != 0) {
      return 0;
    }
    struct termios newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_cc[VMIN] = 0;
    newt.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &newt) != 0) {
      return 0;
    }

    int ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    if (ch == EOF) {
      clearerr(stdin);
      return 0;
    }
    return static_cast<char>(ch);
  }

  void keyboardLoop() {
    ros::Rate rate(25.0);
    while (ros::ok() && keep_running_.load()) {
      const char key = readKeyNonBlocking();
      if (key == ' ') {
        markLapEnd("spacebar");
      } else if (key == 'r' || key == 'R') {
        std::lock_guard<std::mutex> lock(data_mutex_);
        resetLapLocked(ros::Time::now());
        lap_index_ = 1;
        ROS_INFO("statistics_node: lap data reset.");
      }
      rate.sleep();
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber error_sub_;
  ros::Subscriber cmd_sub_;
  ros::Publisher summary_pub_;
  ros::Timer status_timer_;

  std::string error_topic_;
  std::string cmd_topic_;
  std::string log_file_path_;

  bool enable_keyboard_{true};
  double status_period_{5.0};

  std::mutex data_mutex_;
  RunningStats error_stats_;
  std::size_t velocity_samples_{0};
  double linear_sum_{0.0};
  double linear_abs_sum_{0.0};
  double angular_sum_{0.0};
  double angular_abs_sum_{0.0};
  double linear_max_{0.0};
  double angular_max_abs_{0.0};

  bool lap_active_{false};
  std::size_t lap_index_{1};
  ros::Time lap_start_time_;

  std::atomic<bool> keep_running_{false};
  std::thread keyboard_thread_;

  std::ofstream log_file_;
};

}  // namespace line_statistics

int main(int argc, char** argv) {
  ros::init(argc, argv, "statistics");
  line_statistics::StatisticsNode node;
  node.spin();
  return 0;
}
