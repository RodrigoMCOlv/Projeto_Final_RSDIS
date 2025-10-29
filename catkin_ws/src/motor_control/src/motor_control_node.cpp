#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <cmath>
#include <softPwm.h>
#include <wiringPi.h>


namespace {

// Motor driver pins (WiringPi numbering)
constexpr int kLeftForwardPin = 28;
constexpr int kLeftBackwardPin = 29;
constexpr int kRightForwardPin = 22;
constexpr int kRightBackwardPin = 23;
constexpr int kLeftPwmPin = 25;
constexpr int kRightPwmPin = 26;
constexpr float kDeadzone = 0.01;

constexpr int kMaxPwmValue = 100;

bool setupPins() {
  if (wiringPiSetup() == -1) {
    ROS_ERROR("Failed to initialise wiringPi.");
    return false;
  }

  pinMode(kLeftForwardPin, OUTPUT);
  pinMode(kLeftBackwardPin, OUTPUT);
  pinMode(kRightForwardPin, OUTPUT);
  pinMode(kRightBackwardPin, OUTPUT);

  if (softPwmCreate(kLeftPwmPin, 0, kMaxPwmValue) != 0) {
    ROS_ERROR("Failed to create software PWM on left motor pin.");
    return false;
  }

  if (softPwmCreate(kRightPwmPin, 0, kMaxPwmValue) != 0) {
    ROS_ERROR("Failed to create software PWM on right motor pin.");
    return false;
  }

  return true;
}

inline int speedToPwm(float s) {
  // Clamp to [-1, 1] just in case
  if(s > 1) s = 1;
  if(s < -1) s = -1;

  if (std::fabs(s) < kDeadzone) return 0;
  return static_cast<int>(std::round(kMaxPwmValue * std::fabs(s)));
}

void leftDrive(float speed)
{

  const int pwm = speedToPwm(speed);
  if (speed > 0){
    digitalWrite(kLeftForwardPin, LOW);
    digitalWrite(kLeftBackwardPin, HIGH);
    softPwmWrite(kLeftPwmPin,pwm);
  }else if(speed < 0){
    digitalWrite(kLeftForwardPin, HIGH);
    digitalWrite(kLeftBackwardPin, LOW);
    softPwmWrite(kLeftPwmPin,pwm);
  }else{
    digitalWrite(kLeftForwardPin, LOW);
    digitalWrite(kLeftBackwardPin, LOW);
    softPwmWrite(kLeftPwmPin, 0);

    digitalWrite(kRightForwardPin, LOW);
    digitalWrite(kRightBackwardPin, LOW);
    softPwmWrite(kRightPwmPin, 0);
  }

}
void rightDrive(float speed)
{

  const int pwm = speedToPwm(speed);
  if (speed > 0){
    digitalWrite(kRightForwardPin, LOW);
    digitalWrite(kRightBackwardPin, HIGH);
    softPwmWrite(kRightPwmPin,pwm);
  }else if(speed < 0){
    digitalWrite(kRightForwardPin, HIGH);
    digitalWrite(kRightBackwardPin, LOW);
    softPwmWrite(kRightPwmPin,pwm);
  }else{
    digitalWrite(kRightForwardPin, LOW);
    digitalWrite(kRightBackwardPin, LOW);
    softPwmWrite(kRightPwmPin, 0);

    digitalWrite(kLeftForwardPin, LOW);
    digitalWrite(kLeftBackwardPin, LOW);
    softPwmWrite(kLeftPwmPin, 0);
  }

}

}  // namespace

class MotorControl {
 public:
  MotorControl() {
    if (!setupPins()) {
      ros::shutdown();
      return;
    }
    
    leftDrive(0);
    rightDrive(0);
    cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 10, &MotorControl::twistCallback, this);
  }

  ~MotorControl() {
    softPwmStop(kLeftPwmPin);
    softPwmStop(kRightPwmPin);
  }

 private:
  void twistCallback(const geometry_msgs::Twist::ConstPtr& msg) {

    float speed_Right = msg->linear.x + msg->angular.z;
    float speed_Left = msg->linear.x - msg->angular.z;
    
    // Clamp to [-1, 1] just in case
    if(speed_Right > 1) speed_Right = 1;
    if(speed_Right < -1) speed_Right = -1;
    if(speed_Left > 1) speed_Left = 1;
    if(speed_Left < -1) speed_Left = -1;

    leftDrive(speed_Left);
    rightDrive(speed_Right);
  }
  ros::NodeHandle nh_;
  ros::Subscriber cmd_vel_sub_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "motor_control_node");
  MotorControl motor_control;
  ros::spin();
  return 0;
}
