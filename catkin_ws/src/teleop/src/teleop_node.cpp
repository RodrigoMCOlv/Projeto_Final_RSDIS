#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <termios.h>
#include <unistd.h>
#include <unordered_map>


struct KeyBinding {
  double linear;
  double angular;
};

const std::unordered_map<char, KeyBinding> kKeyBindings{
    {'u', {0.3,  0.2}},
    {'i', {0.3,  0.0}},
    {'w', {0.3,  0.0}},
    {'o', {0.3, -0.2}},
    {'j', {0.0,  0.2}},
    {'q', {0.0,  0.2}},
    {'k', {0.0,  0.0}},
    {' ', {0.0,  0.0}},
    {'l', {0.0, -0.2}},
    {'e', {0.0, -0.2}},
    {'m', {-0.3, -0.2}},
    {',', {-0.3,  0.0}},
    {'s', {-0.3,  0.0}},
    {'.', {-0.3,  0.2}},
};


class Teleop {
 public:
     Teleop(ros::NodeHandle& nh) {
        cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 20);
    }
     // Function to get a key press from the keyboard
    char getKeyPress() {
        struct termios oldt, newt;        
        tcgetattr(STDIN_FILENO, &oldt); // Get the current terminal settings
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO); // Disable canonical mode and echo
        newt.c_cc[VMIN] = 0;
        newt.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &newt); // Apply new settings

        int ch = getchar(); // Read a single character from the keyboard

        tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // Restore the old terminal settings
        if (ch == EOF) {
          clearerr(stdin);       // reset EOF state so future reads work
        }
        return ch;
    }

  void run() {
    ros::Rate loop_rate(10);
    while (ros::ok()) {
      geometry_msgs::Twist cmd;            // defaults to zero
      int key = getKeyPress();
      if (key != EOF) {
        auto it = kKeyBindings.find(static_cast<char>(key));
        if (it != kKeyBindings.end()) {
          cmd.linear.x  = it->second.linear;
          cmd.angular.z = it->second.angular;
          std::cout << "Key " << static_cast<char>(key)
                    << " -> vx=" << cmd.linear.x
                    << " wz=" << cmd.angular.z << std::endl;
        }
      }
      cmd_pub.publish(cmd);                // publish zero when idle
      ros::spinOnce();
      loop_rate.sleep();
    }
  }

  private:
  ros::NodeHandle nh_;
  ros::Publisher cmd_pub;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "teleop_node");
  ros::NodeHandle nh;
  Teleop teleop(nh);
  teleop.run();
  return 0;
}
