#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rdr/JHPWMPCA9685.hpp"

using std::placeholders::_1;

class MotorDriver : public rclcpp::Node
{
  public:
  PCA9685 *pca9685 = new PCA9685();
    MotorDriver()
    : Node("motor_driver")
    {
      sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&MotorDriver::topic_callback, this, _1));
      pca9685->openPCA9685();
      pca9685->setAllPWM(0,0);
      pca9685->reset();
      pca9685->setPWMFrequency(60);

    }

  private:
  int servoMin = 220;
  int servoMax = 520;
    static int map ( int x, int in_min, int in_max, int out_min, int out_max) {
        int toReturn =  (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min ;
        // For debugging:
        // printf("MAPPED %d to: %d\n", x, toReturn);
        return toReturn ;
    }

    void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
    {
      float linear_x = msg->linear.x;
      float angular_z = msg->angular.z;
      pca9685->setPWM(0, map(linear_x, -1, 1, servoMin, servoMax));
      pca9685->setPWM(1, map(angular_z, -1, 1, servoMin, servoMax));
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_cmd_vel_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorDriver>());
  rclcpp::shutdown();
  return 0;
}