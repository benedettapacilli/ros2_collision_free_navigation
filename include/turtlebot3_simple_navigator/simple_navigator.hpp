#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <random>

class SimpleNavigator : public rclcpp::Node {
public:
  SimpleNavigator();

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void controlLoop();

  double getRandomTurn();
  double sampleGaussian(double mean, double stddev);
  void scheduleNextTumble();

  struct AvoidanceResult {
    double min_front;
    double min_any;
    double torque;
    bool   clear_front;
  };
  AvoidanceResult computeAvoidance();

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::LaserScan last_scan_;
  bool have_scan_{false};

  double forward_speed_{0.2};
  double turn_speed_{0.8};
  double safety_distance_{0.4};
  double front_angle_range_{0.5};
  double wander_std_{0.1};
  double tumble_mean_interval_{5.0};
  double tumble_max_angle_{M_PI/6.0};
  double influence_distance_{0.8};
  double avoid_gain_{1.2};

  std::random_device rd_;
  std::mt19937 gen_;
  std::uniform_real_distribution<double> turn_dist_;
  std::normal_distribution<double> gaussian_{0.0, 1.0};
  std::exponential_distribution<double> exp_{1.0};

  rclcpp::Time next_tumble_time_{0,0,RCL_ROS_TIME};
  bool have_next_tumble_{false};

  enum class State { MOVING_FORWARD, TURNING };
  State current_state_{State::MOVING_FORWARD};
  double target_turn_amount_{0.0};
  double accumulated_turn_{0.0};
};
