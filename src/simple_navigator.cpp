#include "turtlebot3_local_planner/simple_navigator.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

/**
 * @brief Construct the SimpleNavigator node.
 *
 * Declares and reads motion and parameters; sets up
 * LaserScan subscription, publisher, and a 20 Hz control timer.
 *
 * @note Ensures @c influence_distance_ > @c safety_distance_ for sane avoidance.
 * @post Timer is armed, subscriptions/publishers active, RNG seeded.
 */
SimpleNavigator::SimpleNavigator()
  : Node("simple_navigator")
  , gen_(rd_())
  , turn_dist_(-M_PI/2, M_PI/2)
{
  declare_parameter<double>("forward_speed", forward_speed_);
  declare_parameter<double>("turn_speed", turn_speed_);
  declare_parameter<double>("safety_distance", safety_distance_);
  declare_parameter<double>("front_angle_range", front_angle_range_);

  declare_parameter<double>("wander_std", wander_std_);
  declare_parameter<double>("tumble_mean_interval", tumble_mean_interval_);
  declare_parameter<double>("tumble_max_angle", tumble_max_angle_);

  declare_parameter<double>("influence_distance", influence_distance_);
  declare_parameter<double>("avoid_gain",          avoid_gain_);

  get_parameter("forward_speed", forward_speed_);
  get_parameter("turn_speed", turn_speed_);
  get_parameter("safety_distance", safety_distance_);
  get_parameter("front_angle_range", front_angle_range_);

  get_parameter("wander_std", wander_std_);
  get_parameter("tumble_mean_interval", tumble_mean_interval_);
  get_parameter("tumble_max_angle", tumble_max_angle_);

  get_parameter("influence_distance", influence_distance_);
  get_parameter("avoid_gain", avoid_gain_);

  influence_distance_ = std::max(influence_distance_, safety_distance_ + 0.05);

  if (tumble_mean_interval_ <= 0.1) tumble_mean_interval_ = 0.1;
  exp_ = std::exponential_distribution<double>(1.0 / tumble_mean_interval_);

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS(),
    std::bind(&SimpleNavigator::scanCallback, this, std::placeholders::_1));
  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&SimpleNavigator::controlLoop, this));

  scheduleNextTumble();

  RCLCPP_INFO(get_logger(), "Simple Navigator started");
  RCLCPP_INFO(get_logger(), "Forward speed: %.2f m/s | Turn cap: %.2f rad/s", forward_speed_, turn_speed_);
  RCLCPP_INFO(get_logger(), "Safety: %.2f m | Influence: %.2f m | Avoid gain: %.2f",
              safety_distance_, influence_distance_, avoid_gain_);
}

/**
 * @brief LaserScan callback.
 */
void SimpleNavigator::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  last_scan_ = *msg;
  have_scan_ = true;
}

double SimpleNavigator::getRandomTurn() { return turn_dist_(gen_); }

double SimpleNavigator::sampleGaussian(double mean, double stddev) {
  return mean + stddev * gaussian_(gen_);
}

void SimpleNavigator::scheduleNextTumble() {
  rclcpp::Time now = this->now();
  double dt = std::max(0.2, exp_(gen_));
  next_tumble_time_ = now + rclcpp::Duration::from_seconds(dt);
  have_next_tumble_ = true;
}

/**
 * @brief Unified omnidirectional avoidance + front status.
 *
 * Scans all rays to compute a repulsive steering “torque” from obstacles
 * within @c influence_distance_. Tracks min range in the forward cone and
 * across the full scan, and sets @c clear_front to true iff no forward-cone
 * ray is below @c safety_distance_.
 *
 * If no scan is available, returns {Inf, Inf, 0.0, false}.
 */
SimpleNavigator::AvoidanceResult SimpleNavigator::computeAvoidance() {
  AvoidanceResult out;
  out.min_front   = std::numeric_limits<double>::infinity();
  out.min_any     = std::numeric_limits<double>::infinity();
  out.torque      = 0.0;
  out.clear_front = false;

  if (!have_scan_ || last_scan_.ranges.empty()) {
    return out;
  }

  const auto& s = last_scan_;
  const int N = static_cast<int>(s.ranges.size());

  bool any_front_below_safety = false;

  for (int i = 0; i < N; ++i) {
    const double a = s.angle_min + i * s.angle_increment;
    const double r = s.ranges[i];
    if (std::isnan(r) || std::isinf(r) || r <= 0.0) continue;

    if (r < out.min_any) out.min_any = r;

    if (std::abs(a) <= front_angle_range_) {
      if (r < out.min_front) out.min_front = r;
      if (r < safety_distance_) any_front_below_safety = true;
    }

    if (r < influence_distance_) {
      const double w = (1.0 / r) - (1.0 / influence_distance_);
      out.torque += -w * std::sin(a);
    }
  }

  if (!std::isfinite(out.min_front)) out.min_front = std::numeric_limits<double>::infinity();
  if (!std::isfinite(out.min_any))   out.min_any   = std::numeric_limits<double>::infinity();

  out.clear_front = !any_front_below_safety;
  return out;
}

/**
 * @brief Main control loop running at 20 Hz.
 */
void SimpleNavigator::controlLoop() {
  if (!have_scan_) return;

  geometry_msgs::msg::Twist cmd;
  const double dt = 0.05;

  const auto av = computeAvoidance();

  switch (current_state_) {
    case State::MOVING_FORWARD: {
      const bool time_to_tumble = have_next_tumble_ && (this->now() >= next_tumble_time_);

      if (!av.clear_front) {
        current_state_        = State::TURNING;
        target_turn_amount_   = getRandomTurn();
        accumulated_turn_     = 0.0;
        cmd.linear.x = 0.0;
        cmd.angular.z = (target_turn_amount_ > 0) ? turn_speed_ : -turn_speed_;
        RCLCPP_INFO(get_logger(), "Front blocked (min_front=%.2f). Turning %.1f°",
                    std::isfinite(av.min_front) ? av.min_front : -1.0,
                    target_turn_amount_ * 180.0 / M_PI);
      } else if (time_to_tumble) {
        std::uniform_real_distribution<double> small(-tumble_max_angle_, tumble_max_angle_);
        current_state_      = State::TURNING;
        target_turn_amount_ = small(gen_);
        accumulated_turn_   = 0.0;
        cmd.linear.x = 0.0;
        cmd.angular.z = (target_turn_amount_ > 0) ? turn_speed_ : -turn_speed_;
        scheduleNextTumble();
        RCLCPP_INFO(get_logger(), "Tumble: %.1f°", target_turn_amount_ * 180.0 / M_PI);
      } else {
        double speed_scale = 1.0;
        if (std::isfinite(av.min_front)) {
          if (av.min_front <= safety_distance_) {
            speed_scale = 0.0;
          } else if (av.min_front < influence_distance_) {
            const double num = (av.min_front - safety_distance_);
            const double den = (influence_distance_ - safety_distance_);
            speed_scale = std::clamp(num / den, 0.0, 1.0);
          }
        }

        cmd.linear.x  = forward_speed_ * speed_scale;

        double omega  = avoid_gain_ * av.torque + sampleGaussian(0.0, wander_std_);
        omega = std::clamp(omega, -turn_speed_, turn_speed_);
        cmd.angular.z = omega;
      }
      break;
    }

    case State::TURNING: {
      const double turn_increment = turn_speed_ * dt;
      const bool turning_positive = (target_turn_amount_ > 0);

      if (std::abs(accumulated_turn_) < std::abs(target_turn_amount_)) {
        cmd.linear.x  = 0.0;
        cmd.angular.z = turning_positive ? turn_speed_ : -turn_speed_;
        accumulated_turn_ += turning_positive ? turn_increment : -turn_increment;

        const double min_turn = 10.0 * M_PI / 180.0;
        if (std::abs(accumulated_turn_) >= min_turn && av.clear_front) {
          current_state_ = State::MOVING_FORWARD;
          cmd.linear.x   = forward_speed_;
          cmd.angular.z  = 0.0;
          RCLCPP_INFO(get_logger(), "Early-exit: path clear after %.1f°",
                      accumulated_turn_ * 180.0 / M_PI);
        }
      } else {
        current_state_ = State::MOVING_FORWARD;
        cmd.linear.x   = forward_speed_;
        cmd.angular.z  = 0.0;
        RCLCPP_INFO(get_logger(), "Turn complete");
      }
      break;
    }
  }

  cmd_pub_->publish(cmd);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleNavigator>());
  rclcpp::shutdown();
  return 0;
}
