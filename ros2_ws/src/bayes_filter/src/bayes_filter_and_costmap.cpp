#include <iostream>
#include <functional>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "simulated_world_information.hpp"
#include "bayes_filter/msg/world_update.hpp"

using std::placeholders::_1;

class BayesFilterAndCostmap : public rclcpp::Node
{
public:
  BayesFilterAndCostmap()
  : Node("bayes_filter_and_costmap")
  {
    // The start position is known exactly: all probability on INIT_POSITION.
    for (int x = 0; x < SIZE; x++) {
      for (int y = 0; y < SIZE; y++) {
        prob[x][y] = 0.0f;
      }
    }
    prob[SimulatedWorldInformation::INIT_POSITION_X][SimulatedWorldInformation::INIT_POSITION_Y] = 1.0f;

    publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("prob_map", 10);
    subscription_ = this->create_subscription<bayes_filter::msg::WorldUpdate>("world_update", 10,
        std::bind(&BayesFilterAndCostmap::update, this, _1));
  }

private:
  static constexpr int SIZE = SimulatedWorldInformation::MAX_POSITION + 1;
  static constexpr int MIN = SimulatedWorldInformation::MIN_POSITION;
  static constexpr int MAX = SimulatedWorldInformation::MAX_POSITION;

  void update(const bayes_filter::msg::WorldUpdate & msg) {
    update_old_prob(); // backup the prob distribution so you can read the old variant and write to a new variant
    command_update(msg);  // prediction step
    color_update(msg);    // correction step
    normalize();
    publish_costmap();
    log_estimate();
  }

  // Backup of the grid: command_update reads old_prob and writes prob.
  void update_old_prob() {
    for (int x = 0; x < SIZE; x++) {
      for (int y = 0; y < SIZE; y++) {
        old_prob[x][y] = prob[x][y];
      }
    }
  }

  // Scale the grid so that all cells sum up to 1.
  void normalize() {
    double sum = 0.0;
    for (int x = 0; x < SIZE; x++) {
      for (int y = 0; y < SIZE; y++) {
        sum += prob[x][y];
      }
    }
    if (sum <= 0.0) {
      // Every cell was ruled out (should not happen with these sensor models):
      // fall back to "could be anywhere".
      RCLCPP_WARN(get_logger(), "Probability sum is 0, resetting to a uniform distribution");
      for (int x = 0; x < SIZE; x++) {
        for (int y = 0; y < SIZE; y++) {
          prob[x][y] = 1.0f / (SIZE * SIZE);
        }
      }
      return;
    }
    for (int x = 0; x < SIZE; x++) {
      for (int y = 0; y < SIZE; y++) {
        prob[x][y] = static_cast<float>(prob[x][y] / sum);
      }
    }
  }

  // Correction: multiply every cell by p(detected color | figure is on this cell).
  void color_update(const bayes_filter::msg::WorldUpdate & msg) {
    using namespace SimulatedWorldInformation;
    const auto detected = static_cast<Color>(msg.detected_color);
    for (int x = 0; x < SIZE; x++) {
      for (int y = 0; y < SIZE; y++) {
        const Color real = field_color(x, y);
        float likelihood;
        if (real == BLACK) {
          // black field: seen as black with 0.7, as white with 0.3
          likelihood = (detected == BLACK) ? PROB_BLACK_CORRECT : 1.0f - PROB_BLACK_CORRECT;
        } else {
          // white field: seen as white with 0.8, as black with 0.2
          likelihood = (detected == WHITE) ? PROB_WHITE_CORRECT : 1.0f - PROB_WHITE_CORRECT;
        }
        prob[x][y] *= likelihood;
      }
    }
  }

  // Prediction: move the probability of every cell according to the command.
  // Same motion model as simulated_world: a move that would leave the board is
  // not attempted (figure stays), otherwise it succeeds with PROB_MOVE_SUCCESS
  // and the figure stays where it is with 1 - PROB_MOVE_SUCCESS.
  void command_update(const bayes_filter::msg::WorldUpdate & msg) {
    int dx = 0;
    int dy = 0;
    if (msg.command == "TR") {
      dx = 2;
      dy = 1;
    } else if (msg.command == "D") {
      dy = -1;
    } else if (msg.command == "L") {
      dx = -1;
    } else {
      RCLCPP_WARN(get_logger(), "Unknown command '%s', skipping prediction", msg.command.c_str());
      return;
    }

    const float p_move = SimulatedWorldInformation::PROB_MOVE_SUCCESS;
    for (int x = 0; x < SIZE; x++) {
      for (int y = 0; y < SIZE; y++) {
        prob[x][y] = 0.0f;
      }
    }
    for (int x = 0; x < SIZE; x++) {
      for (int y = 0; y < SIZE; y++) {
        const float p = old_prob[x][y];
        if (p == 0.0f) {
          continue;
        }
        const int nx = x + dx;
        const int ny = y + dy;
        if (nx < MIN || nx > MAX || ny < MIN || ny > MAX) {
          prob[x][y] += p;                 // move not possible: stays
        } else {
          prob[nx][ny] += p_move * p;      // move succeeded
          prob[x][y] += (1.0f - p_move) * p;  // move failed: stays
        }
      }
    }
  }

  // Most likely cell, for checking against the true position (TF map -> robot).
  void log_estimate() {
    int best_x = 0;
    int best_y = 0;
    for (int x = 0; x < SIZE; x++) {
      for (int y = 0; y < SIZE; y++) {
        if (prob[x][y] > prob[best_x][best_y]) {
          best_x = x;
          best_y = y;
        }
      }
    }
    RCLCPP_INFO(get_logger(), "best estimate x=%d y=%d (p=%.3f)", best_x, best_y, prob[best_x][best_y]);
  }

  void publish_costmap() {
    const uint16_t MAX_POSITION = static_cast<uint16_t>(SimulatedWorldInformation::MAX_POSITION);

    nav_msgs::msg::OccupancyGrid map;

    map.header.stamp = this->now();

    map.header.frame_id = "map";

    map.info.resolution = 0.1;
    map.info.width = MAX_POSITION + 1;
    map.info.height = MAX_POSITION + 1;

    const uint16_t vec_size = (MAX_POSITION + 1) * (MAX_POSITION + 1);
    map.data.resize(vec_size);
    int index = 0;
    for (int y = 0; y <= MAX_POSITION; y++) {
      for (int x = 0; x <= MAX_POSITION; x++) {
        map.data[index] = static_cast<int8_t>(std::pow(prob[x][y], 0.25) * 100);
        index += 1;
      }
    }
    assert(index == vec_size);

    publisher_->publish(map);
  }

  float prob[SimulatedWorldInformation::MAX_POSITION + 1][SimulatedWorldInformation::MAX_POSITION + 1];
  float old_prob[SimulatedWorldInformation::MAX_POSITION + 1][SimulatedWorldInformation::MAX_POSITION + 1];
  rclcpp::Subscription<bayes_filter::msg::WorldUpdate>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
};



int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BayesFilterAndCostmap>());
  rclcpp::shutdown();

  return 0;
}

// vim: shiftwidth=2 expandtab
