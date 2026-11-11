#include <chrono>
#include <utility>
#include <cstdint>
#include <random>
#include <iostream>
#include <vector>
#include <cassert>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "simulated_world_information.hpp"
#include "bayes_filter/msg/world_update.hpp"


using namespace std::chrono_literals;


struct Position {
  uint8_t x;
  uint8_t y;
};

class SimulatedWorld : public rclcpp::Node
{
public:
  SimulatedWorld()
  : Node("simulated_world"),
  dist(0, SimulatedWorldInformation::POSSIBLE_COMMANDS.size() - 1),
  prob(0.0, 1.0)
  {
    position = Position{ SimulatedWorldInformation::INIT_POSITION_X, SimulatedWorldInformation::INIT_POSITION_Y };
    std::random_device rd{};
    randgen = std::mt19937{rd()};

    publisher_ = this->create_publisher<bayes_filter::msg::WorldUpdate>("world_update", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    timer_ = this->create_wall_timer(0.33s, std::bind(&SimulatedWorld::publish_updated_world, this));
  }

private:
  void publish_updated_world()
  {
    const std::string & command = SimulatedWorldInformation::POSSIBLE_COMMANDS[dist(randgen)];

    update_position(command);

    publish_position_as_tf();

    const SimulatedWorldInformation::Color detected_color = detect_color();

    publish_world_update(command, detected_color);
  }

  void publish_world_update(const std::string & command, const SimulatedWorldInformation::Color det_color)
  {
    bayes_filter::msg::WorldUpdate world_update;

    world_update.command = command;
    world_update.detected_color = det_color;

    publisher_->publish(world_update);
  }

  SimulatedWorldInformation::Color detect_color()
  {
    SimulatedWorldInformation::Color real_color = SimulatedWorldInformation::field_color(position.x, position.y);

    SimulatedWorldInformation::Color detected_color;
    if (SimulatedWorldInformation::Color::BLACK == real_color) {
      if (prob(randgen) < SimulatedWorldInformation::PROB_BLACK_CORRECT) {
        detected_color = SimulatedWorldInformation::Color::BLACK;
      }
      else {
        detected_color = SimulatedWorldInformation::Color::WHITE;
      }
    }
    else if (SimulatedWorldInformation::Color::WHITE == real_color) {
      if (prob(randgen) < SimulatedWorldInformation::PROB_WHITE_CORRECT) {
        detected_color = SimulatedWorldInformation::Color::WHITE;
      }
      else {
        detected_color = SimulatedWorldInformation::Color::BLACK;
      }
    }
    else {
      assert(false);
    }
    return detected_color;
  }

  void publish_position_as_tf()
  {
    geometry_msgs::msg::TransformStamped tf_msg;

    tf_msg.header.stamp = this->now();

    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "robot";

    tf_msg.transform.translation.x = (static_cast<float>(position.x) + 0.5) / 10.0;
    tf_msg.transform.translation.y = (static_cast<float>(position.y) + 0.5) / 10.0;

    tf_broadcaster_->sendTransform(tf_msg);
  }

  void update_position(const std::string & command)
  {
    const uint8_t MAX_POS = SimulatedWorldInformation::MAX_POSITION;
    const uint8_t MIN_POS = SimulatedWorldInformation::MIN_POSITION;

    if (command == "TR") {
      if (!(position.x + 2 > MAX_POS || position.y + 1 > MAX_POS) && move_success()) {
        position.x += 2;
        position.y += 1;
      }
    }
    else if (command == "D") {
      if (position.y > MIN_POS && move_success()) {
        position.y += -1;
      }
    }
    else if (command == "L") {
      if (position.x > MIN_POS && move_success()) {
        position.x += -1;
      }
    }
    else {
      assert(false);
    }
  }

  bool move_success()
  {
    return prob(randgen) < SimulatedWorldInformation::PROB_MOVE_SUCCESS;
  }

  Position position;
  std::mt19937 randgen;
  std::uniform_int_distribution<std::size_t> dist;
  std::uniform_real_distribution<float> prob;
  rclcpp::Publisher<bayes_filter::msg::WorldUpdate>::SharedPtr publisher_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};



int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimulatedWorld>());
  rclcpp::shutdown();

  return 0;
}

// vim: shiftwidth=2 expandtab
