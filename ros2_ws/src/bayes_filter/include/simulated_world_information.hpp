#ifndef SIMULATED_WORLD_INFORMATION_HPP
#define SIMULATED_WORLD_INFORMATION_HPP

#include <vector>
#include "bayes_filter/msg/world_update.hpp"

namespace SimulatedWorldInformation {
  enum Color: uint8_t {
    BLACK = bayes_filter::msg::WorldUpdate::BLACK,
    WHITE = bayes_filter::msg::WorldUpdate::WHITE
  };

  const static uint8_t MAX_POSITION = 32 - 1;
  const static uint8_t MIN_POSITION = 0;

  const static uint8_t INIT_POSITION_X = 16;
  const static uint8_t INIT_POSITION_Y = 16;

  const static std::vector<std::string> POSSIBLE_COMMANDS{"TR", "D", "L"};
  const static float PROB_MOVE_SUCCESS = 0.9;

  const static float PROB_WHITE_CORRECT = 0.8;
  const static float PROB_BLACK_CORRECT = 0.7;

  static Color field_color(uint8_t x, uint8_t y) {
    return static_cast<Color>((x + y) % 2);
  }
}

#endif // SIMULATED_WORLD_INFORMATION_HPP

// vim: shiftwidth=2 expandtab
