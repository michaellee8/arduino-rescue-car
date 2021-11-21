#include "utils.h"

#include "line_follow_robot_consts.h"

const char* ConvertDirectionToString(Direction v) {
  switch (v) {
    case Direction::kLeft:
      return "LEFT";
      break;
    case Direction::kForward:
      return "FORWARD";
      break;
    case Direction::kRight:
      return "RIGHT";
      break;
    default:
      return "";
      break;
  }
  return "";
}

const char* ConvertSideToString(Side v) {
  switch (v) {
    case Side::kLeft:
      return "LEFT";
      break;
    case Side::kMiddle:
      return "MIDDLE";
      break;
    case Side::kRight:
      return "RIGHT";
      break;
    default:
      return "";
      break;
  }
  return "";
}