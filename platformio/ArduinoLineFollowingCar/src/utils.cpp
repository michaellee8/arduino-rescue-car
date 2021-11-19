#include "utils.h"
#include "line_follow_robot_consts.h"

char* convertDirectionToString(Direction v){
    switch (v)
    {
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

char* convertSideToString(Side v){
    switch (v)
    {
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