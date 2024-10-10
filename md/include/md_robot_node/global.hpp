#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <stdlib.h>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <serial/serial.h>
#include <float.h>
#include <math.h>

#include <sstream>
#include <iostream>

#ifndef GLOBAL_HPP
#define GLOBAL_HPP

// 데이터 타입 정의 (예시)
struct RobotParamDataType {
    int use_MDUI;
    int nBaudrate;
    int reverse_direction;
    int nMaxRPM;
    int enable_encoder;
    int nSlowstart;
    int nSlowdown;
    double nWheelLength;
    int nGearRatio;
    double wheel_radius;
    int encoder_PPR;
};

// 변수 선언
extern RobotParamDataType robotParamData;

#endif  // GLOBAL_HPP
using namespace std;
