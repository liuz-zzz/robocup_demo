#pragma once

#include "types.h"

#define VALIDATION_COMMUNICATION 31201
#define VALIDATION_DISCOVERY 41202
struct TeamCommunicationMsg
{
    int validation = VALIDATION_COMMUNICATION; // validate msg, to determine if it's sent by us.
    int communicationId;
    int teamId;
    int playerId;
    // 球位置信息
    bool ballDetected;        // 是否检测到球
    double ballX;             // 球在场地坐标系中的X坐标
    double ballY;             // 球在场地坐标系中的Y坐标
    rclcpp::Time ballTime;    // 球位置的时间戳
    // TODO: You need to add something you want to send to teammates
    int testInfo;
};

struct TeamDiscoveryMsg
{
    int validation = VALIDATION_DISCOVERY; // validate msg, to determine if it's sent by us.
    int communicationId;
    int teamId;
    int playerId;
};
