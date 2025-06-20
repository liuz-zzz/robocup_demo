#include <cmath>
#include "brain_tree.h"
#include "brain.h"
#include "utils/math.h"
#include "utils/print.h"
#include "utils/misc.h"
#include "std_msgs/msg/string.hpp"

/**
 * Here, a macro definition is used to reduce the amount of code in RegisterBuilder.
 * The effect after expanding REGISTER_BUILDER(Test) is as follows:
 * factory.registerBuilder<Test>(  \
 *      "Test",                    \
 *      [this](const string& name, const NodeConfig& config) { return make_unique<Test>(name, config, brain); });
 */
#define REGISTER_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [this](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });

void BrainTree::init()
{
    BehaviorTreeFactory factory;

    // Action Nodes
    REGISTER_BUILDER(RobotFindBall)
    REGISTER_BUILDER(RobotFindBall1)
    REGISTER_BUILDER(Chase)
    REGISTER_BUILDER(Chase1)
    REGISTER_BUILDER(SimpleChase)
    REGISTER_BUILDER(Adjust)
    REGISTER_BUILDER(Adjust1)
    REGISTER_BUILDER(Kick)
    REGISTER_BUILDER(StrikerDecide)
    REGISTER_BUILDER(CamTrackBall)
    REGISTER_BUILDER(CamFindBall)
    REGISTER_BUILDER(CamScanField)
    REGISTER_BUILDER(SelfLocate)
    REGISTER_BUILDER(SetVelocity)
    REGISTER_BUILDER(CheckAndStandUp)
    REGISTER_BUILDER(RotateForRelocate)
    REGISTER_BUILDER(MoveToPoseOnField)
    REGISTER_BUILDER(GoalieDecide)
    REGISTER_BUILDER(WaveHand)
    REGISTER_BUILDER(GoBackInField)
    REGISTER_BUILDER(TurnOnSpot)
    // 添加speak节点
    REGISTER_BUILDER(Speak)
    // Action Nodes for debug
    REGISTER_BUILDER(PrintMsg)

    factory.registerBehaviorTreeFromFile(brain->config->treeFilePath);
    tree = factory.createTree("MainTree");

    // init blackboard entry
    initEntry();
}

void BrainTree::initEntry()
{
    setEntry<string>("player_role", brain->config->playerRole);
    setEntry<bool>("ball_location_known", false);
    setEntry<bool>("track_ball", true);
    setEntry<bool>("odom_calibrated", false);
    setEntry<string>("decision", "");
    setEntry<string>("defend_decision", "chase");
    setEntry<double>("ball_range", 0);

    setEntry<bool>("gamecontroller_isKickOff", true);
    setEntry<bool>("gamecontroller_isKickOffExecuted", true);

    setEntry<string>("gc_game_state", "");
    setEntry<string>("gc_game_sub_state_type", "NONE");
    setEntry<string>("gc_game_sub_state", "");
    setEntry<bool>("gc_is_kickoff_side", false);
    setEntry<bool>("gc_is_sub_state_kickoff_side", false);
    setEntry<bool>("gc_is_under_penalty", false);

    setEntry<bool>("treat_person_as_robot", false);
    setEntry<int>("control_state", 0);
    setEntry<bool>("B_pressed", false);

    // fallRecovery相关
    setEntry<bool>("should_recalibrate_after_fall_recovery", false);

    setEntry<bool>("we_just_scored", false);
    setEntry<bool>("wait_for_opponent_kickoff", false);

    //调试相关：
    setEntry<double>("Vx_now",0);
    setEntry<double>("Vy_now",0);
    setEntry<double>("yaw_now",0);

}

void BrainTree::tick()
{
    static int cnt = 0;
    cnt++;
    // print states
    if (cnt % 30 == 0)
        prtDebug(format(
            "GameState: %s\tIsKickOffSide: %d\nScore: %d\t JustScored: %d\nball_range:%f\tVx_now:%f\tVy_now:%f\tyaw_now:%f\n decide_now:%s",
            getEntry<string>("gc_game_state").c_str(),
            getEntry<bool>("gc_is_kickoff_side"),
            brain->data->lastScore,
            getEntry<bool>("we_just_scored"),
            getEntry<double>("ball_range"),
            getEntry<double>("Vx_now"),
            getEntry<double>("Vy_now"),
            getEntry<double>("yaw_now"),
            getEntry<string>("decision").c_str()

             ));
    tree.tickOnce();
}

NodeStatus SetVelocity::tick()
{
    double x, y, theta;
    vector<double> targetVec;
    getInput("x", x);
    getInput("y", y);
    getInput("theta", theta);

    auto res = brain->client->setVelocity(x, y, theta);
    return NodeStatus::SUCCESS;
}

NodeStatus CamTrackBall::tick()
{
    double pitch, yaw;
    if (!brain->data->ballDetected)
    {
        pitch = brain->data->ball.pitchToRobot;
        yaw = brain->data->ball.yawToRobot;
    }
    else
    {
        const double pixTolerance = 10;

        double deltaX = mean(brain->data->ball.boundingBox.xmax, brain->data->ball.boundingBox.xmin) - brain->config->camPixX / 2;
        double deltaY = mean(brain->data->ball.boundingBox.ymax, brain->data->ball.boundingBox.ymin) - brain->config->camPixY * 2 / 3;

        if (std::fabs(deltaX) < pixTolerance && std::fabs(deltaY) < pixTolerance)
        {
            return NodeStatus::SUCCESS;
        }

        double smoother = 1.5;
        double deltaYaw = deltaX / brain->config->camPixX * brain->config->camAngleX / smoother;
        double deltaPitch = deltaY / brain->config->camPixY * brain->config->camAngleY / smoother;

        pitch = brain->data->headPitch + deltaPitch;
        yaw = brain->data->headYaw - deltaYaw;
    }

    brain->client->moveHead(pitch, yaw);
    return NodeStatus::SUCCESS;
}

CamFindBall::CamFindBall(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain)
{
    double lowPitch = 0.8;
    double highPitch = 0.3;
    double leftYaw = 0.55;
    double rightYaw = -0.55;

    _cmdSequence[0][0] = lowPitch;
    _cmdSequence[0][1] = leftYaw;
    _cmdSequence[1][0] = lowPitch;
    _cmdSequence[1][1] = 0;
    _cmdSequence[2][0] = lowPitch;
    _cmdSequence[2][1] = rightYaw;
    _cmdSequence[3][0] = highPitch;
    _cmdSequence[3][1] = rightYaw;
    _cmdSequence[4][0] = highPitch;
    _cmdSequence[4][1] = 0;
    _cmdSequence[5][0] = highPitch;
    _cmdSequence[5][1] = leftYaw;

    _cmdIndex = 0;
    _cmdIntervalMSec = 800;
    _cmdRestartIntervalMSec = 50000;
    _timeLastCmd = brain->get_clock()->now();
}

NodeStatus CamFindBall::tick()
{
    if (brain->data->ballDetected)
    {
        return NodeStatus::SUCCESS;
    }

    auto curTime = brain->get_clock()->now();
    auto timeSinceLastCmd = (curTime - _timeLastCmd).nanoseconds() / 1e6;
    if (timeSinceLastCmd < _cmdIntervalMSec)
    {
        return NodeStatus::SUCCESS;
    }
    else if (timeSinceLastCmd > _cmdRestartIntervalMSec)
    {
        _cmdIndex = 0;
    }
    else
    {
        _cmdIndex = (_cmdIndex + 1) % (sizeof(_cmdSequence) / sizeof(_cmdSequence[0]));
    }

    brain->client->moveHead(_cmdSequence[_cmdIndex][0], _cmdSequence[_cmdIndex][1]);
    _timeLastCmd = brain->get_clock()->now();
    return NodeStatus::SUCCESS;
}

NodeStatus CamScanField::tick()
{
    auto sec = brain->get_clock()->now().seconds();
    auto msec = static_cast<unsigned long long>(sec * 1000);
    double lowPitch, highPitch, leftYaw, rightYaw;
    getInput("low_pitch", lowPitch);
    getInput("high_pitch", highPitch);
    getInput("left_yaw", leftYaw);
    getInput("right_yaw", rightYaw);
    int msecCycle;
    getInput("msec_cycle", msecCycle);

    int cycleTime = msec % msecCycle;
    double pitch = cycleTime > (msecCycle / 2.0) ? lowPitch : highPitch;
    double yaw = cycleTime < (msecCycle / 2.0) ? (leftYaw - rightYaw) * (2.0 * cycleTime / msecCycle) + rightYaw : (leftYaw - rightYaw) * (2.0 * (msecCycle - cycleTime) / msecCycle) + rightYaw;

    brain->client->moveHead(pitch, yaw);
    return NodeStatus::SUCCESS;
}

NodeStatus Chase::tick()
{
    if (!brain->tree->getEntry<bool>("ball_location_known"))
    {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }
    double vxLimit, vyLimit, vthetaLimit, dist;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("dist", dist);

    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;

    Pose2D target_f, target_r;
    if (brain->data->robotPoseToField.x - brain->data->ball.posToField.x > (_state == "chase" ? 1.0 : 0.0))
    {
        _state = "circle_back";

        target_f.x = brain->data->ball.posToField.x - dist;

        if (brain->data->robotPoseToField.y > brain->data->ball.posToField.y - _dir)
            _dir = 1.0;
        else
            _dir = -1.0;

        target_f.y = brain->data->ball.posToField.y + _dir * dist;
    }
    else
    { // chase
        _state = "chase";
        target_f.x = brain->data->ball.posToField.x - dist;
        target_f.y = brain->data->ball.posToField.y;
    }

    target_r = brain->data->field2robot(target_f);

    double vx = target_r.x;
    double vy = target_r.y;
    double vtheta = ballYaw * 2.0;

    double linearFactor = 1 / (1 + exp(3 * (ballRange * fabs(ballYaw)) - 3));
    vx *= linearFactor;
    vy *= linearFactor;

    vx = cap(vx, vxLimit, -vxLimit);
    vy = cap(vy, vyLimit, -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);

    brain->client->setVelocity(vx, vy, vtheta, false, false, false);
    return NodeStatus::SUCCESS;
}

NodeStatus SimpleChase::tick()
{
    double stopDist, stopAngle, vyLimit, vxLimit;
    getInput("stop_dist", stopDist);
    getInput("stop_angle", stopAngle);
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);

    if (!brain->tree->getEntry<bool>("ball_location_known"))
    {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    double vx = brain->data->ball.posToRobot.x;
    double vy = brain->data->ball.posToRobot.y;
    double vtheta = brain->data->ball.yawToRobot * 2.0;

    double linearFactor = 1 / (1 + exp(3 * (brain->data->ball.range * fabs(brain->data->ball.yawToRobot)) - 3));
    vx *= linearFactor;
    vy *= linearFactor;

    vx = cap(vx, vxLimit, -0.1);
    vy = cap(vy, vyLimit, -vyLimit);

    if (brain->data->ball.range < stopDist)
    {
        vx = 0;
        vy = 0;
    }

    brain->client->setVelocity(vx, vy, vtheta, false, false, false);
    return NodeStatus::SUCCESS;
}

NodeStatus Adjust::tick()
{
    if (!brain->tree->getEntry<bool>("ball_location_known"))
    {
        return NodeStatus::SUCCESS;
    }

    double turnThreshold, vxLimit, vyLimit, vthetaLimit, maxRange, minRange;
    getInput("turn_threshold", turnThreshold);
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("max_range", maxRange);
    getInput("min_range", minRange);
    string position;
    getInput("position", position);

    double vx = 0, vy = 0, vtheta = 0;
    double kickDir = (position == "defense") ? atan2(brain->data->ball.posToField.y, brain->data->ball.posToField.x + brain->config->fieldDimensions.length / 2) : atan2(-brain->data->ball.posToField.y, brain->config->fieldDimensions.length / 2 - brain->data->ball.posToField.x);
    double dir_rb_f = brain->data->robotBallAngleToField;
    double deltaDir = toPInPI(kickDir - dir_rb_f);
    double dir = deltaDir > 0 ? -1.0 : 1.0;
    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;

    double s = 0.4;
    double r = 0.8;
    vx = -s * dir * sin(ballYaw);
    if (ballRange > maxRange)
        vx += 0.1;
    if (ballRange < maxRange)
        vx -= 0.1;
    vy = s * dir * cos(ballYaw);
    vtheta = (ballYaw - dir * s) / r;

    vx = cap(vx, vxLimit, -vxLimit);
    vy = cap(vy, vyLimit, -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);

    brain->client->setVelocity(vx, vy, vtheta);
    return NodeStatus::SUCCESS;
}
NodeStatus Adjust1::tick()
{
if (!brain->tree->getEntry<bool>("ball_location_known"))
{
return NodeStatus::SUCCESS;
}

double turnThreshold, vxLimit, vyLimit, vthetaLimit, maxRange, minRange;
getInput("turn_threshold", turnThreshold);
getInput("vx_limit", vxLimit);
getInput("vy_limit", vyLimit);
getInput("vtheta_limit", vthetaLimit);
getInput("max_range", maxRange);
getInput("min_range", minRange);
string position;
getInput("position", position);

double vx = 0, vy = 0, vtheta = 0;
double kickDir = (position == "defense") ? atan2(brain->data->ball.posToField.y, brain->data->ball.posToField.x + brain->config->fieldDimensions.length / 2) : atan2(-brain->data->ball.posToField.y, brain->config->fieldDimensions.length / 2 - brain->data->ball.posToField.x);
double dir_rb_f = brain->data->robotBallAngleToField;
double deltaDir = toPInPI(kickDir - dir_rb_f);
double dir = deltaDir > 0 ? -1.0 : 1.0;
double ballRange = brain->data->ball.range;
double ballYaw = brain->data->ball.yawToRobot;

// 增加速度系数，提高响应速度
double s = 0.7; // 原来是0.4，提高到0.7
double r = 0.6; // 原来是0.8，减小到0.6以提高旋转速度

// 计算角度差的绝对值，用于调整速度
double absDeltaDir = fabs(deltaDir);

// 根据角度差动态调整速度系数
// 角度差越大，速度系数越大，以加快调整
double speedFactor = 1.0 + absDeltaDir * 0.5; // 最大可达到1.0 + π * 0.5 ≈ 2.57
s *= speedFactor;

// 计算前进/后退速度
vx = -s * dir * sin(ballYaw);

// 根据与目标距离的差值更激进地调整前进/后退速度
if (ballRange > maxRange)
vx += 0.3; // 原来是0.1，提高到0.3
else if (ballRange < minRange)
vx -= 0.3; // 原来是0.1，提高到0.3

// 计算横向移动速度
vy = s * dir * cos(ballYaw);

// 计算旋转速度，更激进地调整方向
vtheta = (ballYaw - dir * s) / r;

// 当角度差较大时，优先调整方向
if (absDeltaDir > 0.5) { // 角度差超过约30度
// 增加旋转分量
vtheta *= 1.5;
// 减小线性运动分量，以避免过度移动
vx *= 0.8;
vy *= 0.8;
}

// 应用速度限制
vx = cap(vx, vxLimit, -vxLimit);
vy = cap(vy, vyLimit, -vyLimit);
vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);

// 记录调试信息
brain->log->logToScreen("Adjust",
format("Delta: %.2f, Range: %.2f, Speed: [%.2f, %.2f, %.2f]",
deltaDir, ballRange, vx, vy, vtheta),
0x00FFFFFF);

brain->client->setVelocity(vx, vy, vtheta);
return NodeStatus::SUCCESS;
}

NodeStatus StrikerDecide::tick()
{

    double chaseRangeThreshold;
    getInput("chase_threshold", chaseRangeThreshold);
    string lastDecision, position;
    getInput("decision_in", lastDecision);
    getInput("position", position);

    double kickDir = (position == "defense") ? atan2(brain->data->ball.posToField.y, brain->data->ball.posToField.x + brain->config->fieldDimensions.length / 2) : atan2(-brain->data->ball.posToField.y, brain->config->fieldDimensions.length / 2 - brain->data->ball.posToField.x);
    double dir_rb_f = brain->data->robotBallAngleToField;
    auto goalPostAngles = brain->getGoalPostAngles(0.3);
    double theta_l = goalPostAngles[0];
    double theta_r = goalPostAngles[1];
    bool angleIsGood = (theta_l > dir_rb_f && theta_r < dir_rb_f);
    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;

    string newDecision;
    auto color = 0xFFFFFFFF; // for log
    if (!brain->tree->getEntry<bool>("ball_location_known"))
    {
        newDecision = "find";
        color = 0x0000FFFF;
    }
    else if (ballRange > chaseRangeThreshold * (lastDecision == "chase" ? 0.9 : 1.0))
    {
        newDecision = "chase";
        color = 0x00FF00FF;
    }
    else if (angleIsGood)
    {
        newDecision = "kick";
        color = 0xFF0000FF;
    }
    else
    {
        newDecision = "adjust";
        color = 0x00FFFFFF;
    }

    setOutput("decision_out", newDecision);
    brain->log->logToScreen("tree/Decide",
                            format("Decision: %s ballrange: %.2f ballyaw: %.2f kickDir: %.2f rbDir: %.2f angleIsGood: %d", newDecision.c_str(), ballRange, ballYaw, kickDir, dir_rb_f, angleIsGood),
                            color);
    return NodeStatus::SUCCESS;
}

NodeStatus CheckAndStandUp::tick()
{
    if (brain->tree->getEntry<bool>("gc_is_under_penalty") || brain->data->currentRobotModeIndex == 1) {
        brain->data->needManualRelocate = false;
        brain->tree->setEntry<bool>("should_recalibrate_after_fall_recovery", false);
        brain->data->recoveryPerformed = false;
        brain->data->enterDampingPerformed = false;
        brain->log->log("recovery", rerun::TextLog("reset recovery"));
        return NodeStatus::SUCCESS;
    }
    
    if (brain->data->needManualRelocate)
    {
        brain->log->log("recovery", rerun::TextLog("need manual relocate"));
        return NodeStatus::FAILURE;
    }
    
    if (brain->data->recoveryState == RobotRecoveryState::HAS_FALLEN &&
        // brain->data->isRecoveryAvailable && // 倒了就直接尝试RL起身，（不需要关注是否recoveryAailable）
        brain->data->currentRobotModeIndex != 1 && // not in prepare
        !brain->data->recoveryPerformed &&
        !brain->data->enterDampingPerformed) {
        brain->client->standUp();
        brain->data->recoveryPerformed = true;
        brain->data->lastRecoveryTime = brain->get_clock()->now();
        brain->log->log("recovery", rerun::TextLog("Fall detect and stand up"));
    }

    // 如果没有起来, 且已经过了 5 秒, 就进入阻尼模式，且只进入一次
    auto now = brain->get_clock()->now();
    auto seconds_elaps = now.seconds() - brain->data->lastRecoveryTime.seconds();
    if (brain->data->recoveryPerformed &&
        !brain->data->enterDampingPerformed &&
        seconds_elaps >10 &&
        brain->data->recoveryState != RobotRecoveryState::IS_READY) {

        brain->client->enterDamping();
        brain->data->enterDampingPerformed = true;
        brain->tree->setEntry<bool>("should_recalibrate_after_fall_recovery", false);
        brain->log->log("recovery", rerun::TextLog("Enter Damping, seconds_elaps: " + to_string(seconds_elaps) +
        "recoveryState: " + to_string(static_cast<int>(brain->data->recoveryState))));

        // std::cout << "Enter Damping, seconds_elaps: " << seconds_elaps << " recoveryState: " << static_cast<int>(brain->data->recoveryState) << std::endl;
    }

    if (brain->data->recoveryPerformed &&
        !brain->data->enterDampingPerformed &&
        brain->data->recoveryState == RobotRecoveryState::IS_READY) {
        brain->tree->setEntry<bool>("should_recalibrate_after_fall_recovery", true);
        brain->log->log("recovery", rerun::TextLog("Standup success, seconds_elaps: " + to_string(seconds_elaps) +
        "recoveryState: " + to_string(static_cast<int>(brain->data->recoveryState))));
    }

    // 机器人站着且是robocup步态，可以重置跌到爬起的状态
    if (brain->data->recoveryState == RobotRecoveryState::IS_READY &&
        brain->data->currentRobotModeIndex == 8) { // in robocup gait
        brain->data->recoveryPerformed = false;
        brain->data->enterDampingPerformed = false;
        brain->log->log("recovery", rerun::TextLog("Reset recovery, recoveryState: " + to_string(static_cast<int>(brain->data->recoveryState))));
    }

    return NodeStatus::SUCCESS;
}

NodeStatus RotateForRelocate::onStart()
{
    this->_lastSuccessfulLocalizeTime = brain->data->lastSuccessfulLocalizeTime;
    this->_startTime = brain->get_clock()->now();
    return NodeStatus::RUNNING;
}

NodeStatus RotateForRelocate::onRunning()
{
    double vyaw_limit;
    getInput("vyaw_limit", vyaw_limit);
    int max_msec_locate;
    getInput("max_msec_locate", max_msec_locate);
    
    brain->client->moveHead(0.4, 0.0);
    brain->client->setVelocity(0, 0, vyaw_limit);

    if (this->_lastSuccessfulLocalizeTime.nanoseconds() != brain->data->lastSuccessfulLocalizeTime.nanoseconds()) {
        brain->tree->setEntry<bool>("should_recalibrate_after_fall_recovery", false);
        brain->log->log("recovery", rerun::TextLog("Relocated successfully"));
        return NodeStatus::SUCCESS;
    }

    if (brain->msecsSince(this->_startTime) > max_msec_locate) {
        brain->tree->setEntry<bool>("should_recalibrate_after_fall_recovery", false);
        brain->data->needManualRelocate = true;
        brain->client->enterDamping();
        brain->log->log("recovery", rerun::TextLog("Relocated failed for timeout"));
        return NodeStatus::SUCCESS;
    }

    return NodeStatus::RUNNING;
}

void RotateForRelocate::onHalted()
{
    brain->tree->setEntry<bool>("should_recalibrate_after_fall_recovery", false);
}


NodeStatus GoalieDecide::tick()
{

    double chaseRangeThreshold;
    getInput("chase_threshold", chaseRangeThreshold);
    string lastDecision, position;
    getInput("decision_in", lastDecision);

    double kickDir = atan2(brain->data->ball.posToField.y, brain->data->ball.posToField.x + brain->config->fieldDimensions.length / 2);
    double dir_rb_f = brain->data->robotBallAngleToField;
    auto goalPostAngles = brain->getGoalPostAngles(0.3);
    double theta_l = goalPostAngles[0];
    double theta_r = goalPostAngles[1];
    bool angleIsGood = (dir_rb_f > -M_PI / 2 && dir_rb_f < M_PI / 2);
    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;

    string newDecision;
    auto color = 0xFFFFFFFF; // for log
    if (!brain->tree->getEntry<bool>("ball_location_known"))
    {
        newDecision = "find";
        color = 0x0000FFFF;
    }
    else if (brain->data->ball.posToField.x > 0 - static_cast<double>(lastDecision == "retreat"))
    {
        newDecision = "retreat";
        color = 0xFF00FFFF;
    }
    else if (ballRange > chaseRangeThreshold * (lastDecision == "chase" ? 0.9 : 1.0))
    {
        newDecision = "chase";
        color = 0x00FF00FF;
    }
    else if (angleIsGood)
    {
        newDecision = "kick";
        color = 0xFF0000FF;
    }
    else
    {
        newDecision = "adjust";
        color = 0x00FFFFFF;
    }

    setOutput("decision_out", newDecision);
    brain->log->logToScreen("tree/Decide",
                            format("Decision: %s ballrange: %.2f ballyaw: %.2f kickDir: %.2f rbDir: %.2f angleIsGood: %d", newDecision.c_str(), ballRange, ballYaw, kickDir, dir_rb_f, angleIsGood),
                            color);
    return NodeStatus::SUCCESS;
}

NodeStatus Kick::onStart()
{
    _startTime = brain->get_clock()->now();

    double vxLimit, vyLimit;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    int minMSecKick;
    getInput("min_msec_kick", minMSecKick);
    double vxFactor = brain->config->vxFactor;
    double yawOffset = brain->config->yawOffset;

    double adjustedYaw = brain->data->ball.yawToRobot - yawOffset;
    double tx = cos(adjustedYaw) * brain->data->ball.range;
    double ty = sin(adjustedYaw) * brain->data->ball.range;

    double vx, vy;

    if (fabs(ty) < 0.01 && fabs(adjustedYaw) < 0.01)
    {
        vx = vxLimit;
        vy = 0.0;
    }
    else
    {
        vy = ty > 0 ? vyLimit : -vyLimit;
        vx = vy / ty * tx * vxFactor;
        if (fabs(vx) > vxLimit)
        {
            vy *= vxLimit / vx;
            vx = vxLimit;
        }
    }

    double speed = norm(vx, vy);

    _msecKick = speed > 1e-5 ? minMSecKick + static_cast<int>(brain->data->ball.range / speed * 1000) : minMSecKick;

    brain->client->setVelocity(vx, vy, 0, false, false, false);
    return NodeStatus::RUNNING;
}

NodeStatus Kick::onRunning()
{
    if (brain->msecsSince(_startTime) < _msecKick)
        return NodeStatus::RUNNING;

    brain->client->setVelocity(0, 0, 0);
    return NodeStatus::SUCCESS;
}

void Kick::onHalted()
{
    _startTime -= rclcpp::Duration(100, 0);
}

NodeStatus RobotFindBall::onStart()
{
    if (brain->data->ballDetected)
    {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }
    _turnDir = brain->data->ball.yawToRobot > 0 ? 1.0 : -1.0;

    return NodeStatus::RUNNING;
}

NodeStatus RobotFindBall::onRunning()
{
    if (brain->data->ballDetected)
    {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    double vyawLimit;
    getInput("vyaw_limit", vyawLimit);

    double vx = 0;
    double vy = 0;
    double vtheta = 0;
    brain->client->setVelocity(0, 0, vyawLimit * _turnDir);
    return NodeStatus::RUNNING;
}

void RobotFindBall::onHalted()
{
    _turnDir = 1.0;
}

NodeStatus RobotFindBall1::onStart()
{
    if (brain->data->ballDetected)
    {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }
    
    // 使用球的历史位置信息来决定初始旋转方向
    bool useLastBallPos;
    getInput("use_last_ball_pos", useLastBallPos);
    
    if (useLastBallPos && brain->data->ball.timePoint.seconds() > 0) {
        // 如果球之前被看到过，根据球的位置决定初始旋转方向
        double ballYaw = atan2(brain->data->ball.posToRobot.y, brain->data->ball.posToRobot.x);
        _turnDir = (ballYaw > 0) ? 1.0 : -1.0;
        brain->log->log("debug/find_ball", rerun::TextLog(format(
            "Using last ball position for initial direction: %.2f", _turnDir
        )));
    } else {
        // 随机选择初始方向
        _turnDir = ((rand() % 2) == 0) ? 1.0 : -1.0;
        brain->log->log("debug/find_ball", rerun::TextLog("Random initial direction"));
    }
    
    _startTime = brain->get_clock()->now();
    _lastDirectionChangeTime = brain->get_clock()->now();
    
    return NodeStatus::RUNNING;
}

NodeStatus RobotFindBall1::onRunning()
{
    if (brain->data->ballDetected)
    {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    double vyawLimit, scanTime;
    getInput("vyaw_limit", vyawLimit);
    getInput("scan_time", scanTime);
    
    // 计算搜索已经进行的时间
    auto currentTime = brain->get_clock()->now();
    double elapsedTime = (currentTime - _startTime).seconds();
    double timeSinceLastDirectionChange = (currentTime - _lastDirectionChangeTime).seconds();
    
    // 如果搜索时间超过设定值，改变搜索方向
    if (timeSinceLastDirectionChange > scanTime) {
        _turnDir = -_turnDir; // 反转方向
        _lastDirectionChangeTime = currentTime;
        
        // 记录日志
        brain->log->logToScreen("RobotFindBall1", 
            format("Changed direction after %.1f seconds, new dir: %.1f", 
                   timeSinceLastDirectionChange, _turnDir),
            0x00FFFFFF);
    }
    
    // 根据搜索时间动态调整转速
    double speedFactor = 1.0;
    if (elapsedTime > scanTime * 2) {
        // 长时间未找到球，增加转速
        speedFactor = 1.2;
    }
    
    // 设置机器人速度，使用更高的转速
    brain->client->setVelocity(0, 0, vyawLimit * _turnDir * speedFactor);
    
    // 记录搜索状态
    if (int(elapsedTime) % 5 == 0) { // 每5秒记录一次
        brain->log->logToScreen("RobotFindBall1", 
            format("Searching for ball: %.1f seconds, dir: %.1f, speed: %.2f", 
                   elapsedTime, _turnDir, vyawLimit * speedFactor),
            0x0000FFFF);
    }
    
    return NodeStatus::RUNNING;
}

void RobotFindBall1::onHalted()
{
    _turnDir = 1.0;
}

NodeStatus SelfLocate::tick()
{
    string mode = getInput<string>("mode").value();
    double xMin = 0.0, xMax = 0.0, yMin = 0, yMax = 0.0, thetaMin = 0.0, thetaMax = 0.0;
    auto markers = brain->data->getMarkers();

    if (mode == "enter_field")
    {

        xMin = -brain->config->fieldDimensions.length / 2;
        xMax = -brain->config->fieldDimensions.circleRadius;

        if (brain->config->playerStartPos == "left")
        {
            yMin = brain->config->fieldDimensions.width / 2;
            yMax = brain->config->fieldDimensions.width / 2 + 1.0;
        }
        else if (brain->config->playerStartPos == "right")
        {
            yMin = -brain->config->fieldDimensions.width / 2 - 1.0;
            yMax = -brain->config->fieldDimensions.width / 2;
        }

        if (brain->config->playerStartPos == "left")
        {
            thetaMin = -M_PI / 2 - M_PI / 6;
            thetaMax = -M_PI / 2 + M_PI / 6;
        }
        else if (brain->config->playerStartPos == "right")
        {
            thetaMin = M_PI / 2 - M_PI / 6;
            thetaMax = M_PI / 2 + M_PI / 6;
        }
    }
    else if (mode == "face_forward")
    {
        xMin = -brain->config->fieldDimensions.length / 2;
        xMax = brain->config->fieldDimensions.length / 2;
        yMin = -brain->config->fieldDimensions.width / 2;
        yMax = brain->config->fieldDimensions.width / 2;
        thetaMin = -M_PI / 4;
        thetaMax = M_PI / 4;
    }
    else if (mode == "trust_direction")
    {
        int msec = static_cast<int>(brain->msecsSince(brain->data->lastSuccessfulLocalizeTime));
        double maxDriftSpeed = 0.1;
        double maxDrift = msec / 1000.0 * maxDriftSpeed;

        xMin = max(-brain->config->fieldDimensions.length / 2, brain->data->robotPoseToField.x - maxDrift);
        xMax = min(brain->config->fieldDimensions.length / 2, brain->data->robotPoseToField.x + maxDrift);
        yMin = max(-brain->config->fieldDimensions.width / 2, brain->data->robotPoseToField.y - maxDrift);
        yMax = min(brain->config->fieldDimensions.width / 2, brain->data->robotPoseToField.y + maxDrift);
        thetaMin = brain->data->robotPoseToField.theta - M_PI / 18;
        thetaMax = brain->data->robotPoseToField.theta + M_PI / 18;
    } 
    else if (mode == "fall_recovery") 
    {
        int msec = static_cast<int>(brain->msecsSince(brain->data->lastSuccessfulLocalizeTime));
        double maxDriftSpeed = 0.1;                      // m/s
        double maxDrift = msec / 1000.0 * maxDriftSpeed; // 在这个时间内, odom 最多漂移了多少距离

        xMin = -brain->config->fieldDimensions.length / 2 - 2;
        xMax = brain->config->fieldDimensions.length / 2 + 2;
        yMin = -brain->config->fieldDimensions.width / 2 - 2;
        yMax = brain->config->fieldDimensions.width / 2 + 2;
        thetaMin = brain->data->robotPoseToField.theta - M_PI / 180;
        thetaMax = brain->data->robotPoseToField.theta + M_PI / 180;
    }


    // TODO other modes

    // Locate
    PoseBox2D constraints{xMin, xMax, yMin, yMax, thetaMin, thetaMax};
    double residual;
    auto res = brain->locator->locateRobot(markers, constraints);

    // if (brain->config->rerunLogEnable) {
    if (false)
    {
        brain->log->setTimeNow();
        brain->log->log("locator/time",
                        rerun::Scalar(res.msecs));
        brain->log->log("locator/residual",
                        rerun::Scalar(res.residual));
        brain->log->log("locator/result",
                        rerun::Scalar(res.code));
        brain->log->log("locator/constraints",
                        rerun::TextLog(
                            "xMin: " + to_string(xMin) + " " +
                            "xMax: " + to_string(xMax) + " " +
                            "yMin: " + to_string(yMin) + " " +
                            "yMax: " + to_string(yMax) + " " +
                            "thetaMin: " + to_string(thetaMin) + " " +
                            "thetaMax: " + to_string(thetaMax)));
    }
    prtDebug("locate result: res: " + to_string(res.code) + " time: " + to_string(res.msecs));

    if (!res.success)
        return NodeStatus::SUCCESS; // Do not block following nodes.

    brain->calibrateOdom(res.pose.x, res.pose.y, res.pose.theta);
    brain->tree->setEntry<bool>("odom_calibrated", true);
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    prtDebug("locate success: " + to_string(res.pose.x) + " " + to_string(res.pose.y) + " " + to_string(rad2deg(res.pose.theta)) + " Dur: " + to_string(res.msecs));

    return NodeStatus::SUCCESS;
}

NodeStatus MoveToPoseOnField::tick()
{
    double tx, ty, ttheta, longRangeThreshold, turnThreshold, vxLimit, vyLimit, vthetaLimit, xTolerance, yTolerance, thetaTolerance;
    getInput("x", tx);
    getInput("y", ty);
    getInput("theta", ttheta);
    getInput("long_range_threshold", longRangeThreshold);
    getInput("turn_threshold", turnThreshold);
    getInput("vx_limit", vxLimit);
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("x_tolerance", xTolerance);
    getInput("y_tolerance", yTolerance);
    getInput("theta_tolerance", thetaTolerance);

    brain->client->moveToPoseOnField(tx, ty, ttheta, longRangeThreshold, turnThreshold, vxLimit, vyLimit, vthetaLimit, xTolerance, yTolerance, thetaTolerance);
    return NodeStatus::SUCCESS;
}

NodeStatus WaveHand::tick()
{
    string action;
    getInput("action", action);
    if (action == "start")
        brain->client->waveHand(true);
    else
        brain->client->waveHand(false);
    return NodeStatus::SUCCESS;
}

NodeStatus GoBackInField::tick()
{
    double valve;
    getInput("valve", valve);
    double vx = 0; 
    double vy = 0; 
    double dir = 0;
    auto fd = brain->config->fieldDimensions;
    if (brain->data->robotPoseToField.x > fd.length / 2.0 - valve) dir = - M_PI;
    else if (brain->data->robotPoseToField.x < - fd.length / 2.0 + valve) dir = 0;
    else if (brain->data->robotPoseToField.y > fd.width / 2.0 + valve) dir = - M_PI / 2.0;
    else if (brain->data->robotPoseToField.y < - fd.width / 2.0 - valve) dir = M_PI / 2.0;
    else { // 没出界
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    // 出界了, 往回走
    double dir_r = toPInPI(dir - brain->data->robotPoseToField.theta);
    vx = 0.4 * cos(dir_r);
    vy = 0.4 * sin(dir_r);
    brain->client->setVelocity(vx, vy, 0, false, false, false);
    return NodeStatus::SUCCESS;
}

NodeStatus TurnOnSpot::onStart()
{
    _timeStart = brain->get_clock()->now();
    _lastAngle = brain->data->robotPoseToOdom.theta;
    _cumAngle = 0.0;

    bool towardsBall = false;
    _angle = getInput<double>("rad").value();
    getInput("towards_ball", towardsBall);
    if (towardsBall) {
        double ballPixX = (brain->data->ball.boundingBox.xmin + brain->data->ball.boundingBox.xmax) / 2;
        _angle = fabs(_angle) * (ballPixX < brain->config->camPixX / 2 ? 1 : -1);
    }

    brain->client->setVelocity(0, 0, _angle, false, false, true);
    return NodeStatus::RUNNING;
}

NodeStatus TurnOnSpot::onRunning()
{
    double curAngle = brain->data->robotPoseToOdom.theta;
    double deltaAngle = toPInPI(curAngle - _lastAngle);
    _lastAngle = curAngle;
    _cumAngle += deltaAngle;
    double turnTime = brain->msecsSince(_timeStart);
    brain->log->log("debug/turn_on_spot", rerun::TextLog(format(
        "angle: %.2f, cumAngle: %.2f, deltaAngle: %.2f, time: %.2f",
        _angle, _cumAngle, deltaAngle, turnTime
    )));
    if (
        fabs(_cumAngle) - fabs(_angle) > -0.1
        || turnTime > _msecLimit
    ) {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }
    // else 
    // brain->client->setVelocity(0, 0, (_angle - _cumAngle)*2);

    //TODO
    // 增加速度系数，使旋转更快
    double speedFactor = 2.5;  // 增加基础速度系数
    
    // 当剩余角度较大时使用更高速度，接近目标时逐渐减速
    double remainingAngle = fabs(_angle - _cumAngle);
    if (remainingAngle > 1.0) {  // 如果剩余超过1弧度（约57度）
        speedFactor = 3.0;  // 使用更高的速度
    }
    
    // 计算转向速度，保持原有方向
    double turnSpeed = (_angle - _cumAngle) * speedFactor;
    
    // 设置最小和最大速度限制
    double minSpeed = 1.5;  // 最小速度提高到1.5 rad/s
    double maxSpeed = 3.0;  // 最大速度提高到3.0 rad/s
    
    if (fabs(turnSpeed) < minSpeed) {
        turnSpeed = (turnSpeed > 0) ? minSpeed : -minSpeed;
    }
    if (fabs(turnSpeed) > maxSpeed) {
        turnSpeed = (turnSpeed > 0) ? maxSpeed : -maxSpeed;
    }
    
    // 记录日志
    brain->log->logToScreen("TurnOnSpot",
        format("Remaining: %.2f, Speed: %.2f", remainingAngle, turnSpeed),
        0x00FFFFFF);
    
    brain->client->setVelocity(0, 0, turnSpeed);
    return NodeStatus::RUNNING;
}


NodeStatus PrintMsg::tick()
{
    Expected<std::string> msg = getInput<std::string>("msg");
    if (!msg)
    {
        throw RuntimeError("missing required input [msg]: ", msg.error());
    }
    std::cout << "[MSG] " << msg.value() << std::endl;
    return NodeStatus::SUCCESS;
}

NodeStatus Chase1::tick()
{
    if (!brain->tree->getEntry<bool>("ball_location_known"))
    {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    double vxLimit, vyLimit, vthetaLimit, dist;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("dist", dist);

    // 获取球的位置和速度信息
    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;
    
    // 预测球的运动
    double predictTime = 0.5; // 预测0.5秒后的位置
    
    // 使用计算好的球速度
    Pose2D ballVel;
    ballVel.x = brain->data->ball.velocityToField.x;
    ballVel.y = brain->data->ball.velocityToField.y;

    // 预测球的位置
    Pose2D predictedBallPos;
    predictedBallPos.x = brain->data->ball.posToField.x + ballVel.x * predictTime;
    predictedBallPos.y = brain->data->ball.posToField.y + ballVel.y * predictTime;

    // 计算目标位置
    Pose2D target_f, target_r;
    // double ballSpeed = sqrt(brain->data->ball.velocityToField.x * brain->data->ball.velocityToField.x +
    //                       brain->data->ball.velocityToField.y * brain->data->ball.velocityToField.y);
    // 根据球的速度判断是否需要拦截
    double ballSpeed = sqrt(ballVel.x * ballVel.x + ballVel.y * ballVel.y);
    bool needIntercept = ballSpeed > 0.5; // 当球速大于0.5m/s时进行拦截

    if (needIntercept) {
        // 拦截模式：移动到预测位置
        target_f = predictedBallPos;
        target_f.x -= dist * 0.8; // 在预测位置前方等待
    } else {
        // 常规追球模式
        if (brain->data->robotPoseToField.x - brain->data->ball.posToField.x > (_state == "chase" ? 1.0 : 0.0))
        {
            _state = "circle_back";
            target_f.x = brain->data->ball.posToField.x - dist;

            // 优化绕球方向选择
            double robotToBallAngle = atan2(
                brain->data->ball.posToField.y - brain->data->robotPoseToField.y,
                brain->data->ball.posToField.x - brain->data->robotPoseToField.x
            );
            
            // 根据机器人到球的角度选择最优绕球方向
            if (toPInPI(robotToBallAngle) > 0)
                _dir = 1.0;
            else
                _dir = -1.0;

            target_f.y = brain->data->ball.posToField.y + _dir * dist;
        }
        else
        {
            _state = "chase";
            target_f.x = brain->data->ball.posToField.x - dist;
            target_f.y = brain->data->ball.posToField.y;
        }
    }

    // 将目标位置转换到机器人坐标系
    target_r = brain->data->field2robot(target_f);

    // 计算速度
    double vx = target_r.x;
    double vy = target_r.y;
    double vtheta = ballYaw * 2.0;

    // 改进的速度控制因子
    double distFactor = 1.0 / (1.0 + exp(-4 * (ballRange - 1.0))); // 距离因子，更激进的响应
    double angleFactor = 1.0 / (1.0 + exp(3 * (fabs(ballYaw) - 0.8))); // 角度因子，更大的容忍角度
    // double ballSpeed = sqrt(brain->data->ball.velocityToField.x * brain->data->ball.velocityToField.x +
    //                       brain->data->ball.velocityToField.y * brain->data->ball.velocityToField.y);

    // 添加速度因子，当球速较快时提高响应速度
    // 使用已经计算过的ballSpeed变量
    double velocityFactor = 1.0 + 0.5 * (ballSpeed > 0.5 ? ballSpeed : 0.0); // 球速大于0.5m/s时增加响应
    
    double speedFactor = distFactor * angleFactor * velocityFactor;

    // 应用速度因子
    vx *= speedFactor;
    vy *= speedFactor;

    // 自适应平滑因子，距离近时更敏捷
    double alpha = 0.8; // 更高的平滑因子，更快的响应
    static double last_vx = 0, last_vy = 0, last_vtheta = 0;
    
    // 当方向改变较大时，减少平滑以提高响应速度
    if (last_vx * vx < 0 || last_vy * vy < 0) {
        alpha = 0.9; // 方向改变时使用更大的alpha值
    }
    
    vx = alpha * vx + (1 - alpha) * last_vx;
    vy = alpha * vy + (1 - alpha) * last_vy;
    vtheta = alpha * vtheta + (1 - alpha) * last_vtheta;

    // 保存当前速度
    last_vx = vx;
    last_vy = vy;
    last_vtheta = vtheta;

    // 限制速度
    vx = cap(vx, vxLimit, -vxLimit);
    vy = cap(vy, vyLimit, -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);

    // 输出调试信息
    brain->log->logToScreen("Chase1",
        format("State: %s, BallRange: %.2f, BallSpeed: %.2f, SpeedFactor: %.2f",
            _state.c_str(), ballRange, ballSpeed, speedFactor),
        0x00FF00FF);

    // 设置机器人速度
    brain->client->setVelocity(vx, vy, vtheta, false, false, false);
    return NodeStatus::SUCCESS;
}
NodeStatus Speak::tick()
{
    string text;
    getInput("text", text);
    brain->speak(text);
    return NodeStatus::SUCCESS;
}
