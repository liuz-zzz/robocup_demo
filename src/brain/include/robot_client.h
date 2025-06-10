#pragma once

#include <iostream>
#include <string>
#include <rerun.hpp>

#include <booster_msgs/msg/rpc_req_msg.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>

using namespace std;

class Brain; // Forward declaration

/**
 * The RobotClient class. All operations for controlling the robot by calling the RobotSDK are placed here.
 */
class RobotClient
{
public:
    RobotClient(Brain *argBrain) : brain(argBrain) {}

    void init();

    /**
     * @brief Move the robot's head.
     *
     * @param pitch
     * @param yaw
     *
     * @return int, 0 indicates successful execution.
     */
    int moveHead(double pitch, double yaw);

    /**
     * @brief Set the moving speed of the robot.
     *
     * @param x double, forward (m/s).
     * @param y double, leftward (m/s).
     * @param theta double, counterclockwise rotation angle (rad/s).
     * @param applyMinX, applyMinY, applyMinTheta bool Whether to adjust the command size to prevent non-response when the speed command is too small.
     *
     * @return int, 0 indicates successful execution.
     *
     */
    int setVelocity(double x, double y, double theta, bool applyMinX = true, bool applyMinY = true, bool applyMinTheta = true);

    /**
     * @brief Walk towards a certain Pose in the pitch coordinate system in speed mode. Note that the final orientation should also be reached.
     *
     * @param tx, ty, ttheta double, the target Pose, in the Field coordinate system.
     * @param longRangeThreshold double, when the distance exceeds this value, it is preferred to turn towards the target point and walk over instead of directly adjusting the position.
     * @param turnThreshold double, when the angle difference between the direction of the target point (note that it is not the final orientation ttheta) and the current angle is greater than this value, first turn to face the target.
     * @param vxLimit, vyLimit, vthetaLimit double, the upper limits of speeds in each direction, m/s, rad/s.
     * @param xTolerance, yTolerance, thetaTolerance double, the tolerances for judging that the target point has been reached.
     *
     * @return int The return value of the motion control command, 0 represents success.
     */
    int moveToPoseOnField(double tx, double ty, double ttheta, double longRangeThreshold, double turnThreshold, double vxLimit, double vyLimit, double vthetaLimit, double xTolerance, double yTolerance, double thetaTolerance);

    /**
     * @brief 带避障功能的moveToPoseOnField，使用深度相机检测前方障碍物
     * 
     * @param tx, ty, ttheta double, the target Pose, in the Field coordinate system.
     * @param longRangeThreshold double, when the distance exceeds this value, it is preferred to turn towards the target point and walk over instead of directly adjusting the position.
     * @param turnThreshold double, when the angle difference between the direction of the target point (note that it is not the final orientation ttheta) and the current angle is greater than this value, first turn to face the target.
     * @param vxLimit, vyLimit, vthetaLimit double, the upper limits of speeds in each direction, m/s, rad/s.
     * @param xTolerance, yTolerance, thetaTolerance double, the tolerances for judging that the target point has been reached.
     * 
     * @return int The return value of the motion control command, 0 represents success.
     */
    int moveToPoseOnField1(double tx, double ty, double ttheta, double longRangeThreshold = 1.0, double turnThreshold = 0.4, 
                          double vxLimit = 0.3, double vyLimit = 0.2, double vthetaLimit = 0.8,
                          double xTolerance = 0.1, double yTolerance = 0.1, double thetaTolerance = 0.1);

    /**
     * @brief Wave the hand.
     */
    int waveHand(bool doWaveHand);

    /**
     * @brief 起身
     */
    int standUp();

    /**
     * @brief 进阻尼
     */
    int enterDamping();

private:
    rclcpp::Publisher<booster_msgs::msg::RpcReqMsg>::SharedPtr publisher;
    Brain *brain;
};