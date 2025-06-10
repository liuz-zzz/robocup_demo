#include <cmath>

#include "brain.h"
#include "robot_client.h"

#include "utils/math.h"
#include "utils/print.h"
#include "utils/misc.h"

#include "booster_msgs/message_utils.h"
#include <sensor_msgs/point_cloud2_iterator.hpp>

void RobotClient::init()
{
    publisher = brain->create_publisher<booster_msgs::msg::RpcReqMsg>("LocoApiTopicReq", 10);
}

int RobotClient::moveHead(double pitch, double yaw)
{
    yaw = cap(yaw, brain->config->headYawLimitLeft, brain->config->headYawLimitRight);
    pitch = max(pitch, brain->config->headPitchLimitUp);

    booster_msgs::msg::RpcReqMsg msg = booster_msgs::CreateRotateHeadMsg(pitch, yaw);
    publisher->publish(msg);
    return 0;
}

int RobotClient::waveHand(bool doWaveHand)
{
    auto msg = booster_msgs::CreateWaveHandMsg(booster::robot::b1::HandIndex::kRightHand, doWaveHand ? booster::robot::b1::HandAction::kHandOpen : booster::robot::b1::HandAction::kHandClose);
    publisher->publish(msg);
    return 0;
}

int RobotClient::standUp()
{
    auto msg = booster_msgs::CreateGetUpMsg();
    publisher->publish(msg);
    return 0;
}

int RobotClient::enterDamping()
{
    auto msg = booster_msgs::CreateChangeModeMsg(booster::robot::RobotMode::kDamping);
    publisher->publish(msg);
    return 0;
}

int RobotClient::setVelocity(double x, double y, double theta, bool applyMinX, bool applyMinY, bool applyMinTheta)
{
    brain->log->setTimeNow();
    brain->log->log("RobotClient/setVelocity_in",
                    rerun::TextLog(format("vx: %.2f  vy: %.2f  vtheta: %.2f", x, y, theta)));

    rerun::Collection<rerun::Vec2D> vxLine = {{0, 0}, {x, 0}};
    rerun::Collection<rerun::Vec2D> vyLine = {{0, 0}, {0, -y}};
    rerun::Collection<rerun::Vec2D> vthetaLine = {{0, 0}, {2.0 * cos(-theta), 2.0 * sin(-theta)}};

    brain->log->log("robotframe/velocity",
                    rerun::LineStrips2D({vxLine, vyLine, vthetaLine})
                        .with_colors({0xFF0000FF, 0x00FF00FF, 0x0000FFFF})
                        .with_radii({0.05, 0.05, 0.02})
                        .with_draw_order(1.0));


    double minx = 0.05, miny = 0.08, mintheta = 0.05;
    if (applyMinX && fabs(x) < minx && fabs(x) > 1e-5)
        x = x > 0 ? minx : -minx;
    if (applyMinY && fabs(y) < miny && fabs(y) > 1e-5)
        y = y > 0 ? miny : -miny;
    if (applyMinTheta && fabs(theta) < mintheta && fabs(theta) > 1e-5)
        theta = theta > 0 ? mintheta : -mintheta;
    x = cap(x, brain->config->vxLimit, -brain->config->vxLimit);
    y = cap(y, brain->config->vyLimit, -brain->config->vyLimit);
    theta = cap(theta, brain->config->vthetaLimit, -brain->config->vthetaLimit);

    brain->log->log("RobotClient/setVelocity_out",
                    rerun::TextLog(format("vx: %.2f  vy: %.2f  vtheta: %.2f", x, y, theta)));

    auto msg = booster_msgs::CreateMoveMsg(x, y, theta);
    publisher->publish(msg);
    return 0;
}

int RobotClient::moveToPoseOnField(double tx, double ty, double ttheta, double longRangeThreshold, double turnThreshold, double vxLimit, double vyLimit, double vthetaLimit, double xTolerance, double yTolerance, double thetaTolerance)
{
    Pose2D target_f, target_r; 
    target_f.x = tx;
    target_f.y = ty;
    target_f.theta = ttheta;
    target_r = brain->data->field2robot(target_f);
    double targetAngle = atan2(target_r.y, target_r.x);
    double targetDist = norm(target_r.x, target_r.y);

    double vx, vy, vtheta;

    if (
        (fabs(brain->data->robotPoseToField.x - target_f.x) < xTolerance) && (fabs(brain->data->robotPoseToField.y - target_f.y) < yTolerance) && (fabs(toPInPI(brain->data->robotPoseToField.theta - target_f.theta)) < thetaTolerance))
    {
        return setVelocity(0, 0, 0);
    }

    static double breakOscillate = 0.0;
    if (targetDist > longRangeThreshold - breakOscillate)
    {
        breakOscillate = 0.5;

        if (fabs(targetAngle) > turnThreshold)
        {
            vtheta = cap(targetAngle, vthetaLimit, -vthetaLimit);
            return setVelocity(0, 0, vtheta);
        }

        // else

        vx = cap(target_r.x, vxLimit, -vxLimit);
        vtheta = cap(targetAngle, vthetaLimit, -vthetaLimit);
        return setVelocity(vx, 0, vtheta, false, false, false);
    }

    breakOscillate = 0.0;
    vx = cap(target_r.x, vxLimit, -vxLimit);
    vy = cap(target_r.y, vyLimit, -vyLimit);
    vtheta = cap(target_r.theta, vthetaLimit, -vthetaLimit);
    return setVelocity(vx, vy, vtheta);
}

int RobotClient::moveToPoseOnField1(double tx, double ty, double ttheta, double longRangeThreshold, double turnThreshold, 
                                   double vxLimit, double vyLimit, double vthetaLimit, 
                                   double xTolerance, double yTolerance, double thetaTolerance)
{
    // 定义静态变量用于避障
    static bool isAvoiding = false;
    static rclcpp::Time avoidStartTime;
    static const double TURN_DURATION = 2.0;  // 左转持续时间，单位：秒
    static const double FORWARD_DURATION = 2.0;  // 前进持续时间，单位：秒
    static bool isTurning = true;  // 用于区分是在左转阶段还是前进阶段
    
    // 获取深度图像数据
    auto depth_image = brain->getLatestDepthImage();
    
    // 如果正在避障中，检查是否已经避障足够时间
    if (isAvoiding) {
        double elapsedTime = brain->msecsSince(avoidStartTime) / 1000.0; // 转换为秒
        
        if (isTurning) {
            // 左转阶段
            if (elapsedTime < TURN_DURATION) {
                // 继续左转
                return setVelocity(0.0, 0.0, vthetaLimit);
            } else {
                // 左转阶段结束，开始前进阶段
                isTurning = false;
                avoidStartTime = brain->get_clock()->now();
                return setVelocity(vxLimit * 0.5, 0.0, 0.0);  // 开始前进，使用一半的速度
            }
        } else {
            // 前进阶段
            if (elapsedTime < FORWARD_DURATION) {
                // 继续前进
                return setVelocity(vxLimit * 0.5, 0.0, 0.0);
            } else {
                // 避障完成，重置状态
                isAvoiding = false;
                isTurning = true;  // 重置为左转阶段，为下次避障做准备
                
                // 重新计算目标方向并继续导航
                double tarDir = atan2(ty - brain->data->robotPoseToField.y, 
                                    tx - brain->data->robotPoseToField.x);
                double faceDir = brain->data->robotPoseToField.theta;
                double tarDir_r = toPInPI(tarDir - faceDir);
                return setVelocity(0.0, 0.0, tarDir_r);  // 转向目标点
            }
        }
    }
    
    // 检查是否需要开始避障
    if (depth_image != nullptr) {
        // 定义感兴趣区域（ROI）
        int roi_x = depth_image->width / 2 - 50;  // ROI中心区域
        int roi_y = depth_image->height / 2 - 50;
        int roi_width = 100;
        int roi_height = 100;
        
        // 计算ROI区域内的平均深度
        float sum_depth = 0;
        int valid_points = 0;
        
        // 将ROS图像消息转换为OpenCV格式以便处理
        cv::Mat depth_mat(depth_image->height, depth_image->width, CV_32FC1, 
                         const_cast<float*>(reinterpret_cast<const float*>(depth_image->data.data())));
        
        for (int y = roi_y; y < roi_y + roi_height && y < depth_image->height; y++) {
            for (int x = roi_x; x < roi_x + roi_width && x < depth_image->width; x++) {
                float depth = depth_mat.at<float>(y, x);
                if (depth > 0.01 && depth < 5.0) {  // 忽略0值和异常值
                    sum_depth += depth;
                    valid_points++;
                }
            }
        }
        
        if (valid_points > 100) {  // 确保有足够多的有效点
            float avg_depth = sum_depth / valid_points;
            if (avg_depth < 1.0) {  // 如果1米内有障碍物
                // 开始避障
                isAvoiding = true;
                isTurning = true;  // 从左转阶段开始
                avoidStartTime = brain->get_clock()->now();
                return setVelocity(0.0, 0.0, vthetaLimit);  // 开始左转
            }
        }
    }
    
    // 如果没有深度图像数据或没有检测到障碍物，使用普通的moveToPose逻辑
    return moveToPoseOnField(tx, ty, ttheta, longRangeThreshold, turnThreshold,
                            vxLimit, vyLimit, vthetaLimit,
                            xTolerance, yTolerance, thetaTolerance);
}
