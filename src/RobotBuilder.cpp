#include "RobotBuilder.h"


RobotBuilder& RobotBuilder::SetPose(Pose pose_) { m_Pose = pose_; return *this; }
RobotBuilder& RobotBuilder::SetKinematicModel(RobotKinematicModel model_) { m_KinematicModel = model_; return *this; }
RobotBuilder& RobotBuilder::SetDimension(double dimension_) { m_Dimension = dimension_; return *this; }
RobotBuilder& RobotBuilder::SetSpeed(double speed_) { m_Speed = speed_; return *this; }
RobotBuilder& RobotBuilder::SetMotionState(RobotMotionState state_) { m_MotionState = state_; return *this; }
RobotBuilder& RobotBuilder::SetMissionState(RobotMissionState state_) { m_MissionState = state_; return *this; }

std::unique_ptr<Robot> RobotBuilder::Build() const {
    std::unique_ptr<Kinematics<Robot>> kinematics;
    switch (m_KinematicModel) {
        case RobotKinematicModel::BICYCLE:
            kinematics = std::make_unique<KinematicsBicycle<Robot>>(m_Dimension, 0.0, m_Speed);
            break;
        case RobotKinematicModel::DIFFERENTIAL:
            kinematics = std::make_unique<KinematicsDifferential<Robot>>(m_Dimension, 0.0, m_Speed);
            break;
        case RobotKinematicModel::SIMPLESIM:
            kinematics = std::make_unique<SimpleSimKinematics<Robot>>(m_Speed);
            break;
        default:
            throw std::logic_error("RobotBuilder: unhandled kinematics model");
    }

    return std::unique_ptr<Robot>(new Robot(m_Pose, Trajectory<Pose>{}, std::move(kinematics), m_MotionState, m_MissionState));
}
