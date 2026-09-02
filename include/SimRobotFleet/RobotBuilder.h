#pragma once
#include <iostream>
#include "Kinematics.h"
#include "KinematicsBicycle.h"
#include "KinematicsDifferential.h"
#include "KinematicsSimpleSim.h"
#include "Robot.h"
#include <memory>
#include <stdexcept>



class RobotBuilder {
    Pose m_Pose{0.0, 0.0, 0.0};
    double m_Speed = 5.0;
    double m_Dimension = 10.0; // wheelBase (Bicycle) or width (Differential); unused for SimpleSim
    RobotKinematicModel m_KinematicModel = RobotKinematicModel::SIMPLESIM;
    RobotMotionState m_MotionState = RobotMotionState::STOP;
    RobotMissionState m_MissionState = RobotMissionState::ACTIVE;

public:
    RobotBuilder() = default;
    RobotBuilder& SetPose(Pose pose_);
    RobotBuilder& SetKinematicModel(RobotKinematicModel model_);
    RobotBuilder& SetDimension(double dimension_);
    RobotBuilder& SetSpeed(double speed_);
    RobotBuilder& SetMotionState(RobotMotionState state_);
    RobotBuilder& SetMissionState(RobotMissionState state_);

    std::unique_ptr<Robot> Build() const;
};