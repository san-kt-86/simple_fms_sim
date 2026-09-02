#pragma once
#include <iostream>
#include <memory>
#include <shared_mutex>
#include "ID.h"
#include "Pose.h"
#include "KinematicsSimpleSim.h"
#include "Trajectory.h"
#include "Helpers.h"
#include "SimulationCycle.h"



class Robot{
    const size_t m_Id;
    Pose m_Pose;
    Trajectory<Pose> m_Path;
    size_t m_CurrentWaypointIdx;
    const std::unique_ptr<Kinematics<Robot>> m_Kinematics;
    RobotMotionState m_MotionState;
    RobotMissionState m_MissionState;
    mutable std::shared_mutex m_Mtx;

    friend class RobotBuilder;

    Robot(Pose startPose_, Trajectory<Pose> path_,
          std::unique_ptr<Kinematics<Robot>> kinematics_,
          RobotMotionState initialState_, RobotMissionState missionState_);

public:
    Robot(const Robot&) = delete;
    Robot(Robot&&) = delete;
    Robot& operator=(const Robot&) = delete;
    Robot& operator=(Robot&&) = delete;

    void Progress(double dt_);

    bool IsTrajectoryFinished();
    
    size_t GetId() const;
    Pose GetPose() const;
    Trajectory<Pose>& GetTrajectory();
    const Trajectory<Pose>& GetTrajectory() const;
    size_t GetCurrentWaypointIdx() const ;
    RobotMotionState GetRobotMotionState() const;
    RobotMissionState GetRobotMissionState() const;
    const Kinematics<Robot>* GetKinematics() const;
    std::shared_mutex& GetMutex() const;
	

    void SetPose(Pose p_);
	void SetTrajectory(Trajectory<Pose> path_);
    void SetRobotMotionState(RobotMotionState motionState_);
    void SetRobotMissionState(RobotMissionState missionState_);
    void IncrementWaypointIdx();
};