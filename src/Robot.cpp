#include "Robot.h"




Robot::Robot(Pose startPose_, Trajectory<Pose> path_,
      std::unique_ptr<Kinematics<Robot>> kinematics_,
      RobotMotionState initialState_, RobotMissionState missionState_)
    : m_Id(ID::GenId()), m_Pose(startPose_), m_Path(std::move(path_)),
      m_CurrentWaypointIdx(0), m_Kinematics(std::move(kinematics_)),
      m_MotionState(initialState_), m_MissionState(missionState_) {}


void Robot::Progress(double dt_) {
    if (m_MotionState != RobotMotionState::RUN) return;

    if(!IsTrajectoryFinished())
      m_Kinematics->ForwardProgress(this, dt_);
}

bool Robot::IsTrajectoryFinished(){return m_CurrentWaypointIdx >= m_Path.Size();}

size_t Robot::GetId() const { return m_Id; }

Pose Robot::GetPose() const { return m_Pose; }

Trajectory<Pose>& Robot::GetTrajectory() { return m_Path; }

const Trajectory<Pose>& Robot::GetTrajectory() const { return m_Path; }

size_t Robot::GetCurrentWaypointIdx() const { return m_CurrentWaypointIdx; }

RobotMotionState Robot::GetRobotMotionState() const { return m_MotionState; }

RobotMissionState Robot::GetRobotMissionState() const { return m_MissionState;}

const Kinematics<Robot>* Robot::GetKinematics() const { return m_Kinematics.get(); }

std::shared_mutex& Robot::GetMutex() const { return m_Mtx; }

void Robot::SetPose(Pose p_) { m_Pose = p_;}

void Robot::SetTrajectory(Trajectory<Pose> path_) {
  m_Path = std::move(path_);
  m_CurrentWaypointIdx = 0;
}

void Robot::SetRobotMotionState(RobotMotionState motionState_) { m_MotionState = motionState_;}

void Robot::SetRobotMissionState(RobotMissionState missionState_) {m_MissionState = missionState_;}

void Robot::IncrementWaypointIdx() { if (m_CurrentWaypointIdx < m_Path.Size()) ++m_CurrentWaypointIdx; }

