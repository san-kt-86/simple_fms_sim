#include "FleetManager.h"





int FleetManager::GenRandomInt(int leftEnd_, int rightEnd_){
    static std::mt19937 gen(std::random_device{}());
	std::lock_guard<std::mutex> lock(mtx);
    std::uniform_int_distribution<> dist(leftEnd_, rightEnd_);
    int randInt = dist(gen);
    return randInt;
}

Pose FleetManager::GenRandomPose(){
	return Pose(GenRandomInt(m_XLeftLimit, m_XRightLimit), GenRandomInt(m_YLeftLimit, m_YRightLimit), 0);
}

Pose FleetManager::GenRandomPoseNearby(Pose p){
	return Pose(GenRandomInt(p.GetX() - NEARBY_RANGE, p.GetX() + NEARBY_RANGE), GenRandomInt(p.GetY() - NEARBY_RANGE, p.GetY() + NEARBY_RANGE), 0);
}

FleetManager::FleetManager(std::unique_ptr<Planner> planner_, std::shared_ptr<RobotFleet> fleet_) :	m_Planner(std::move(planner_)),
																						m_Fleet(fleet_),
																						m_XLeftLimit(50),
																						m_XRightLimit(750),
																						m_YLeftLimit(50),
																						m_YRightLimit(550) {}

bool FleetManager::GenRobotTrajectory(Robot* robot_){
	const Trajectory<Pose>& traj = robot_->GetTrajectory();
	Trajectory<Pose> trajNew;
	bool trajNewSuccess;
	switch(robot_->GetRobotMissionState()){
		case RobotMissionState::ACTIVE:
			break;

		case RobotMissionState::BLOCKED:
			{
				Pose goalMid = GenRandomPoseNearby(robot_->GetPose());
				Pose goal = traj.Size() > 0 ? traj[traj.Size()-1] : GenRandomPose();

				Trajectory<Pose> trajToMid, trajFromMid;

				bool trajToMidSuccess = m_Planner->GenTrajectory(robot_->GetPose(), goalMid, trajToMid);
				bool trajFromMidSuccess = m_Planner->GenTrajectory(goalMid, goal, trajFromMid);

				trajNewSuccess = trajToMidSuccess && trajFromMidSuccess;
				if(trajNewSuccess){
					for(int i = 0;i < trajToMid.Size();i++)
						trajNew.Add(trajToMid[i]);

					for(int i = 0;i < trajFromMid.Size();i++)
						trajNew.Add(trajFromMid[i]);
				}
			}
			break;

		case RobotMissionState::FINISHED:
			Pose goal = GenRandomPose();
			trajNewSuccess = m_Planner->GenTrajectory(robot_->GetPose(), goal, trajNew);
			break;
	}

	if(trajNewSuccess)
		robot_->SetTrajectory(trajNew);

	return trajNewSuccess;
}

void FleetManager::FMSActionRefresh(Robot* robot_, FMSAction action_){
	switch(action_){
		case FMSAction::NONE:
			break;

		case FMSAction::HALT:
			{
				std::unique_lock<std::shared_mutex> uniqueLockRobot(robot_->GetMutex());
				robot_->SetRobotMotionState(RobotMotionState::STOP);						
			}
			break;

		case FMSAction::GEN_PATH:
			{
				std::unique_lock<std::shared_mutex> uniqueLockRobot(robot_->GetMutex());
				bool trajGen = GenRobotTrajectory(robot_);
				if(trajGen){
					robot_->SetRobotMotionState(RobotMotionState::RUN);
					robot_->SetRobotMissionState(RobotMissionState::ACTIVE);
				}
				else
					robot_->SetRobotMotionState(RobotMotionState::STOP);
			}
			break;
	}

}

void FleetManager::HandleFleet(){
	while(!m_Stop){
		{
			std::shared_lock<std::shared_mutex> sharedLockFleet(m_Fleet->GetMutex());
			std::vector<size_t> ids = m_Fleet->GetIDs();

			for(size_t i = 0;i < ids.size();i++){
				{
					Robot* robot = m_Fleet->GetRobot(ids[i]);
					FMSAction action = FMSAction::NONE;
					{
						std::shared_lock<std::shared_mutex> sharedLockRobot(robot->GetMutex());
						RobotMissionState robotMissionState = robot->GetRobotMissionState();
						if(robotMissionState == RobotMissionState::BLOCKED || robotMissionState == RobotMissionState::FINISHED){
							if(robot->GetRobotMotionState() == RobotMotionState::STOP)
								action = FMSAction::GEN_PATH;
							else
								action = FMSAction::HALT;
						}
					}

					FMSActionRefresh(robot, action);
				}
			}
		}


		int waitTime_ms = 1000/RUN_FREQUENCY;
	    std::this_thread::sleep_for(std::chrono::milliseconds(waitTime_ms));
	}
}

void FleetManager::Stop(){
	m_Stop.store(true);
}

void FleetManager::SetXLimits(int xLeft_, int xRight_){m_XLeftLimit = xLeft_; m_XRightLimit = xRight_;}

void FleetManager::SetYLimits(int yLeft_, int yRight_){m_YLeftLimit = yLeft_; m_YRightLimit = yRight_;}

