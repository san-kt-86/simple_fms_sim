#pragma once
#include <iostream>
#include <chrono>
#include <random>
#include <thread>
#include "Planner.h"
#include "RobotFleet.h"





class FleetManager{
	std::unique_ptr<Planner> m_Planner;
	std::shared_ptr<RobotFleet> m_Fleet;
	std::atomic<bool> m_Stop = false;
	const int GOAL_FAR_THRESHOLD = 100;
	const int RUN_FREQUENCY = 10;
	const int NEARBY_RANGE = 15;
	int m_XLeftLimit, m_XRightLimit, m_YLeftLimit, m_YRightLimit;
    std::mutex mtx;

    enum class FMSAction{NONE, HALT, GEN_PATH};
	int GenRandomInt(int leftEnd_, int rightEnd_);
	Pose GenRandomPose();
	Pose GenRandomPoseNearby(Pose p);

public:
	FleetManager(std::unique_ptr<Planner> planner_, std::shared_ptr<RobotFleet> fleet_);
	bool GenRobotTrajectory(Robot* robot_);
	void FMSActionRefresh(Robot* robot_, FMSAction action_);
	void HandleFleet();
	void Stop();
	void SetXLimits(int xLeft_, int xRight_);
	void SetYLimits(int yLeft_, int yRight_);
};


