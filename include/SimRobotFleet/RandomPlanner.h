#pragma once
#include <iostream>
#include <random>
#include "Robot.h"
#include "Trajectory.h"
#include "Planner.h"


class RandomPlanner : public Planner{
	const int RAND_DIRECTION_MAX = 100;
	const int GOAL_THRESHOLD = 5;
	const int MAX_ITERATIONS = 100;

	enum class Direction {HORIZONTAL, VERTICAL};
	int GenRandomInt(int leftEnd_, int rightEnd_);
	int GenIntermediateNumber(int num1_, int num2_);

public:
	RandomPlanner() = default;
	bool GenTrajectory(Pose start_, Pose goal_, Trajectory<Pose>& traj_) override;
};