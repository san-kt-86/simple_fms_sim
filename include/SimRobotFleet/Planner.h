#pragma once
#include <iostream>
#include "Trajectory.h"
#include "Pose.h"



class Planner{
public:
	virtual bool GenTrajectory(Pose start, Pose goal, Trajectory<Pose>& traj_) = 0;
	virtual ~Planner() = default;
};