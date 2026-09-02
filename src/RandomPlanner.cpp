#include "RandomPlanner.h"



int RandomPlanner::GenRandomInt(int leftEnd_, int rightEnd_){
    static std::mt19937 gen(std::random_device{}());
    static std::mutex mtx;

    std::lock_guard<std::mutex> lock(mtx);
    std::uniform_int_distribution<> dist(leftEnd_, rightEnd_);
    return dist(gen);
}

int RandomPlanner::GenIntermediateNumber(int num1_, int num2_){
    int leftEnd = std::min(num1_, num2_);
    int rightEnd = std::max(num1_, num2_);

    int num = GenRandomInt(leftEnd, rightEnd);
    if(num == num1_ && num != num2_)
    	return GenIntermediateNumber(num1_, num2_);
    else
    	return num;
}

bool RandomPlanner::GenTrajectory(Pose start_, Pose goal_, Trajectory<Pose>& traj_) {
	int startX = start_.GetX();
	int startY = start_.GetY();

	int goalX = goal_.GetX();
	int goalY = goal_.GetY();

	traj_.Clear();
	traj_.Add(start_);
	int itr = 0;
	while(itr++ < MAX_ITERATIONS){
		int num = 0;
		Direction dir = GenRandomInt(0,RAND_DIRECTION_MAX) <= RAND_DIRECTION_MAX/2.0 ? Direction::HORIZONTAL : Direction::VERTICAL;

		if(dir == Direction::HORIZONTAL){
			num = GenIntermediateNumber(startX, goalX);
			startX = std::abs(num - goalX) < GOAL_THRESHOLD ? goalX : num;
		}
		else{
			num = GenIntermediateNumber(startY, goalY);
			startY = std::abs(num - goalY) < GOAL_THRESHOLD ? goalY : num;
		}

		traj_.Add(Pose(startX, startY,0));
		if(startX == goalX && startY == goalY)
			return true;
	}

	return false;
}