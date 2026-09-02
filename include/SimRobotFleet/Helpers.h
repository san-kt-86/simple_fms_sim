#pragma once
#include <iostream>




enum class SimulationState {RUN, STOP};

enum class RobotKinematicModel {BICYCLE, DIFFERENTIAL, SIMPLESIM};

enum class RobotMissionState {ACTIVE, BLOCKED, FINISHED};

enum class RobotMotionState {RUN, STOP};


inline void Print(std::string msg){
	static std::mutex mtx;
	std::lock_guard<std::mutex> lock(mtx);
	std::cout<<msg;
}