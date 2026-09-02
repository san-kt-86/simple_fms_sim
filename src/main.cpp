#include <iostream>
#include <thread>
#include "Planner.h"
#include "RobotBuilder.h"
#include "KinematicsSimpleSim.h"
#include "Helpers.h"
#include "Display.h"
#include "FleetManager.h"
#include "RandomPlanner.h"
#include "Simulator.h"


const int simulatorWorkerThreads = 2;
const int numRobots = 20;
const int numWorkerThreads = 2;
// Total threads
// worker threads are for simulator to simulate the robots and an additional one for main loop of simulator
// one for fms
// main loop, also runs display class

const int speedLowerLimit = 5;
const int speedUpperLimit = 10;

const int xLowerLimit = 50;
const int xUpperLimit = 750;
const int yLowerLimit = 50;
const int yUpperLimit = 550;

const uint mapRows = 600;
const uint mapCols = 800;

const int dim = 10;


int GenRandomInt(int leftEnd_, int rightEnd_){
    static std::mt19937 gen(std::random_device{}());
    std::uniform_int_distribution<> dist(leftEnd_, rightEnd_);
    int ret = dist(gen);
    return ret;
}

Pose GenRandomPose(){
	return Pose(GenRandomInt(xLowerLimit, xUpperLimit), GenRandomInt(yLowerLimit, yUpperLimit), 0);
}

void PlotRobot(Display& d, Robot* r){
    d.PlotRobot(r);
}

int main(){
	std::shared_ptr<RobotFleet> fleet(new RobotFleet());
	for(int i = 0;i < numRobots;i++){
		Pose start = GenRandomPose();

		RobotBuilder rb;
		double speed = (double)GenRandomInt(speedLowerLimit, speedUpperLimit);
		std::unique_ptr<Robot> robot =  rb.SetPose(start)
										  .SetKinematicModel(RobotKinematicModel::SIMPLESIM)
										  .SetDimension(dim)
										  .SetSpeed(speed)
										  .SetMotionState(RobotMotionState::RUN)
										  .SetMissionState(RobotMissionState::FINISHED)
										  .Build();

		{
			std::unique_lock<std::shared_mutex> lock(fleet->GetMutex());
			fleet->AddRobot(std::move(robot));
		}
	}

	std::shared_ptr<FleetManager> fms(new FleetManager(std::make_unique<RandomPlanner>(), fleet));
	std::thread tFMS(std::bind(&FleetManager::HandleFleet, fms));

	std::shared_ptr<Simulator<numWorkerThreads>> simulator(new Simulator<numWorkerThreads>(fleet));
	std::thread tSimulator(std::bind(&Simulator<numWorkerThreads>::Run, simulator));

	Display disp(mapRows, mapCols);
	while(true){
		disp.PlotFleet(fleet.get());
		uint key = disp.Show(1);
		bool done = false;
		switch(key){
			case 27:
				fms->Stop();
				simulator->Stop();
				done = true;
				break;

			case 32:
				disp.SetLinesSetting();
				break;

			default:
				break;

		}

		if(done)
			break;
	}

	tFMS.join();
	tSimulator.join();
}
