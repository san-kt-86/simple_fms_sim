#include <iostream>
#include <thread>
#include <mutex>
#include <queue>
#include <unordered_map>
#include <ostream>
#include <condition_variable>
#include "RobotBuilder.h"
#include "FleetManager.h"
#include "Display.h"


template<int N>
class Simulator{
	std::shared_ptr<RobotFleet> m_Fleet;
	std::array<std::thread, N> m_Workers;
	std::condition_variable m_CVWorkers, m_CVTaskAllocator;
	std::unordered_map<size_t, std::atomic<int>> m_Blocked;
	std::mutex m_BlockedMutex;
	std::mutex m_Mutex;
	std::queue<std::function<void(void)>> m_Tasks;
	std::atomic<int> m_FinishedTasks = 0;
	int m_NumTasks = 0;
	std::atomic<bool> m_Stop = false;

	const double m_TickTime = 0.2;
	const int RUN_FREQUENCY = 10;
	const double COLLISION_THRESHOLD = 10;
	const double COLLISION_CHECK_TIME = 1;
	const int MAX_BLOCK_COUNT = 20;

public:
	Simulator(std::shared_ptr<RobotFleet> fleet_) : m_Fleet(fleet_) {
		for(int i = 0;i < N;i++)
			m_Workers[i] = std::thread([this](){this->Tick();});
	}

	void Tick(){
		while(true){
			std::unique_lock<std::mutex> lock(m_Mutex);
			m_CVWorkers.wait(lock, [this] () {	return m_Stop || !this->m_Tasks.empty(); });

			if(m_Stop && m_Tasks.empty())
				break;

			auto task = m_Tasks.front();
			m_Tasks.pop();
			lock.unlock();

			task();				// check inside task if robot is nullptr or not, because between creating tasks and executing it could've been deleted, if so then just return
			m_FinishedTasks++;
			m_CVTaskAllocator.notify_one();
		}
	}

	bool CollisionCheck(Robot* r1_, Robot* r2_){
		bool ret = false;
		const Kinematics<Robot>* motionModel1 = r1_->GetKinematics();
		const Kinematics<Robot>* motionModel2 = r2_->GetKinematics();

		for(double dt = 0; dt < COLLISION_CHECK_TIME;dt+=m_TickTime){
			if(Pose::Distance(motionModel1->PredictForwardProgress(r1_, dt), motionModel2->PredictForwardProgress(r2_, dt)) <= COLLISION_THRESHOLD){
				ret = true;
				break;
			}
		}
		return ret;
	}

	void AddBlock(Robot* r){
		size_t id = r->GetId();
		if(m_Blocked.count(id) > 0){
			m_Blocked[id]++;
		}
		else{
			m_Blocked[r->GetId()].store(1);
		}
	}

	void RemoveBlock(Robot* r){
		m_Blocked.erase(r->GetId());
	}

	void RobotStateAction(size_t id_){
		{
			std::shared_lock<std::shared_mutex> fleetLock(m_Fleet->GetMutex());
			Robot* robot = m_Fleet->GetRobot(id_);

			std::unique_lock<std::shared_mutex> robotLock(robot->GetMutex());
			std::unique_lock<std::mutex> blockLock(m_BlockedMutex);

			if(m_Blocked.count(robot->GetId()) > 0){
				robot->SetRobotMissionState(RobotMissionState::BLOCKED);
				if(m_Blocked[robot->GetId()] > MAX_BLOCK_COUNT){
					robot->SetRobotMissionState(RobotMissionState::FINISHED);
					robot->SetRobotMotionState(RobotMotionState::STOP);
					RemoveBlock(robot);
				}
				else
					AddBlock(robot);
			}
			else{
				if(robot->IsTrajectoryFinished()){
					robot->SetRobotMissionState(RobotMissionState::FINISHED);
					robot->SetRobotMotionState(RobotMotionState::STOP);
					if(m_Blocked.count(robot->GetId()) > 0) RemoveBlock(robot);
				}
				else{
					robot->SetRobotMissionState(RobotMissionState::ACTIVE);
					robot->SetRobotMotionState(RobotMotionState::RUN);
					robot->Progress(m_TickTime);
					if(m_Blocked.count(robot->GetId()) > 0) RemoveBlock(robot);
				}
			}
			robotLock.unlock();
		}

	}

	void RobotTick(size_t id_){
		{
			std::shared_lock<std::shared_mutex> fleetLock(m_Fleet->GetMutex());
			Robot* robot = m_Fleet->GetRobot(id_);
			auto ids = m_Fleet->GetIDs();
			for(auto id : ids){
				Robot* otherRobot = m_Fleet->GetRobot(id);

				{
					std::shared_lock<std::shared_mutex> lockRobot(robot->GetMutex());
					std::shared_lock<std::shared_mutex> lockOtherRobot(otherRobot->GetMutex());
					bool collisionStatus = CollisionCheck(robot, otherRobot);
					
					if(collisionStatus){
						if(robot->GetId() != otherRobot->GetId()){
							std::unique_lock<std::mutex> blockLock(m_BlockedMutex);
							AddBlock(robot);
							break;
						}
					}
					else
						RemoveBlock(robot);
				}
			}
		}

		RobotStateAction(id_);
		// [ppp]
	}

	void Run(){
		while(!m_Stop){
			{
				std::unique_lock<std::mutex> lock(m_Mutex);	
				m_CVTaskAllocator.wait(lock, [this](){return m_FinishedTasks == m_NumTasks || m_Stop; });

				if(m_Stop) break;

				{
					std::shared_lock<std::shared_mutex> fleetLock(m_Fleet->GetMutex());
					std::vector<size_t> robotIDs = m_Fleet->GetIDs();
					fleetLock.unlock();


					for(auto id : robotIDs)
						m_Tasks.push([this, id](){this->RobotTick(id);});

					m_NumTasks = m_Tasks.size();
					m_FinishedTasks = 0;
				}
			}

			m_CVWorkers.notify_all();
			int waitTime_ms = 1000/RUN_FREQUENCY;
		    std::this_thread::sleep_for(std::chrono::milliseconds(waitTime_ms));
		}
	}

	void Stop(){
		m_Stop.store(true);
		m_CVWorkers.notify_all();
		m_CVTaskAllocator.notify_all();
	}

	~Simulator(){
		for(int i = 0;i < N;i++)
			m_Workers[i].join();
	}
};