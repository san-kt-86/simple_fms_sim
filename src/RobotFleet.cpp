#include "RobotFleet.h"




void RobotFleet::AddRobot(std::unique_ptr<Robot> robot_){
	size_t id = robot_->GetId();
	m_Fleet[id] = std::move(robot_);

	m_IDs.push_back(id);
}

void RobotFleet::RemoveRobot(size_t id_){
	m_Fleet.erase(id_);

	auto it = std::find(m_IDs.begin(), m_IDs.end(), id_);
	if (it != m_IDs.end())
	    m_IDs.erase(it);		
}

Robot* RobotFleet::GetRobot(size_t id) {
    auto it = m_Fleet.find(id);
    return (it != m_Fleet.end()) ? it->second.get() : nullptr;
}

RobotFleet::~RobotFleet(){
	m_Fleet.clear();
	m_IDs.clear();
}

std::shared_mutex& RobotFleet::GetMutex() const {return m_Mtx;}
std::vector<size_t> RobotFleet::GetIDs() const {return m_IDs;}
size_t RobotFleet::Size() const {return m_IDs.size();}

