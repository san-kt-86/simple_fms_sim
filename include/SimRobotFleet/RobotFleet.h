#pragma once
#include <iostream>
#include <unordered_map>
#include <memory>
#include <atomic>
#include <algorithm>
#include <shared_mutex>
#include <vector>
#include "Robot.h"


class RobotFleet{
	std::unordered_map<size_t, std::unique_ptr<Robot>> m_Fleet;
	std::vector<size_t> m_IDs;
	std::atomic<size_t> m_Count = 0;
	mutable std::shared_mutex m_Mtx;

public:
	RobotFleet() = default;
	RobotFleet(const RobotFleet&) = delete;
	RobotFleet(RobotFleet&& fleet_) = delete;
	RobotFleet& operator=(const RobotFleet&) = delete;
	RobotFleet& operator=(RobotFleet&&) = delete;

	void AddRobot(std::unique_ptr<Robot> robot_);
	void RemoveRobot(size_t id_);

	Robot* GetRobot(size_t id);

	~RobotFleet();

	std::shared_mutex& GetMutex() const;
	std::vector<size_t> GetIDs() const;
	size_t Size() const;

};