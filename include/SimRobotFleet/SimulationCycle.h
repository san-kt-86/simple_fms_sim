#pragma once
#include <iostream>




class SimulationCycle{
public:
	SimulationCycle() = default;
	virtual void Tick(double dt_) = 0;

	virtual ~SimulationCycle() = default;
};