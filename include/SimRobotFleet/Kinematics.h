#pragma once
#include <iostream>
#include <string>
#include "Helpers.h"
#include "Pose.h"


template <typename T>
class Kinematics{
	const std::string m_KinematicModelName;
	const RobotKinematicModel m_KinematicModel;

public:
	Kinematics(RobotKinematicModel kinematicModel_, std::string kinematicModelName_) : m_KinematicModel(kinematicModel_), m_KinematicModelName(kinematicModelName_) {}

	virtual std::string GetKinematicModelName() const final {return m_KinematicModelName;}
	virtual RobotKinematicModel GetKinematicModel() const final {return m_KinematicModel;}
	virtual void ForwardProgress(T* t_, double dt_) = 0;
	virtual Pose PredictForwardProgress(T* t_, double dt_) const = 0;

	virtual ~Kinematics() = default;

};