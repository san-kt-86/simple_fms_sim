#pragma once
#include <iostream>
#include <cmath>
#include "Kinematics.h"




template <typename T>
class KinematicsBicycle : public Kinematics<T> {
	const double m_WheelBase;
	double m_Delta, m_Speed;

public:
	KinematicsBicycle(double wheelBase_, double delta_, double speed_) : Kinematics<T>(RobotKinematicModel::BICYCLE, "Bicycle"), m_WheelBase(wheelBase_), m_Delta(delta_), m_Speed(speed_) {}

	void ForwardProgress(T* t_, double dt_) override {
		double theta = (m_Speed/m_WheelBase)*std::tan(m_Delta)*dt_ + t_->GetPose().GetTheta();
		double x = (m_WheelBase/std::tan(m_Delta)) * (std::sin(theta) - std::sin(t_->GetPose().GetTheta())) + t_->GetPose().GetX();
		double y = (m_WheelBase/std::tan(m_Delta)) * (std::cos(t_->GetPose().GetTheta()) - std::cos(theta)) + t_->GetPose().GetY();

		t_->SetPose(Pose(x,y,theta));
	}

    Pose PredictForwardProgress(T* t_, double dt_) const override {
    	return Pose(0,0,0);
    }

	void SetDelta(double delta_) {m_Delta = delta_;}
	void SetSpeed(double speed_) {m_Speed = speed_;}
	double GetDelta() const {return m_Delta;}
	double GetSpeed() const {return m_Speed;}
	double GetWheelBase() const {return m_WheelBase;}
};