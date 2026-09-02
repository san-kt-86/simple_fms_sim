#pragma once
#include <iostream>
#include <cmath>
#include "Kinematics.h"




template <typename T>
class KinematicsDifferential : public Kinematics<T> {
	const double m_Width;
	double m_Angular, m_Speed;

public:
	KinematicsDifferential(double width_, double angular_, double speed_) : Kinematics<T>(RobotKinematicModel::DIFFERENTIAL, "Differential"), m_Width(width_), m_Angular(angular_), m_Speed(speed_) {}

	void ForwardProgress(T* t_, double dt_) override {
		double theta = m_Angular*dt_ + t_->GetPose().GetTheta();
		double x = (m_Speed/m_Angular) * (std::sin(theta) - std::sin(t_->GetPose().GetTheta())) + t_->GetPose().GetX();
		double y = (m_Speed/m_Angular) * (std::cos(t_->GetPose().GetTheta()) - std::cos(theta)) + t_->GetPose().GetY();

		t_->SetPose(Pose(x,y,theta));
	}


    Pose PredictForwardProgress(T* t_, double dt_) const override{
    	return Pose(0,0,0);
    }

	void SetAngular(double angular_) {m_Angular = angular_;}
	void SetSpeed(double speed_) {m_Speed = speed_;}
	double GetAngular() const {return m_Angular;}
	double GetSpeed() const {return m_Speed;}
	double GetWidth() const {return m_Width;}
};