#pragma once
#include <iostream>
#include <array>





class Pose{
	double m_X, m_Y, m_Theta;

public:
	Pose();
	Pose(double x_, double y_, double theta_);

	double GetX() const ;
	double GetY() const ;
	double GetTheta() const ;
	std::array<double,3> GetPose() const ;
	void SetX(double x_);
	void SetY(double y_);
	void SetTheta(double t_);
	void SetPose(double x_, double y_, double theta_);

	static double Distance(Pose p1_, Pose p2_);
};

std::ostream& operator<<(std::ostream& os, const Pose& p);
