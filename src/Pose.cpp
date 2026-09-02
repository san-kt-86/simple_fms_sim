#include "Pose.h"
#include <cmath>



Pose::Pose() : m_X(-1), m_Y(-1), m_Theta(0) {}
Pose::Pose(double x_, double y_, double theta_) : m_X(x_), m_Y(y_), m_Theta(theta_) {}

double Pose::GetX() const {return m_X;}

double Pose::GetY() const {return m_Y;}

double Pose::GetTheta() const {return m_Theta;}

std::array<double,3> Pose::GetPose() const {return {m_X, m_Y, m_Theta};}

void Pose::SetX(double x_){m_X = x_;}

void Pose::SetY(double y_){m_Y = y_;}

void Pose::SetTheta(double t_){m_Theta = t_;}

void Pose::SetPose(double x_, double y_, double theta_){m_X=x_;m_Y=y_;m_Theta=theta_;}

double Pose::Distance(Pose p1_, Pose p2_){return sqrt(std::pow(p1_.GetX()-p2_.GetX(), 2) + std::pow(p1_.GetY()-p2_.GetY(), 2));}

std::ostream& operator<<(std::ostream& os, const Pose& p){
	os<<"Pose {X, Y, Theta} : "<<p.GetX()<<','<<p.GetY()<<','<<p.GetTheta();
	return os;
}
