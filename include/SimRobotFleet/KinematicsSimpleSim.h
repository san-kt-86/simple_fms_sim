#pragma once
#include <iostream>
#include <cmath>
#include "Kinematics.h"
#include "Helpers.h"
#include "Pose.h"



template <typename T>
class SimpleSimKinematics : public Kinematics<T> {
    double m_Speed;
public:
    explicit SimpleSimKinematics(double speed_) : Kinematics<T>(RobotKinematicModel::SIMPLESIM, "SimpleSim"), m_Speed(speed_) {}

    Pose CalKinematicMovement (Pose current_, Pose target_, double dt_) const{
        double dx = target_.GetX() - current_.GetX();
        double dy = target_.GetY() - current_.GetY();
        double distance = std::sqrt(dx * dx + dy * dy);

        double step = m_Speed * dt_;

        if (step >= distance) {
            return target_;
        } else {
            double theta = std::atan2(dy, dx);
            return Pose(current_.GetX() + std::cos(theta) * step, current_.GetY() + std::sin(theta) * step, theta);
        }

    }

    Pose PredictForwardProgress(T* t_, double dt_) const override{
        auto& traj = t_->GetTrajectory();
        size_t idx = t_->GetCurrentWaypointIdx();

        Pose current = t_->GetPose();
        if (idx >= traj.Size()) return current; // trajectory complete, nothing to do

        Pose target = traj[idx];
        Pose predPose = CalKinematicMovement(current, target, dt_);

        return predPose;
    }

    void ForwardProgress(T* t_, double dt_) override {
        auto& traj = t_->GetTrajectory();
        size_t idx = t_->GetCurrentWaypointIdx();
        if (idx >= traj.Size()) return;
        Pose target = traj[idx];
        Pose predPose = PredictForwardProgress(t_, dt_);

        t_->SetPose(predPose);
        if(Pose::Distance(predPose, target) < 1)
            t_->IncrementWaypointIdx();
    }

    void SetSpeed(double speed_) { m_Speed = speed_; }
    double GetSpeed() const { return m_Speed; }
};