#pragma once
#include <vector>
#include <cstddef>
#include <stdexcept>


template<typename T>
class Trajectory {
    std::vector<T> m_Traj;
public:
    Trajectory() = default;

    void Add(const T& t_) {
        m_Traj.push_back(t_);
    }

    void Erase(size_t ind_) {
        if (ind_ >= m_Traj.size())
            throw std::out_of_range("Trajectory::erase index out of range");
        m_Traj.erase(m_Traj.begin() + ind_);
    }

    void Clear() {
        m_Traj.clear();
    }

    size_t Size() const {
        return m_Traj.size();
    }

    T& operator[](size_t ind_) {
        if (ind_ >= m_Traj.size())
            throw std::out_of_range("Trajectory::operator[] index out of range");
        return m_Traj[ind_];
    }

    const T& operator[](size_t ind_) const {
        if (ind_ >= m_Traj.size())
            throw std::out_of_range("Trajectory::operator[] index out of range");
        return m_Traj[ind_];
    }

    ~Trajectory() = default;
};