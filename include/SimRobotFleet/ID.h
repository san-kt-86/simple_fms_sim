#pragma once
#include <iostream>
#include <atomic>


class ID{
private:
	inline static std::atomic<size_t> m_IdCounter = 0;

public:
	ID() = delete;

	static size_t GenId() {return m_IdCounter++;}
};