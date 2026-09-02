#pragma once
#include <opencv2/opencv.hpp>
#include <shared_mutex>
#include <unordered_map>
#include <random>
#include "Robot.h"
#include "RobotFleet.h"
#include "Helpers.h"



class Display {
public:
    Display(int rows_, int columns_);
    void PlotFleet(RobotFleet* robotFleet_) ;
    int Show(int waitTime_) ;
    void SetLinesSetting();
    friend void PlotRobot(Display&, Robot*);
    ~Display();

private:
    cv::Mat m_Map, m_MapDisplay;
    std::unordered_map<size_t, cv::Scalar> m_ColorMap;
    bool m_Stop = false;

    const int m_ROWS, m_COLUMNS;
    const int LINE_WIDTH = 2;
    const int CIRCLE_RADIUS = 7;
    const int SMALL_CIRCLE_RADIUS = 5;
    const int WAYPOINT_RADIUS = 3;
    const std::string m_Window = "Simulation";

    bool m_LinesVisible = true;
	int GenRandomInt();
	cv::Scalar GenRandomColor();
    cv::Scalar GetColor(Robot* r_);
    void PlotRobot(Robot* r_);
    void PrintData(Robot* r);
};