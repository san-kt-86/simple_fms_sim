#include "Display.h"


Display::Display(int rows_, int columns_) : m_ROWS(rows_), m_COLUMNS(columns_) {
    m_Map = cv::Mat(m_ROWS, m_COLUMNS, CV_8UC3, cv::Scalar(0, 0, 0));
    m_MapDisplay = m_Map.clone();
}

void Display::PlotFleet(RobotFleet* robotFleet_) {
    if (!robotFleet_) return;

    m_MapDisplay = m_Map.clone(); // fresh frame from fixed base map each time
    std::shared_lock<std::shared_mutex> lock(robotFleet_->GetMutex());
    auto ids = robotFleet_->GetIDs();
    for (size_t id : ids) {
        Robot* r = robotFleet_->GetRobot(id);
        std::shared_lock<std::shared_mutex> lock(r->GetMutex());
        // PrintData(r);
        PlotRobot(r);
    }
}

int Display::Show(int waitTime_) {
    cv::imshow(m_Window, m_MapDisplay);
    return cv::waitKey(waitTime_);
}

void Display::SetLinesSetting(){
    m_LinesVisible = !m_LinesVisible;
}

Display::~Display(){
    cv::destroyWindow(m_Window);
}

int Display::GenRandomInt(){
    static std::mt19937 gen(std::random_device{}());
    std::uniform_int_distribution<> dist(50, 255);
    return dist(gen);
}

cv::Scalar Display::GenRandomColor(){
	return cv::Scalar(GenRandomInt(), GenRandomInt(), GenRandomInt());
}

cv::Scalar Display::GetColor(Robot* r_){
    cv::Scalar color;
    switch (r_->GetRobotMissionState()) {
        case RobotMissionState::ACTIVE:
            if(m_ColorMap.count(r_->GetId()) > 0)
                color = m_ColorMap[r_->GetId()];
            else
                color = GenRandomColor();
                m_ColorMap[r_->GetId()] = color;
            break;
        
        case RobotMissionState::BLOCKED:
            color = cv::Scalar(0, 0, 255);
            break;

        case RobotMissionState::FINISHED:
            color = cv::Scalar(255, 0, 0);
            break;

        default:
            color = cv::Scalar(200, 200, 200);
            break;
    }

    return color;
}

void Display::PlotRobot(Robot* r_){
    if (!r_) return;

    Pose pose = r_->GetPose();
    int x = static_cast<int>(pose.GetX());
    int y = static_cast<int>(pose.GetY());
    if (x < 0 || x >= m_COLUMNS || y < 0 || y >= m_ROWS) return;
    cv::Scalar color = GetColor(r_);

    const Trajectory<Pose>& traj = r_->GetTrajectory();
    size_t startIdx = r_->GetCurrentWaypointIdx();
    cv::Point prevPoint(x, y);
    for (size_t i = startIdx; i < traj.Size() && m_LinesVisible; ++i) {
        const Pose& wp = traj[i];
        int wx = static_cast<int>(wp.GetX());
        int wy = static_cast<int>(wp.GetY());
        if (wx < 0 || wx >= m_COLUMNS || wy < 0 || wy >= m_ROWS) continue;
        cv::Point wpPoint(wx, wy);
        cv::line(m_MapDisplay, prevPoint, wpPoint, color, LINE_WIDTH);
        prevPoint = wpPoint;
    }

    cv::circle(m_MapDisplay, cv::Point(x, y), SMALL_CIRCLE_RADIUS, color, -1);
    cv::putText(
        m_MapDisplay,
        std::to_string(r_->GetId()),
        cv::Point(x + 8, y - 8),          // Offset from robot circle
        cv::FONT_HERSHEY_SIMPLEX,
        0.3,                              // Font scale
        cv::Scalar(255, 255, 255),        // White text
        1,                                // Thickness
        cv::LINE_AA
    );
    if(traj.Size() > 0)
        cv::circle(m_MapDisplay, cv::Point(prevPoint.x, prevPoint.y), CIRCLE_RADIUS, color, -1);
}

void Display::PrintData(Robot* r){
    std::ostringstream os;
    os<<"ID : "<<r->GetId()<<", Pose : "<<r->GetPose()<<", Mission : "<<(int)r->GetRobotMissionState()<<", Motion : "<<(int)r->GetRobotMotionState()<<std::endl;
    Print(os.str());
}
