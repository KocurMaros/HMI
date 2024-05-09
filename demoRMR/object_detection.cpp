#include "object_detection.h"


#include <vector>
#include <iostream>
// #include <thread>


ObjectDetection::ObjectDetection(QObject *parent) : QObject(parent)
    ,m_savedImg(false)
    ,m_circleCounter(0)
{
}

ObjectDetection::~ObjectDetection()
{
    // Destructor implementation
}


void ObjectDetection::detectObjects(cv::Mat frame)
{
    if(frame.empty()) {std::cout << "ERRROR!";  }
    
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::medianBlur(gray, gray, 5);
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1,
    gray.rows, // change this value to detect circles with different distances to each other
    200, 30, 0, 0 // change the last two parameters
    // (min_radius & max_radius) to detect larger circles
    );
    for( size_t i = 0; i < circles.size(); i++ )
    {
        cv::Vec3i c = circles[i];
        cv::Point center = cv::Point(c[0], c[1]);
        // circle center
        // cv::circle( frame, center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA);
        // circle outline
        int radius = c[2];
        cv::circle( frame, center, radius, cv::Scalar(255,0,255), 3, cv::LINE_AA);
        if(!m_savedImg && circles.size() != m_circleCounter && circles.size() > 0){
            std::cout << "Circle detected!" << " " << c[0] << " " << c[1] << std::endl;
            cv::imwrite("circles.jpg", frame);
            emit on_circleDetected(center);
            m_savedImg = true;
        }
    }
}
