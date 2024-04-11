#include "object_detection.h"


#include <vector>
#include <iostream>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

ObjectDetection::ObjectDetection(std::string url)
{
    // Constructor implementation
    m_url = "http://"+url+":8000/stream.mjpg";
}

ObjectDetection::~ObjectDetection()
{
    // Destructor implementation
}


void ObjectDetection::detectObjects(float confThreshold, float nmsThreshold)
{
    std::cout << "Object detection started!" << std::endl;
    // Object detection implementation
    cv::Mat frame;
    cv::VideoCapture cap(m_url);
    if (cap.isOpened()==false) {std::cout << "ERRROR!";return;}

    while(1)
    {
        cap >> frame;
        if(frame.empty()) {std::cout << "ERRROR!";return;}
        imshow ("Live",frame);
        if(cv::waitKey(5)>0) break;
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::medianBlur(gray, gray, 5);
        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1,
        gray.rows/16, // change this value to detect circles with different distances to each other
        100, 30, 1, 30 // change the last two parameters
        // (min_radius & max_radius) to detect larger circles
        );
        for( size_t i = 0; i < circles.size(); i++ )
        {
            cv::Vec3i c = circles[i];
            cv::Point center = cv::Point(c[0], c[1]);
            // circle center
            cv::circle( frame, center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA);
            // circle outline
            int radius = c[2];
            cv::circle( frame, center, radius, cv::Scalar(255,0,255), 3, cv::LINE_AA);
        }
    }
}