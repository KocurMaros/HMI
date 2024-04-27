#pragma once

#include <string>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"


#include <QObject>
class ObjectDetection : public QObject{
Q_OBJECT
public:
    ObjectDetection();
    ~ObjectDetection();

public slots:
    void detectObjects(cv::Mat frame);
    void getCircleParam();

private:
    cv::Point m_center_of_object;
    int m_radius_circle;
    bool m_savedImg;
    uint8_t m_circleCounter;
};
