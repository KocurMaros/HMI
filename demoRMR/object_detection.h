#include <string>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"



class ObjectDetection{
public:
    ObjectDetection();
    ~ObjectDetection();

    void detectObjects(cv::Mat frame);
private:
    std::string m_url;
    cv::VideoCapture cap;
    bool m_savedImg;
    uint8_t m_circleCounter;
};
