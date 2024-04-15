#include <string>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"



class ObjectDetection{
public:
    ObjectDetection();
    ~ObjectDetection();

    cv::Point detectObjects(cv::Mat frame);
private:
    bool m_savedImg;
    uint8_t m_circleCounter;
};
