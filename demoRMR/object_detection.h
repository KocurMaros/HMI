#ifndef OBJECT_DETECIION_H
#define OBJECT_DETECTION_H

#include <string>

#include <QObject>
class ObjectDetection
: public QWidget
{
	Q_OBJECT;
public:
    ObjectDetection(std::string url);
    ~ObjectDetection();

    void detectObjects(float confThreshold, float nmsThreshold);
private:
    std::string m_url;
};

#endif // OBJECT_DETECTION_H