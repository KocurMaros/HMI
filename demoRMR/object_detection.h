#pragma once
#include <string>

#include <QObject>
#include <QWidget>


class ObjectDetection
: public QWidget
{
	Q_OBJECT;
public:
    explicit ObjectDetection(QWidget *parent = nullptr);
    ~ObjectDetection();

    void detectObjects(float confThreshold, float nmsThreshold);
signals:

public slots:
private:
    std::string m_url;
};
