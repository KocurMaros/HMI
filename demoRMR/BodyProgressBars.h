#pragma once

#include <QWidget>
#include <QProgressBar>

class BodyProgressBars
	: public QWidget
{
	Q_OBJECT;
public:
	explicit BodyProgressBars(QWidget *parent = nullptr);
	void setVelocity(int velocity);
	void setRotation(double velocity);
	void setValues(int velocity, double rotation);
signals:

public slots:
private:
	QProgressBar *m_angleProgressBar;
	QProgressBar *m_velocityProgressBar;
};
