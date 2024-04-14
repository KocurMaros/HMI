#pragma once

#include "CKobuki.h"
#include <QObject>

class PositionTracker
	: public QObject
{
	Q_OBJECT;

public:
	explicit PositionTracker(QObject *parent = nullptr);

public slots:
	void on_positionResults_handle(const TKobukiData &robotdata, double correction);

signals:
	void resultsReady(double x, double y, double fi);

private:
	void calculateOdometry(const TKobukiData &robotdata, double correction);

private:
	double m_x;
	double m_y;
	double m_fi;
	double m_fiCorrection;
	bool m_robotStartupLocation;
	int m_lastLeftEncoder;
	int m_lastRightEncoder;
};

