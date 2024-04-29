#include "PositionTracker.h"
#include <QDebug>

#define SHORT_MAX 65535
#define TO_RADIANS 0.017453292519943295769236907684886
#define TICK_TO_METER 0.000085292090497737556558

PositionTracker::PositionTracker(QObject *parent)
	: QObject(parent)
	, m_lastLeftEncoder(0)
	, m_lastRightEncoder(0)
	, m_x(0)
	, m_y(0)
	, m_fi(0)
{
}

void PositionTracker::calculateOdometry(const TKobukiData &robotdata, double correction)
{
	int diffLeftEnc = robotdata.EncoderLeft - m_lastLeftEncoder;
	int diffRightEnc = robotdata.EncoderRight - m_lastRightEncoder;

	if (m_lastRightEncoder > 60'000 && robotdata.EncoderRight < 1'000)
		diffRightEnc += SHORT_MAX;
	if (m_lastLeftEncoder > 60'000 && robotdata.EncoderLeft < 1'000)
		diffLeftEnc += SHORT_MAX;

	if (m_lastRightEncoder < 1'000 && robotdata.EncoderRight > 60'000)
		diffRightEnc -= SHORT_MAX;
	if (m_lastLeftEncoder < 1'000 && robotdata.EncoderLeft > 60'000)
		diffLeftEnc -= SHORT_MAX;

	auto leftEncDist = TICK_TO_METER * diffLeftEnc;
	auto rightEncDist = TICK_TO_METER * diffRightEnc;

	m_lastLeftEncoder = robotdata.EncoderLeft;
	m_lastRightEncoder = robotdata.EncoderRight;

	double l = (rightEncDist + leftEncDist) / 2.0;
	m_fi = robotdata.GyroAngle / 100. * TO_RADIANS - correction;
	m_x = m_x + l * std::cos(m_fi);
	m_y = m_y + l * std::sin(m_fi);

	emit resultsReady(m_x, m_y, m_fi);
}

void PositionTracker::on_positionResults_handle(const TKobukiData &robotdata, double correction)
{
	calculateOdometry(robotdata, correction);
}
