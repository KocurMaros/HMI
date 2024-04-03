#include "BodyProgressBars.h"

#include <QDebug>
#include <QGridLayout>

#include <algorithm>
#include <cmath>

#define TO_DEGREES(rad) int(rad * 180 / M_PI)

BodyProgressBars::BodyProgressBars(QWidget *parent)
	: QWidget(parent)
{
	m_angleLabel = new QLabel("Angle (deg)", this);
	m_velocityLabel = new QLabel("Velocity (mm/s)", this);

	m_angleProgressBar = new QProgressBar(this);
	m_angleProgressBar->setOrientation(Qt::Horizontal);
	m_angleProgressBar->setRange(-250, 250);
	m_angleProgressBar->setValue(0);
	m_angleProgressBar->setTextVisible(false);

	m_velocityProgressBar = new QProgressBar(this);
	m_velocityProgressBar->setOrientation(Qt::Horizontal);
	m_velocityProgressBar->setRange(TO_DEGREES(-M_PI / 4.), TO_DEGREES(M_PI / 4.));
	m_velocityProgressBar->setValue(0);
	m_velocityProgressBar->setTextVisible(false);

	QGridLayout *layout = new QGridLayout(this);
	layout->addWidget(m_angleLabel, 0, 0);
	layout->addWidget(m_velocityLabel, 1, 0);
	layout->addWidget(m_angleProgressBar, 0, 1);
	layout->addWidget(m_velocityProgressBar, 1, 1);
	setLayout(layout);
}

void BodyProgressBars::setVelocity(int velocity)
{
	velocity = std::clamp(velocity, m_velocityProgressBar->minimum() + 1, m_velocityProgressBar->maximum() - 1);
	qDebug() << "Setting velocity to " << velocity;
	m_velocityProgressBar->setValue(velocity);
}

void BodyProgressBars::setRotation(double velocity)
{
	velocity = std::clamp(TO_DEGREES(velocity), m_angleProgressBar->minimum() + 1, m_angleProgressBar->maximum() - 1);
	qDebug() << "Setting rotation to " << velocity;
	m_angleProgressBar->setValue(velocity);
}

void BodyProgressBars::setValues(int velocity, double rotation)
{
	setVelocity(velocity);
	setRotation(rotation);
}
