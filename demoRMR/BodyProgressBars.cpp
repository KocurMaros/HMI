#include "BodyProgressBars.h"
#include <QBoxLayout>
#include <QDebug>

#include <algorithm>
#include <cmath>

#define TO_DEGREES(rad) int(rad * 180/M_PI)

BodyProgressBars::BodyProgressBars(QWidget *parent)
	: QWidget(parent)
{
	m_angleProgressBar = new QProgressBar(this);
	m_angleProgressBar->setOrientation(Qt::Horizontal);
	m_angleProgressBar->setRange(-250, 250);
	m_angleProgressBar->setValue(0);

	m_velocityProgressBar = new QProgressBar(this);
	m_velocityProgressBar->setOrientation(Qt::Horizontal);
	m_velocityProgressBar->setRange(TO_DEGREES(-M_PI/4.), TO_DEGREES(M_PI/4.));
	m_velocityProgressBar->setValue(0);

	QVBoxLayout *layout = new QVBoxLayout(this);
	layout->addWidget(m_angleProgressBar);
	layout->addWidget(m_velocityProgressBar);
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

