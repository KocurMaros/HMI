#include "BodyProgressBars.h"
#include <QBoxLayout>

BodyProgressBars::BodyProgressBars(QWidget *parent)
	: QWidget(parent)
{
	m_angleProgressBar = new QProgressBar(this);
	m_angleProgressBar->setOrientation(Qt::Horizontal);
	m_angleProgressBar->setRange(-300, 300);
	m_angleProgressBar->setValue(0);

	m_velocityProgressBar = new QProgressBar(this);
	m_velocityProgressBar->setOrientation(Qt::Horizontal);
	m_velocityProgressBar->setRange(-360, 360);
	m_velocityProgressBar->setValue(0);

	QVBoxLayout *layout = new QVBoxLayout(this);
	layout->addWidget(m_angleProgressBar);
	layout->addWidget(m_velocityProgressBar);
	setLayout(layout);
}
