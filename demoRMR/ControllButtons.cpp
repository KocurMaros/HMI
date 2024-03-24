#include "ControllButtons.h"
#include "ui_mainwindow.h"
#include <QWidget>
#include <QObject>
#include <QPushButton>

ControllButtons::ControllButtons(bool *reverse, bool *forward, MainWindow *parent)
	: QWidget(parent)
	, m_parent(parent)
	, m_isLeftHand(false)
	, m_reverseRobot(reverse)
	, m_forwardRobot(forward)
{
	m_parent->ui->actionAdd_motion_buttons->setText("Remove motion buttons");

	m_buttonGridlayout = new QGridLayout(this);

	QSize buttonSize(50, 75);
	QSizePolicy buttonSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

	m_forwardButtons = new QPushButton("Forward", this);
	m_forwardButtons->setMinimumSize(buttonSize);
	m_forwardButtons->setSizePolicy(buttonSizePolicy);
	m_buttonGridlayout->addWidget(m_forwardButtons, 0, 1, 1, 1);
	connect(m_forwardButtons, &QPushButton::clicked, this, &ControllButtons::on_forwardButtons_clicked);

	m_backButtons = new QPushButton("Back", this);
	m_backButtons->setMinimumSize(buttonSize);
	m_backButtons->setSizePolicy(buttonSizePolicy);
	m_buttonGridlayout->addWidget(m_backButtons, 2, 1, 1, 1);
	connect(m_backButtons, &QPushButton::clicked, this, &ControllButtons::on_backButtons_clicked);

	m_leftButtons = new QPushButton("Left", this);
	m_leftButtons->setMinimumSize(buttonSize);
	m_leftButtons->setSizePolicy(buttonSizePolicy);
	m_buttonGridlayout->addWidget(m_leftButtons, 1, 0, 1, 1);
	connect(m_leftButtons, &QPushButton::clicked, this, &ControllButtons::on_leftButtons_clicked);

	m_rigthButtons = new QPushButton("Right", this);
	m_rigthButtons->setMinimumSize(buttonSize);
	m_rigthButtons->setSizePolicy(buttonSizePolicy);
	m_buttonGridlayout->addWidget(m_rigthButtons, 1, 2, 1, 1);
	connect(m_rigthButtons, &QPushButton::clicked, this, &ControllButtons::on_rigthButtons_clicked);

	m_stopButtons = new QPushButton("Stop", this);
	m_stopButtons->setMinimumSize(buttonSize);
	m_stopButtons->setSizePolicy(buttonSizePolicy);
	m_buttonGridlayout->addWidget(m_stopButtons, 1, 1, 1, 1);
	connect(m_stopButtons, &QPushButton::clicked, this, &ControllButtons::on_stopButtons_clicked);

	if (!m_isLeftHand) {
		m_parent->ui->topGridLayout->addLayout(m_buttonGridlayout, 1, 1);
		m_buttonGridlayout->removeItem(m_spacer);
		return;
	}

	m_parent->ui->topGridLayout->addLayout(m_buttonGridlayout, 1, 0);
	m_buttonGridlayout->addItem(m_spacer, 2, 2);
}

void ControllButtons::switchHand(bool checked)
{
	m_isLeftHand = checked;

	if (m_isLeftHand) {
		m_spacer = new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Minimum);
		m_buttonGridlayout->addItem(m_spacer, 2, 3);
		m_parent->ui->topGridLayout->addLayout(m_buttonGridlayout, 1, 1);
		return;
	}

	m_parent->ui->topGridLayout->addLayout(m_buttonGridlayout, 1, 0);
	m_buttonGridlayout->removeItem(m_spacer);
}

void ControllButtons::addProgressBars(BodyProgressBars *progressBars)
{
	m_buttonGridlayout->addWidget(progressBars, 1, 3, 1, 2);
}

void ControllButtons::removeProgressBars(BodyProgressBars *progressBars)
{
	m_buttonGridlayout->removeWidget(progressBars);
}

void ControllButtons::on_forwardButtons_clicked()
{
	m_parent->robot->setTranslationSpeed(500);
	*m_reverseRobot = false;
	*m_forwardRobot = true;
}

void ControllButtons::on_backButtons_clicked()
{
	m_parent->robot->setTranslationSpeed(-250);
	*m_reverseRobot = true;
	*m_forwardRobot = false;
}

void ControllButtons::on_leftButtons_clicked()
{
	m_parent->robot->setRotationSpeed(3.14159 / 2);
	*m_reverseRobot = false;
	*m_forwardRobot = false;
}

void ControllButtons::on_rigthButtons_clicked()
{
	m_parent->robot->setRotationSpeed(-3.14159 / 2);
	*m_reverseRobot = false;
	*m_forwardRobot = false;
}

void ControllButtons::on_stopButtons_clicked()
{
	m_parent->robot->setTranslationSpeed(0);
}

