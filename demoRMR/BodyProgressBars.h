#pragma once

#include <QWidget>
#include <QProgressBar>

class BodyProgressBars
	: public QWidget
{
	Q_OBJECT;
public:

explicit BodyProgressBars(QWidget *parent = nullptr);
	
signals:

public slots:
private:
	QProgressBar *m_angleProgressBar;
	QProgressBar *m_velocityProgressBar;
};
