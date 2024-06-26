#include "mainwindow.h"
#include <qlayoutitem.h>

class ControllButtons : public QWidget
{
	Q_OBJECT;

public:
	explicit ControllButtons(bool *reverse, bool *forward, MainWindow *parent = nullptr);
	void switchHand(bool checked);
	void addProgressBars(BodyProgressBars *progressBars);
	void removeProgressBars(BodyProgressBars *progressBars);
	bool reverse() const { return m_reverseRobot; }
	bool forward() const { return m_forwardRobot; }

public slots:
	void on_forwardButtons_clicked();
	void on_backButtons_clicked();
	void on_leftButtons_clicked();
	void on_rigthButtons_clicked();
	void on_stopButtons_clicked();

private:
	MainWindow *m_parent;
	QGridLayout *m_buttonGridlayout;
	QPushButton *m_forwardButtons;
	QPushButton *m_backButtons;
	QPushButton *m_leftButtons;
	QPushButton *m_rigthButtons;
	QPushButton *m_stopButtons;

	QSpacerItem *m_spacer;

	bool m_isLeftHand;
	bool *m_reverseRobot;
	bool *m_forwardRobot;
};
