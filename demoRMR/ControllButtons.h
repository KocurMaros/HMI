#include "mainwindow.h"
#include <qlayoutitem.h>

class ControllButtons
	: public QWidget
{
	Q_OBJECT;
public:

	explicit ControllButtons(MainWindow *parent = nullptr);

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
};
