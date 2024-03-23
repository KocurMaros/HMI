#pragma once

#include <QDialog>

#include "ui_HelpWindow.h"

class HelpWindow : public QDialog
{
	friend class MainWindow;
	Q_OBJECT;

public:
	explicit HelpWindow(QWidget *parent = nullptr);

private:
	void on_anchorClicked_scroll(const QUrl &url);

private:
	Ui::HelpWindow ui;
	QString m_content;
};
