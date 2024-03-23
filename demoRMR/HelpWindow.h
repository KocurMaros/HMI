#pragma once

#include <QDialog>

#include "ui_HelpWindow.h"

class HelpWindow
	: public QDialog
{
	friend class MainWindow;
	Q_OBJECT;
public:
	explicit HelpWindow(QWidget *parent = nullptr);

private:
	Ui::HelpWindow ui;
};
