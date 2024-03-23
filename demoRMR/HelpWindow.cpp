#include "HelpWindow.h"

HelpWindow::HelpWindow(QWidget *parent)
	: QDialog(parent)
{
	this->setWindowTitle("Help");
	ui.setupUi(this);
}

