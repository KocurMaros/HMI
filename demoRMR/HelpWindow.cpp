#include "HelpWindow.h"

#include <QTextDocument>
#include <QFile>
#include <QDebug>

HelpWindow::HelpWindow(QWidget *parent)
	: QDialog(parent)
{
	this->setWindowTitle("Help");
	setFixedSize(730, 381);
	QFile file(":/md/help.md");

	if (file.open(QIODevice::ReadOnly | QIODevice::Text)) {
		m_content = file.readAll();
	} else {
		m_content = "Error loading help file.";
	}
	file.close();

	ui.setupUi(this);

	ui.helpText->setMarkdown(m_content);

	connect(ui.helpText, &QTextBrowser::anchorClicked, this, &HelpWindow::on_anchorClicked_scroll);
}


void HelpWindow::on_anchorClicked_scroll(const QUrl &url)
{
	QString anchor = url.toString().mid(1).replace("-", " "); // Remove "#" prefix

	/*
	 * https://wiki.qt.io/New_Features_in_Qt_5.14
	 * QTextDocument supports reading and writing Markdown format, as an alternative to HTML. If you read HTML and write
	 * Markdown, or vice-versa, the formatting should be preserved to the extent that the CommonMark and GitHub specs allow
	 * (including headings, tables, bullet lists, block quotes and code blocks); but we don't guarantee all cases yet,
	 * because it's thinly tested so far.
	 */
	ui.helpText->scrollToAnchor(anchor);
}
