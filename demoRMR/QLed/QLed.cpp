#include "QLed.h"
#include <QPainter>
#include <QBrush>
#include <QLinearGradient>
#include <qnamespace.h>

QLed::QLed(QWidget *parent)
	: QToolButton(parent)
	, m_ledColor(QColor(COLOR_DISCONNECTED))
	, m_label(EMPTY_IP_ADDRESS)
	, m_label2(DISCONNECTED)
	, m_Width(LED_WIDTH)
{
}

QLed::~QLed()
{
}

void QLed::setToConnectedState(const QString &ip)
{
	m_ip = ip;
	setLedColor(COLOR_CONNECTED);
	setText(m_ip);
	setText2(CONNECTED);
}

void QLed::setToDisconnectedState()
{
	setLedColor(COLOR_DISCONNECTED);
	setText(EMPTY_IP_ADDRESS);
	setText2(DISCONNECTED);
}

void QLed::setToEmgStopState()
{
	setLedColor(COLOR_EMG_STOP);
	setText(m_ip);
	setText2("EMG Stop");
}

void QLed::paintEvent(QPaintEvent *e)
{
	(void)e;

	QFont font;
	QFontMetrics fontMetrics(font);
	int textLength = fontMetrics.width(" " + m_label + " ");
	int textLength2 = fontMetrics.width(" " + m_label2 + " ");

	if (m_Width >= 24) {
		textLength = textLength2 > textLength ? textLength2 : textLength;
	}
	setFixedWidth(textLength + m_Width);
	setFixedHeight(m_Width);
	drawLed(textLength);
}

void QLed::drawLed(int p_length)
{
	QPainter painter(this);

	painter.setRenderHint(QPainter::Antialiasing, true);

	qreal l_margin = 4;
	qreal l_height = height() - l_margin;
	QRadialGradient gradient(1 + l_height / 2, l_height / 2 + l_margin / 2, m_Width * 3 / 5);

	gradient.setColorAt(0.0, m_ledColor);
	gradient.setColorAt(0.2, m_ledColor);
	gradient.setColorAt(1.0, Qt::black);
	painter.setBrush(gradient);
	painter.setPen(Qt::NoPen);
	painter.drawEllipse(1, static_cast<int>(l_margin / 2), static_cast<int>(l_height), static_cast<int>(l_height));

	qreal l_highlight = m_Width * 2 / 5;
	QRadialGradient radGrad(1 + l_height / 2, l_height / 2 + l_margin / 2 - m_Width / 4, m_Width / 4);

	radGrad.setColorAt(0.0, Qt::white);
	radGrad.setColorAt(1.0, m_ledColor);
	painter.setBrush(radGrad);
	painter.setPen(Qt::NoPen);
	painter.drawEllipse(static_cast<int>(1 + l_height / 2 - l_highlight / 2), static_cast<int>(l_height / 2 - l_highlight / 2 + l_margin / 2 - m_Width / 8),
						static_cast<int>(l_highlight), static_cast<int>(l_highlight - m_Width / 8));


	if (m_Width < 24) {
		painter.setPen(m_ledColor);
		QRect rect(m_Width, 0, p_length, height());
		painter.drawText(rect, Qt::AlignLeft | Qt::AlignVCenter, " " + m_label + " ");
	}
	else {
		painter.setPen(m_ledColor);
		QRect rect(m_Width, 0, p_length, height() / 2);
		painter.drawText(rect, Qt::AlignLeft | Qt::AlignVCenter, " " + m_label + " ");

		QRect rect2(m_Width, height() / 2, p_length, height() / 2);
		painter.drawText(rect2, Qt::AlignLeft | Qt::AlignVCenter, " " + m_label2 + " ");
	}
}

