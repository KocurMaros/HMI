#ifndef MYTOOLBUTTON_H
#define MYTOOLBUTTON_H

#include <QObject>
#include <QToolButton>
#include <QTimer>
#include <QTime>

#define LED_WIDTH 32
#define EMPTY_IP_ADDRESS "0.0.0.0"
#define DISCONNECTED "Disconnected"
#define CONNECTED "Connected      "
#define EMG_STOP "EMG STOP      "

#define COLOR_CONNECTED Qt::green
#define COLOR_DISCONNECTED Qt::red
#define COLOR_EMG_STOP Qt::darkYellow


class QLed : public QToolButton
{
public:
	QLed(QWidget *parent = NULL);
	~QLed();

	void setToConnectedState(const QString &ip);
	void setToDisconnectedState();
	void setToEmgStopState();
	bool isInConnectedState() const { return m_ledColor != COLOR_DISCONNECTED; }

private:
	inline void setLedColor(const QColor &c) { m_ledColor = c; }
	inline void setText(const QString &s) { m_label = s; }
	inline void setText2(const QString &s) { m_label2 = s; }
	inline void setLedWidth(int w) { m_Width = w; }

	void paintEvent(QPaintEvent *event);
	void drawLed(int length);

private:
	QColor m_ledColor;
	QString m_label;
	QString m_label2;
	QString m_ip;
	double m_Width;
};


#endif // MYTOOLBUTTON_H
