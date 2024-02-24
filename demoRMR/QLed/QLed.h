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


class QLed : public QToolButton
{
public:
	QLed(QWidget *parent = NULL);
	~QLed();

	void setToConnectedState(const QString &ip);
	void setToDisconnectedState();
	bool isInConnectedState() const { return m_label2 == CONNECTED; }

private:
	inline void setLedColor(const QColor &c) { m_ledColor = c; }
	inline void setText(const QString &s) { m_label = s; }
	inline void setText2(const QString &s) { m_label2 = s; }
	inline void setLedWidth(int w) { m_Width = w; }

	void paintEvent(QPaintEvent *event);
	void drawLed(int length);

private:
	QColor m_ledColor;
	QString m_label, m_label2;
	double m_Width;
};


#endif // MYTOOLBUTTON_H
