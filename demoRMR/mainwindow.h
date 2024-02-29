#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "QLed.h"
#include <QMainWindow>
#include <QMessageBox>
#include <QTimer>

#ifdef _WIN32
#include <windows.h>
#endif

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "robot.h"
#include "stylesheeteditor.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sys/types.h>

#include <QJoysticks.h>

namespace Ui {
class MainWindow;
}

///toto je trieda s oknom.. ktora sa spusti ked sa spusti aplikacia.. su tu vsetky gombiky a spustania...
class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	bool useCamera1;
	// cv::VideoCapture cap;

	int actIndex;
	// cv::Mat frame[3];

	cv::Mat frame[3];
	explicit MainWindow(QWidget *parent = 0);
	~MainWindow();

	int processThisLidar(LaserMeasurement laserData);

	int processThisRobot(TKobukiData robotdata);

	int processThisCamera(cv::Mat cameraData);

private:
	void disableAllButtons(bool disable);
	bool isIPValid(const QString &ip);

private slots:
	void on_pushButton_9_clicked();

	void on_pushButton_2_clicked();

	void on_pushButton_3_clicked();

	void on_pushButton_6_clicked();

	void on_pushButton_5_clicked();

	void on_pushButton_4_clicked();

	void on_pushButton_clicked();

	void on_emgStopButton_clicked();

	void on_changeStyleSheet_triggered();

    void parse_lidar_data(LaserMeasurement laserData, uint16_t *distance);
	//protected:
	//	void contextMenuEvent(QContextMenuEvent *event) override;

public slots:
	void setUiValues(double robotX, double robotY, double robotFi);

signals:
	void uiValuesChanged(double newrobotX, double newrobotY, double newrobotFi); ///toto nema telo

private:
	//--skuste tu nic nevymazat... pridavajte co chcete, ale pri odoberani by sa mohol stat nejaky drobny problem, co bude vyhadzovat chyby
	Ui::MainWindow *ui;
	void paintEvent(QPaintEvent *event); // Q_DECL_OVERRIDE;
	int updateLaserPicture;
	LaserMeasurement copyOfLaserData;
	std::string m_ipaddress;
	Robot *robot;
	TKobukiData robotdata;
	int datacounter;
	QTimer *timer;

	QJoysticks *instance;

	double forwardspeed;  //mm/s
	double rotationspeed; //omega/s

	StyleSheetEditor *m_styleSheetEditor;

	QLed *m_connectionLed;

    uint16_t distanceFromWall[4] = {lidarDistance::FAR};;
};

#endif // MAINWINDOW_H
