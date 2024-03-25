#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "HelpWindow.h"
#include "QLed.h"
#include <QMainWindow>
#include <QMessageBox>
#include <QTimer>
#include <QKeyEvent>

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

class ControllButtons;

///toto je trieda s oknom.. ktora sa spusti ked sa spusti aplikacia.. su tu vsetky gombiky a spustania...
class MainWindow : public QMainWindow
{
	friend class ControllButtons;
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
	int processThisSkeleton(skeleton skeledata);

private:
	void disableAllButtons(bool disable);
	bool isIPValid(const QString &ip);
	void inPaintEventProcessSkeleton();
	void setRobotDirection();
protected:
	void keyPressEvent(QKeyEvent *event) override;

private slots:
	void on_pushButton_9_clicked();

	void on_pushButton_clicked();

	void on_emgStopButton_clicked();

	void on_bodyControlButton_clicked();

	void on_changeStyleSheet_triggered();
	void on_actionAdd_motion_buttons_triggered();
	void on_actionChangeHand_toggled();
	void on_actionShowHelp_triggered();

	void parse_lidar_data(LaserMeasurement laserData, uint16_t *distance);
	void calc_colisions_points(LaserMeasurement laserData, bool *colisions);
	//protected:
	//	void contextMenuEvent(QContextMenuEvent *event) override;

public slots:
	void setUiValues(double robotX, double robotY, double robotFi);

signals:
	void uiValuesChanged(double newrobotX, double newrobotY, double newrobotFi); ///toto nema telo
    void changeSpeed(double forwardspeed, double rotationspeed);

private:
	//--skuste tu nic nevymazat... pridavajte co chcete, ale pri odoberani by sa mohol stat nejaky drobny problem, co bude vyhadzovat chyby
	Ui::MainWindow *ui;
	void paintEvent(QPaintEvent *event); // Q_DECL_OVERRIDE;
	int updateLaserPicture;
	LaserMeasurement copyOfLaserData;
	std::string m_ipaddress;
	std::unique_ptr<Robot> robot;
	TKobukiData robotdata;
	int datacounter;
	int updateSkeletonPicture;
	bool useSkeleton;
	skeleton skeleJoints;
	QTimer *timer;

	QJoysticks *instance;

	double forwardspeed;  //mm/s
	double rotationspeed; //omega/s
    double prev_forwardspeed;
    double prev_rotationspeed;

	StyleSheetEditor *m_styleSheetEditor;
	HelpWindow *m_helpWindow;

	QLed *m_connectionLed;

	uint16_t distanceFromWall[8] = { lidarDistance::FAR };
	double avg_dist[8] = { 0 };

	bool colisionDetected = false ;
	bool reverse_robot = false;
	bool forward_robot = false;


	QImage colision_image;

	bool m_motionButtonsVisible;
	bool m_leftHandedMode;
	ControllButtons *m_controllButtons;
};

#endif // MAINWINDOW_H
