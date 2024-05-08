#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "BodyProgressBars.h"
#include "FloodPlanner.h"
#include "HelpWindow.h"
#include "MapLoader.h"
#include "PositionTracker.h"
#include "QLed.h"
#include "RobotTrajectoryController.h"
#include "object_detection.h"
#include "robot.h"
#include "stylesheeteditor.h"
#include "ui_mainwindow.h"

#include <QJoysticks.h>
#include <QKeyEvent>
#include <QMainWindow>
#include <QMessageBox>
#include <QThread>
#include <QTimer>
#include <mutex>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include <sys/types.h>

#ifdef _WIN32
#include <windows.h>
#endif

namespace Ui {
class MainWindow;
}

class ControllButtons;

///toto je trieda s oknom.. ktora sa spusti ked sa spusti aplikacia.. su tu vsetky gombiky a spustania...
class MainWindow : public QMainWindow
{
	friend class ControllButtons;
	friend class MapLoader;
	Q_OBJECT

	enum UserMode {
		Telecontrol,
		Supervisor,
	};

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
	QRect getFrameGeometry() const { return m_ui->frame->geometry(); }
	QPair<double, double> calculateTrajectory();
	QPair<double, double> calculateTrajectoryTo(const QPointF &point);
	double finalRotationError();
	double localRotationError(const QPointF &point);

signals:
	void arcResultsReady(double distance, double rotation, QVector<QPointF> points);
	void moveForward(double speed);
	void changeRotation(double angle);
	void requestPath(const QPointF &start, const QPointF &end);

private:
	void disableAllButtons(bool disable);
	void inPaintEventProcessSkeleton();
	void setRobotDirection();
	void drawLidarData(QPainter &painter, QPen &pen, QRect &rect, int scale = 20);
	void drawImageData(QPainter &painter, QRect &rect, bool mini = false);
	void calculateOdometry(const TKobukiData &robotdata);
	void paintEvent(QPaintEvent *event) override; // Q_DECL_OVERRIDE;
	void parse_lidar_data(LaserMeasurement laserData, uint16_t *distance);
	void calc_colisions_points(LaserMeasurement laserData, bool *colisions);
	void paintTeleControl();
	void paintSupervisorControl();
	double getX();
	double getY();
	double getFi();

	void bodyControlTeleview();
	void bodyControlSupervisor();

	void pushButtonTeleview();
	void pushButtonSupervisor();

	void mousePressEvent(QMouseEvent *event) override;
	void mouseReleaseEvent(QMouseEvent *event) override;
	void mouseMoveEvent(QMouseEvent *event) override;
	QPointF createLineParams(const QPointF &p);

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
	void on_resultsReady_updateUi(double x, double y, double fi);
	void on_actionTelecontrol_triggered();
	void on_actionSupervisor_triggered();
	void openFileDialog();
	void calculePositionOfObject(cv::Point center_of_object);

public slots:
    void updateLidarCircle(cv::Point center_of_object);
	void setUiValues(double robotX, double robotY, double robotFi);
	void on_rtc_removePoint();
	void handlePath(QVector<QPointF> path);
	void returnHomeActivated();

signals:
	void uiValuesChanged(double newrobotX, double newrobotY, double newrobotFi); ///toto nema telo
	void changeSpeed(double forwardspeed, double rotationspeed);
    void haffTransform(cv::Mat frame);
	void changeArc(double forwardspeed, double rotationspeed);
	void positionResults(const TKobukiData &robotdat, double correction);
	void batteryLevel(int battery);
    void randomSignalObjectDetectionCircleOtherThreadRandom(cv::Point m_objectDetected);

private:
	//--skuste tu nic nevymazat... pridavajte co chcete, ale pri odoberani by sa mohol stat nejaky drobny problem, co bude vyhadzovat chyby
	Ui::MainWindow *m_ui;
	QThread *m_odometryThread;
	PositionTracker *m_positionTracker;
	int m_updateLaserPicture;
	LaserMeasurement m_copyOfLaserData;
	std::string m_ipaddress;
	std::shared_ptr<Robot> m_robot;
	TKobukiData m_robotdata;
	int m_datacounter;
	int m_updateSkeletonPicture;
	bool m_useSkeleton;
	skeleton m_skeleJoints;
	QTimer *m_timer;

	QJoysticks *m_instance;

	double forwardspeed;	//mm/s
	double m_rotationspeed; //omega/s
	double m_prevForwardspeed;
	double m_prevRotationspeed;

	StyleSheetEditor *m_styleSheetEditor;
	HelpWindow *m_helpWindow;

	QLed *m_connectionLed;

	uint16_t m_distanceFromWall[8] = { lidarDistance::FAR };
	double m_avgDist[8] = { 0 };

	bool m_colisionDetected = false;
	bool m_reverseRobot = false;
	bool m_forwardRobot = false;

	QImage m_colisionImage;

	bool m_motionButtonsVisible;
	bool m_leftHandedMode;
	ControllButtons *m_controllButtons;
	BodyProgressBars *m_bodyProgressBars;
	
	int m_lastLeftEncoder;
	int m_lastRightEncoder;
	double m_fiCorrection;
	bool m_robotStartupLocation;

	
    ObjectDetection *m_ObjectDetection;
	// ObjectDetection m_ObjectDetection;
	bool start_pressed = false;

    double m_global_center_of_circle;
	bool m_draw_c;
	bool m_draw_c_was;
	double object_pos_x;
	double object_pos_y;
	double robot_x_find_object;
	double robot_y_find_object;
	double robot_fi_find_object;
	// double m_prev_size_window_width;
	// double m_prev_size_window_height;
	bool m_detection_update_lidar;

    uint32_t m_frame_counter = 0;
	cv::Point m_objectOnMap;
    cv::Point m_objectDetected;
   // std::shared_ptr<ObjectDetection> m_ObjectDetection;

	std::mutex m_odometryLock;
	double m_fi;
	double m_x;
	double m_y;

	std::shared_ptr<MapLoader> m_mapLoader;
	QVector<QPointF> m_transitionPoints;
	std::shared_ptr<QPointF> m_endPosition;
	UserMode m_userMode;

	std::shared_ptr<RobotTrajectoryController> m_trajectoryController;

	QVector<QMetaObject::Connection> m_rtcConnections;
	QPushButton *m_loadMapButton;
	QPushButton *m_returnHomeButton;
	QVector<QMetaObject::Connection> m_supervisorConnections;
	bool m_dragNDrop;
	std::atomic<bool> m_inCollision;

	std::shared_ptr<FloodPlanner> m_floodPlanner;
	QThread *m_floodPlannerThread;
	QPointF m_dockPosition;
	bool m_goingHome;
};

#endif // MAINWINDOW_H
