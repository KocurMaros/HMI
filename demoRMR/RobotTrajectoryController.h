#ifndef ROBOTTRAJECTORYCONTROLLER_H
#define ROBOTTRAJECTORYCONTROLLER_H

#include "PidController.h"
#include "robot.h"
#include <QObject>
#include <QThread>
#include <QTimer>
#include <QWidget>
#include <memory>

#define TO_RADIANS 3.14159 / 180.0

QPointF computeLineParameters(QPointF p1, QPointF p2);

class RobotTrajectoryController : public QObject
{
	Q_OBJECT

public:
	enum class MovementType {
		None,
		Forward,
		Rotation,
		Arc
	};

	QString movementTypeToString(MovementType type) {
		switch (type) {
		case MovementType::None:
			return "None";
		case MovementType::Forward:
			return "Forward";
		case MovementType::Rotation:
			return "Rotation";
		case MovementType::Arc:
			return "Arc";
		}
		return "None";
	}

	using Map = std::vector<std::vector<int>>;
	RobotTrajectoryController(std::shared_ptr<Robot> robot, QObject *window, double timerInterval = 100);

	void setTranslationSpeed(double velocity, bool stopPositionTimer = false, double accelerationRate = 50);
	void setRotationSpeed(double omega, bool stopPositionTimer = false, double accelerationRate = 0.1);
	void setArcSpeed(double velocity, double omega, bool stopPositionTimer = false, double accelerationRate = 50, double omegaRate = 20);

	void rotateRobotTo(double rotation);
	void moveForwardBy(double distance);
	void moveByArcTo(double distance, double rotation);

private:
	bool isNear(double currentVelocity);
	double finalDistanceError();
	double localDistanceError();
	double finalRotationError();
	double localRotationError();

public slots:
	void on_stoppingTimerTimeout_stop();
	void on_accelerationTimerTimeout_control();
	void on_positionTimerTimeout_changePosition();

	void onMoveForwardMove(double speed);
	void onChangeRotationRotate(double speed);
	void handleLinResults(double distance, double rotation, QVector<QPointF> points);
	void handleArcResults(double distance, double rotation, QVector<QPointF> points);

	void on_requestMovement_move(double distance);
	void on_requestRotation_move(double rotation);
	void on_requestArc_move(double distance, double rotation);

private:
signals:
	void requestMovement(double distance);
	void requestRotation(double rotation);
	void requestArc(double distance, double rotation);

private:
	std::shared_ptr<Robot> m_robot;
	QObject *m_mainWindow;

	QTimer m_accelerationTimer;
	QTimer m_positionTimer;
	QTimer m_stoppingTimer;

	std::shared_ptr<PIDController> m_controller;
	std::shared_ptr<PIDController> m_rotationController;

	QVector<QPointF> m_points;

	MovementType m_movementType;
	bool m_stopped;

	double m_forwardSpeed;
	double m_rotationSpeed;

	double m_targetVelocity;
	double m_accelerationRate;
	double m_omegaRate;

	double m_targetPosition;
	double m_targetOmega;
	double m_lastArcSpeed;

	int m_fileWriteCounter;
	bool m_arcExpected;
	Map m_map;
};

std::ostream &operator<<(std::ostream &os, const RobotTrajectoryController::Map &map);

#endif // ROBOTTRAJECTORYCONTROLLER_H
