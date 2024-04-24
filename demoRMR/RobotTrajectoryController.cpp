#include "mainwindow.h"
#include "PidController.h"
#include "qnamespace.h"
#include "RobotTrajectoryController.h"
#include <QDebug>
#include <QTimer>
#include <algorithm>
#include <memory>

QPointF computeLineParameters(QPointF p1, QPointF p2)
{
	QPointF line;
	// Compute slope (a)
	if (p1.x() != p2.x()) {
		auto x = (p2.y() - p1.y()) / (p2.x() - p1.x());
		line.setX(x);
	}
	else {
		// If the line is vertical, slope is infinity, so set a to a large value
		line.setY(1e9);
	}
	// Compute intercept (b)
	line.ry() = p1.y() - line.x() * p1.x();
	return line;
}

RobotTrajectoryController::RobotTrajectoryController(std::shared_ptr<Robot> robot, QObject *window, double timerInterval)
	: QObject()
	, m_robot(robot)
	, m_mainWindow(window)
	, m_accelerationTimer(this)
	, m_positionTimer(this)
	, m_stoppingTimer(this)
	, m_controller(nullptr)
	, m_rotationController(nullptr)
	, m_forwardSpeed(0)
	, m_rotationSpeed(0)
	, m_lastArcSpeed(0)
	, m_fileWriteCounter(0)
	, m_arcExpected(false)
	, m_map(50, std::vector<int>(50, 0))
{
	m_accelerationTimer.setInterval(timerInterval);
	m_accelerationTimer.setSingleShot(false);

	m_positionTimer.setInterval(timerInterval * 2);
	m_positionTimer.setSingleShot(false);

	m_stoppingTimer.setInterval(3'000);
	m_stoppingTimer.setSingleShot(true);

	connect(&m_stoppingTimer, &QTimer::timeout, this, &RobotTrajectoryController::on_stoppingTimerTimeout_stop, Qt::QueuedConnection);
	connect(&m_accelerationTimer, &QTimer::timeout, this, &RobotTrajectoryController::on_accelerationTimerTimeout_control, Qt::QueuedConnection);
	connect(&m_positionTimer, &QTimer::timeout, this, &RobotTrajectoryController::on_positionTimerTimeout_changePosition, Qt::QueuedConnection);

	connect(this, &RobotTrajectoryController::requestMovement, this, &RobotTrajectoryController::on_requestMovement_move, Qt::QueuedConnection);
	connect(this, &RobotTrajectoryController::requestRotation, this, &RobotTrajectoryController::on_requestRotation_move, Qt::QueuedConnection);
	connect(this, &RobotTrajectoryController::requestArc, this, &RobotTrajectoryController::on_requestArc_move, Qt::QueuedConnection);
}

void RobotTrajectoryController::setTranslationSpeed(double velocity, bool stopPositionTimer, double accelerationRate)
{
	m_rotationSpeed = 0;
	m_movementType = MovementType::Forward;
	m_stoppingTimer.stop();
	if (stopPositionTimer) {
		m_positionTimer.stop();
	}

	m_targetVelocity = velocity;
	m_accelerationRate = accelerationRate;

	if (velocity == 0) {
		m_forwardSpeed = 0;
		m_robot->setTranslationSpeed(0);
		return;
	}

	if (m_targetVelocity < m_forwardSpeed) {
		m_accelerationRate = -m_accelerationRate;
	}

	m_accelerationTimer.start();
}

void RobotTrajectoryController::setRotationSpeed(double omega, bool stopPositionTimer, double accelerationRate)
{
	m_forwardSpeed = 0;
	m_lastArcSpeed = 0;
	m_movementType = MovementType::Rotation;
	m_stoppingTimer.stop();

	if (stopPositionTimer) {
		m_positionTimer.stop();
	}

	m_targetVelocity = omega;
	m_accelerationRate = accelerationRate;

	if (omega == 0) {
		m_rotationSpeed = 0;
		m_robot->setRotationSpeed(0);
		return;
	}

	if (m_targetVelocity < m_rotationSpeed) {
		m_accelerationRate = -m_accelerationRate;
	}

	m_accelerationTimer.start();
}

void RobotTrajectoryController::setArcSpeed(double velocity, double omega, bool stopPositionTimer, double accelerationRate, double omegaRate)
{
	m_movementType = MovementType::Arc;
	if (velocity > m_lastArcSpeed + accelerationRate) {
		velocity = m_lastArcSpeed + accelerationRate;
	}

	m_lastArcSpeed = velocity;
	m_robot->setArcSpeed(velocity, omega);
}

void RobotTrajectoryController::rotateRobotTo(double rotation)
{
	m_movementType = MovementType::Rotation;
	m_stopped = false;

	m_accelerationTimer.stop();
	m_stoppingTimer.stop();

	m_controller = std::make_shared<PIDController>(0.75, 0, 0, rotation);
	m_positionTimer.start();
}

void RobotTrajectoryController::moveForwardBy(double distance)
{
	m_movementType = MovementType::Forward;
	m_stopped = false;

	m_accelerationTimer.stop();
	m_stoppingTimer.stop();

	m_controller = std::make_shared<PIDController>(1000, 0, 0, distance);
	m_positionTimer.start();
}

void RobotTrajectoryController::moveByArcTo(double distance, double rotation)
{
	m_movementType = MovementType::Arc;
	m_stopped = false;

	m_accelerationTimer.stop();
	m_stoppingTimer.stop();

	m_controller = std::make_shared<PIDController>(1000, 0, 0, distance);
	m_rotationController = std::make_shared<PIDController>(1, 0, 0, rotation);

	m_positionTimer.start();
}

bool RobotTrajectoryController::isNear(double currentVelocity)
{
	return std::abs(currentVelocity - m_targetVelocity) <= std::abs(m_accelerationRate / 2.);
}

double RobotTrajectoryController::finalDistanceError()
{
	MainWindow *win = qobject_cast<MainWindow *>(m_mainWindow);
	auto [distance, rotation] = win->calculateTrajectory();
	return distance;
}

double RobotTrajectoryController::localDistanceError()
{
	QPointF point = m_points.first();

	MainWindow *win = qobject_cast<MainWindow *>(m_mainWindow);
	auto [distance, rotation] = win->calculateTrajectoryTo({ point.x(), point.y() });

	return distance;
}

double RobotTrajectoryController::finalRotationError()
{
	MainWindow *win = qobject_cast<MainWindow *>(m_mainWindow);
	return win->finalRotationError();
}

double RobotTrajectoryController::localRotationError()
{
	QPointF point = m_points.first();

	MainWindow *win = qobject_cast<MainWindow *>(m_mainWindow);
	return win->localRotationError({ point.x(), point.y() });
}

void RobotTrajectoryController::on_stoppingTimerTimeout_stop()
{
	m_robot->setTranslationSpeed(0);
	m_movementType = MovementType::None;
	m_forwardSpeed = 0;
	m_rotationSpeed = 0;

	m_accelerationTimer.stop();
	m_positionTimer.stop();
	m_stoppingTimer.stop();

	// m_stoppingTimer.setInterval(3'000);
	m_stopped = true;
}

void RobotTrajectoryController::on_accelerationTimerTimeout_control()
{
	if (isNear(m_forwardSpeed) || isNear(m_rotationSpeed)) {
		m_accelerationTimer.stop();
		m_stoppingTimer.start();
	}

	static auto limit = [](double &speed, const double target, double rate) {
		if (speed > 0 && target < 0) {
			speed += rate;
		}
		else if (std::abs(speed + rate) > std::abs(target)) {
			speed = target;
		}
		else {
			speed += rate;
		}
	};

	if (m_movementType == MovementType::Rotation) {
		limit(m_rotationSpeed, m_targetVelocity, m_accelerationRate);
		m_robot->setRotationSpeed(m_rotationSpeed);
	}
	else if (m_movementType == MovementType::Forward) {
		limit(m_forwardSpeed, m_targetVelocity, m_accelerationRate);
		m_robot->setTranslationSpeed(m_forwardSpeed);
	}
}

void RobotTrajectoryController::on_positionTimerTimeout_changePosition()
{
	static double error, maxCorrection;
	MainWindow *win = qobject_cast<MainWindow *>(m_mainWindow);

	if (m_movementType == MovementType::Rotation) {
		error = localRotationError();
		if (m_arcExpected) {
			maxCorrection = 0.3;
		}
		else {
			maxCorrection = 0.1;
		}
	}
	else if (m_movementType == MovementType::Forward) {
		error = localDistanceError();
		auto rotError = localRotationError();
		maxCorrection = std::abs((std::sin(rotError) * error) * 1.1);
	}
	else if (m_movementType == MovementType::Arc) {
		error = localDistanceError();
		if (finalDistanceError() == localRotationError()) {
			maxCorrection = 0.05;
		}
		else {
			maxCorrection = 0.1;
		}
	}

	if (std::abs(error) < maxCorrection) {
		if (m_movementType == MovementType::Rotation && !m_arcExpected) {
			emit requestMovement(localDistanceError());
		}
		else if (m_movementType == MovementType::Rotation && m_arcExpected) {
			emit requestArc(localDistanceError(), localRotationError());
		}
		else if (m_movementType == MovementType::Forward && finalDistanceError() > 0.05) {
			if (m_points.size() > 1) {
				m_points.removeFirst();
				emit removePoint();
			}

			emit requestRotation(localRotationError());
		}
		else if (m_movementType == MovementType::Arc) {
			if (m_points.size() > 1) {
				m_points.removeFirst();
				emit removePoint();
			}

			double rotation = localRotationError();
			if (rotation > PI / 2 || rotation < -PI / 2) {
				m_arcExpected = true;
				m_forwardSpeed = 0;
				m_rotationSpeed = 0;
				emit requestRotation(rotation);
			}
			else {
				emit requestArc(localDistanceError(), localRotationError());
			}
		}

		if (m_movementType == MovementType::Rotation || m_movementType == MovementType::Forward || m_points.size() == 1) {
			on_stoppingTimerTimeout_stop();
		}
		return;
	}

	double u;
	if (m_movementType == MovementType::Rotation) {
		u = m_controller->computeFromError(error);
		setRotationSpeed(u);
	}
	else if (m_movementType == MovementType::Forward) {
		u = m_controller->computeFromError(error, true);
		setTranslationSpeed(u);
	}
	else if (m_movementType == MovementType::Arc) {
		u = m_controller->computeFromError(finalDistanceError(), true);
		double o = m_rotationController->computeFromError(localRotationError());
		setArcSpeed(u, u / o);
	}
}

void RobotTrajectoryController::onMoveForwardMove(double speed)
{
	setTranslationSpeed(speed, true);
}

void RobotTrajectoryController::onChangeRotationRotate(double speed)
{
	setRotationSpeed(speed, true);
}

void RobotTrajectoryController::onMoveArcMove(double speed, double rotation)
{
	setArcSpeed(speed, rotation);
}

void RobotTrajectoryController::handleLinResults(double distance, double rotation, QVector<QPointF> points)
{
	m_points = points;
	m_arcExpected = false;
	rotateRobotTo(rotation);
}

void RobotTrajectoryController::handleArcResults(double distance, double rotation, QVector<QPointF> points)
{
	m_points = points;
	qDebug() << "Transition points: " << m_points;
	if (rotation > PI / 2 || rotation < -PI / 2) {
		m_arcExpected = true;
		rotateRobotTo(rotation);
		return;
	}

	moveByArcTo(distance, rotation);
}

void RobotTrajectoryController::on_requestMovement_move(double distance)
{
	moveForwardBy(distance);
}

void RobotTrajectoryController::on_requestRotation_move(double rotation)
{
	rotateRobotTo(rotation);
}

void RobotTrajectoryController::on_requestArc_move(double distance, double rotation)
{
	moveByArcTo(distance, rotation);
}
