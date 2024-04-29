#include "PidController.h"

PIDController::PIDController(double p, double i, double d, double target)
	: m_kp(p)
	, m_ki(i)
	, m_kd(d)
	, m_target(target)
	, m_integral(0)
	, m_prev_error(0)
	, m_clampedOutput(0)
	, m_virginOutput(0)
{
}

double PIDController::compute(double current)
{
	double error = m_target - current;
	return computeFromError(error);
}

double PIDController::computeFromError(double error, bool limit)
{
	m_integral += error;
	m_integral += m_clampedOutput - m_virginOutput;

	double derivative = error - m_prev_error;

	m_virginOutput = m_kp * error + m_ki * m_integral + m_kd * derivative;

	if (limit) {
		if (m_virginOutput < 0) {
			m_clampedOutput = std::clamp(m_virginOutput, -250., -15.);
		}
		else {
			m_clampedOutput = std::clamp(m_virginOutput, 15., 250.);
		}
	}

	m_prev_error = m_clampedOutput;

	if (limit) {
		return m_clampedOutput;
	}

	return m_virginOutput;
}
