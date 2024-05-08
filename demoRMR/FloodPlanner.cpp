#include "FloodPlanner.h"
#include "RobotTrajectoryController.h"
#include "mainwindow.h"

#include <algorithm>
#include <iostream>
#include <iomanip>

#include <QFile>
#include <QDebug>
#include <QSet>

#define START_FLAG -5
#define END_FLAG -6

uint qHash(const QPoint &point)
{
	return qHash(point.x()) ^ qHash(point.y());
}

bool operator>(const QPointF &a, const QPointF &b)
{
	return a.x() > b.x() || a.y() > b.y();
}

FloodPlanner::FloodPlanner(const QString &filename)
	: QObject(nullptr)
	, m_map(new Map())
{
	loadMap(filename);
}

QPoint FloodPlanner::toMapCoord(const QPointF &point)
{
	auto tmp = QPointF(point.x(), -point.y()) * 1000. / TILE_SIZE + QPoint(m_map->at(0).size() / 4., m_map->size() / 2.);
	return tmp.toPoint();
}

QPointF FloodPlanner::toWorlCoord(const QPoint &point)
{
	return QPointF(((point.x() - m_map->at(0).size() / 4.) * TILE_SIZE + TILE_SIZE / 2.) / 1000.,
				   (-(point.y() - m_map->size() / 2.) * TILE_SIZE - TILE_SIZE / 2.) / 1000.);
}

void FloodPlanner::loadMap(const QString &filename)
{
	m_map->clear();

	fillMap(filename);
	// std::cout << "Loaded from file:\n" << *m_map << std::endl;
	expandObstacles();
	// std::cout << "Expanded with obstacles:\n" << *m_map << std::endl;
}

void FloodPlanner::on_requestPath_plan(const QPointF &start, const QPointF &end)
{
	QPoint s = toMapCoord(start);
	QPoint e = toMapCoord(end);

	if (!isTileValid(*m_map, e)) {
		qDebug() << "Invalid start or end point";
		printMapWithPath(QVector<QPoint> { s, e });
		return;
	}

	qDebug() << "Start: " << s << " End: " << e;
	if (s == e) {
		emit pathPlanned({end});
		return;
	}

	auto path = planPath(s, e, TrajectoryType::Diagonal);

	emit pathPlanned(path);
}

void FloodPlanner::fillMap(const QString &filename)
{
	QFile file(filename);
	if (!file.open(QIODevice::ReadOnly)) {
		throw std::runtime_error("Failed to open file");
	}

	while (!file.atEnd()) {
		QString line = file.readLine();
		QVector<int> row;
		for (auto c : line) {
			if (c == '#') {
				row.push_back(-1);
			}
			else {
				row.push_back(0);
			}
		}
		m_map->push_back(row);
	}
}

void FloodPlanner::expandObstacles()
{
	auto directions = getDirections(TrajectoryType::Diagonal);
	for (int y = 0; y < m_map->size(); ++y) {
		for (int x = 0; x < m_map->at(y).size(); ++x) {
			if (m_map->at(y).at(x) == -1) {
				for (size_t i = 0; i < directions.first.size(); i++) {
					QPoint next = QPoint(x, y) + QPoint(directions.first[i], directions.second[i]);
					if (isTileValid(*m_map, next) && m_map->at(next.y()).at(next.x()) == 0) {
						(*m_map)[next.y()][next.x()] = -2;
					}
				}
			}
		}
	}
}

bool FloodPlanner::isInCFree(const QPoint &point)
{
	return m_map->at(point.y()).at(point.x()) >= 0;
}

QPoint FloodPlanner::nearestCFreePoint(const QPoint &point, TrajectoryType type)
{
	if (isInCFree(point)) {
		return point;
	}

	Map map = *m_map;

	auto [dx, dy] = getDirections(type);

	QVector<QPoint> toVisit { point };
	QSet<QPoint> visited { point };

	while (!toVisit.empty()) {
		QPoint curr = toVisit.front();
		toVisit.pop_front();

		if (map[curr.y()][curr.x()] == 0) {
			return curr;
		}

		for (int i = 0; i < dx.size(); ++i) {
			QPoint next = curr + QPoint(dx[i], dy[i]);
			if (isTileValid(map, next) && !isTileObstacle(map, next) && !visited.contains(next)) {
				toVisit.push_back(next);
				visited.insert(next);
			}
		}
	}
}

int FloodPlanner::maxFromNeighbours(const Map &map, const QPoint &point)
{
	auto directions = getDirections(TrajectoryType::Diagonal);

	int max = 0;
	for (size_t i = 0; i < directions.first.size(); i++) {
		QPoint next = point + QPoint(directions.first[i], directions.second[i]);
		if (isTileValid(map, next) && map[next.y()][next.x()] > max) {
			max = map[next.y()][next.x()];
		}
	}

	return max;
}

bool FloodPlanner::isTileObstacle(const Map &map, const QPoint &point)
{
	return m_map->at(point.y()).at(point.x()) < 0;
}

bool FloodPlanner::isTileValid(const Map &map, const QPoint &point)
{
	// Check for out of bounds.
	if (point.x() < 0 || point.x() >= map[0].size()) {
		return false;
	}
	if (point.y() < 0 || point.y() >= map.size()) {
		return false;
	}

	return true;
}

void FloodPlanner::markTiles(Map &map, const QPoint &start, const QPoint &end, TrajectoryType type)
{
	auto [dx, dy] = getDirections(type);

	QVector<QPoint> toVisit { start };
	QSet<QPoint> visited { start };

	while (!toVisit.empty()) {
		QPoint curr = toVisit.front();
		toVisit.pop_front();

		if (curr == end) {
			return;
		}

		for (int i = 0; i < dx.size(); ++i) {
			QPoint next = curr + QPoint(dx[i], dy[i]);
			if (isTileValid(map, next) && !isTileObstacle(map, next) && !visited.contains(next)) {
				map[next.y()][next.x()] = map[curr.y()][curr.x()] + 1;
				toVisit.push_back(next);
				visited.insert(next);
			}
		}
	}
}

QVector<QPointF> FloodPlanner::planPath(const QPoint &start, const QPoint &end, TrajectoryType type)
{
	Map map = *m_map;

	auto s = nearestCFreePoint(start, type);
	auto e = nearestCFreePoint(end, type);

	std::cout << "Cfree:\n";
	printMapWithPath(QVector<QPoint> { s, e });

	markTiles(map, e, s, type);

	std::cout << "Flooded:\n";
	std::cout << map << std::endl;

	qDebug() << "Creating path from start to end from the flooded data.";
	auto path = pathFromMap(map, s, e, TrajectoryType::Diagonal);

	// path.pop_back();
	// path.push_back(end);

	std::cout << "Unpruned:\n";
	printMapWithPath(path);

	path.push_front(start);
	auto pathF = prunePath(path);

	qDebug() << pathF;

	return pathF;
}

QVector<QPoint> FloodPlanner::pathFromMap(Map &map, const QPoint &start, const QPoint &end, TrajectoryType type)
{
	QVector<QPoint> path;
	QPoint curr = start;
	auto [dx, dy] = getDirections(type);

	// We do this to ensure that the start point is marked correctly even if we want to go the the beginning of the path.
	if (map[start.y()][start.x()] == 0) {
		map[start.y()][start.x()] = maxFromNeighbours(map, start);
	}

	while (curr != end) {
		QPair<QPoint, int> lowest = { curr, map[curr.y()][curr.x()] };
		for (size_t i = 0; i < dx.size(); i++) {
			QPoint next = curr + QPoint(dx[i], dy[i]);
			if (isTileValid(map, next) && !isTileObstacle(map, next) && lowest.second > map[next.y()][next.x()] && lowest.second - map[next.y()][next.x()] < 3) {
				lowest = { next, map[next.y()][next.x()] };
			}
		}

		qDebug() << "Lowest: " << lowest.first << " " << lowest.second;
		path.push_back(lowest.first);
		curr = lowest.first;
	}

	path.push_back(curr);

	return path;
}

QVector<QPointF> FloodPlanner::prunePath(const QVector<QPoint> &path)
{
	auto curr = path.begin();
	auto next = path.begin() + 1;
	QPoint lastDiff = { next->x() - curr->x(), next->y() - curr->y() };

	QVector<QPoint> output;

	while (next != path.end()) {
		QPoint diff = { next->x() - curr->x(), next->y() - curr->y() };

		if (diff != lastDiff) {
			output.push_back(*curr);
			lastDiff = diff;
		}

		curr = next;
		next++;
	}

	qDebug() << "Points: " << output;

	for (size_t i = 0; i < output.length() - 1; i++) {
		QPoint distance = output[i + 1] - output[i];
		if (distance == QPoint(1, 0) || distance == QPoint(0, 1)) {
			output.remove(i);
		}
	}

	if (output[0] - output[1] == QPoint(1, 0) || output[0] - output[1] == QPoint(0, 1)) {
		output.remove(1);
	}
	output.remove(0);

	printMapWithPath(output);
	qDebug() << "Points: " << output;

	QVector<QPointF> outputF;
	std::transform(output.begin(), output.end(), std::back_inserter(outputF), [this](const QPoint &p) { return toWorlCoord(p); });

	outputF = removeUnnecessaryPoints(outputF);
	qDebug() << "After pruning: " << outputF;

	std::cout << "Pruned:\n";
	printMapWithPath(outputF);

	return outputF;
}

QPointF absoluteDiff(const QPointF &a, const QPointF &b)
{
	return QPointF(std::abs(a.x() - b.x()), std::abs(a.y() - b.y()));
}

QVector<QPoint> FloodPlanner::generatePoints(const QPointF &start, const QPointF &end, uint samples)
{
	QVector<QPoint> points;

	double step = (end - start).x() / samples;
	if (step == 0) {
		return QVector<QPoint> { toMapCoord(end) };
	}
	else if (std::abs(step) < 0.1) {
		step = (end - start).x() / 2.;
	}

	QPointF line = computeLineParameters(start, end);
	QPointF mid = start;
	qDebug() << "start: " << start << " end: " << end << " line: " << line;
	qDebug() << "start: " << toMapCoord(start) << " end: " << toMapCoord(end) << " line: " << line;

	QPointF diff = absoluteDiff(mid, end);
	while (mid != end) {
		points.push_back(toMapCoord(mid));
		qDebug() << "mid: " << mid;
		mid = { mid.x() + step, line.x() * (mid.x() + step) + line.y() };
		if (absoluteDiff(mid, end) > diff) {
			break;
		}
		diff = absoluteDiff(mid, end);
	}

	points.push_back(toMapCoord(end));
	points = points.toList().toSet().toList().toVector();
	qDebug() << "Returning path: " << points;
	return points;
}

QVector<QPointF> FloodPlanner::removeUnnecessaryPoints(const QVector<QPointF> &path)
{
	uint samples = 10;
	auto curr = path.begin();
	auto next = curr + 2;
	QVector<QPointF> output;

	while (next != path.end()) {
		output.push_back(*curr);
		QVector<QPoint> midPoints = generatePoints(*curr, *next, samples);

		bool inCollision = false;
		for (size_t i = 0; i < midPoints.size(); i++) {
			auto &tmp = midPoints[i];
			if (!isInCFree(tmp)) {
				inCollision = true;
				break;
			}
		}

		if (inCollision) {
			curr = next - 1;
			next = curr + 2;
			continue;
		}
		else {
			// Just so that there are no duplicate points.
			next++;
			if (next != path.end()) {
				output.pop_back();
			}
		}
	}
	output.push_back(path.back());

	return output;
}

QPair<QVector<int>, QVector<int>> FloodPlanner::getDirections(TrajectoryType type)
{
	// First is dx
	// Second is dy
	// (-1,-1) | (0, -1) | (1,-1)
	// (-1, 0) | (0,  0) | (1, 0)
	// (-1, 1) | (0,  1) | (1, 1)
	QPair<QVector<int>, QVector<int>> dirs;
	if (type == TrajectoryType::Manhattan) {
		dirs.first = { 0, 1, 0, -1 };
		dirs.second = { -1, 0, 1, 0 };
	}
	else {
		dirs.first = { 0, 1, 1, 1, 0, -1, -1, -1 };
		dirs.second = { -1, -1, 0, 1, 1, 1, 0, -1 };
	}

	return dirs;
}

void FloodPlanner::printMapWithPath(const QVector<QPoint> &points)
{
	Map map = *m_map;
	for (const auto &p : points) {
		map[p.y()][p.x()] = START_FLAG;
	}

	std::cout << map << std::endl;
}

void FloodPlanner::printMapWithPath(const QVector<QPointF> &points)
{
	Map map = *m_map;
	for (const auto &p : points) {
		auto tmp = toMapCoord(p);
		map[tmp.y()][tmp.x()] = START_FLAG;
	}

	std::cout << map << std::endl;
}

std::ostream &operator<<(std::ostream &os, const FloodPlanner::Map &map)
{
	os << "  |";
	for (size_t i = 0; i < map[0].size(); i++) {
		os << std::setw(2) << i << "|";
	}
	os << std::endl << std::string(map[0].size() * 3 + 3, '-') << std::endl;

	int idx = 0;
	for (const auto &row : map) {
		int nu = 0;
		os << std::setw(2) << idx++ << "|";
		for (const auto &tile : row) {
			if (tile == 0) {
				os << "  ";
			}
			else if (tile > 0) {
				os << std::setw(2) << tile;
			}
			else if (tile == START_FLAG) {
				os << "SS";
			}
			else if (tile == END_FLAG) {
				os << "EE";
			}
			else {
				os << "##";
			}
			os << '|';
		}
		os << std::endl << std::string(row.size() * 3 + 3, '-') << std::endl;
	}
	return os;
}
