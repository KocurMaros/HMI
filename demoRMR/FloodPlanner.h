#pragma once

#include <memory>

#include <QObject>
#include <QPoint>
#include <QString>
#include <QVector>

#define TILE_SIZE 280. // 280 mm => 28cm

class FloodPlanner : public QObject
{
	Q_OBJECT;

	using Map = QVector<QVector<int>>;
	enum class TrajectoryType {
		Diagonal,
		Manhattan,
	};

	friend std::ostream &operator<<(std::ostream &os, const Map &map);

public:
	explicit FloodPlanner(const QString &filename);

public slots:
	void on_requestPath_plan(const QPointF &start, const QPointF &end);

public:
signals:
	void pathPlanned(QVector<QPointF> path);

private:
	QPoint toMapCoord(const QPointF &point);
	QPointF toWorlCoord(const QPoint &point);

	void loadMap(const QString &filename);
	void fillMap(const QString &filename);
	void expandObstacles();

	bool isInCFree(const QPoint &point);
	QPoint nearestCFreePoint(const QPoint &point, TrajectoryType type);

	int maxFromNeighbours(const Map &map, const QPoint &point);
	bool isTileObstacle(const Map &map, const QPoint &point);
	bool isTileValid(const Map &map, const QPoint &point);
	void markTiles(Map &map, const QPoint &start, const QPoint &end, TrajectoryType type);
	QVector<QPointF> planPath(const QPoint &start, const QPoint &end, TrajectoryType type = TrajectoryType::Manhattan);
	QVector<QPoint> pathFromMap(Map &map, const QPoint &start, const QPoint &end, TrajectoryType type);
	QVector<QPointF> prunePath(const QVector<QPoint> &path);

	QVector<QPoint> generatePoints(const QPointF &start, const QPointF &end, uint samples);
	QVector<QPointF> removeUnnecessaryPoints(const QVector<QPointF> &path);

	QPair<QVector<int>, QVector<int>> getDirections(TrajectoryType type);

	void printMapWithPath(const QVector<QPoint> &points);
	void printMapWithPath(const QVector<QPointF> &points);

private:
	std::shared_ptr<Map> m_map;
	QPoint m_start;
	QPoint m_end;
};

std::ostream &operator<<(std::ostream &os, const FloodPlanner::Map &map);
