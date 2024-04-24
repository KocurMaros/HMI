#include "MapLoader.h"
#include "mainwindow.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include <limits>
#include <iostream>
#include <QDebug>
#include <qwindowdefs.h>

#define DOUBLE_MAX std::numeric_limits<double>::max()

static QPointF createLineParams(const QPointF &start, const QPointF &end)
{
	QPointF line;
	// Compute slope (a)
	if (start.x() != end.x()) {
		auto x = (end.y() - start.y()) / (end.x() - start.x());
		line.setX(x);
	}
	else {
		// If the line is vertical, slope is infinity, so set a to a large value
		line.setY(1e9);
	}
	// Compute intercept (b)
	line.ry() = start.y() - line.x() * start.x();
	return line;
}

MapLoader::MapLoader(QObject *parrent, double width, double height)
	: QObject(parrent)
	, m_width(width)
	, m_height(height)
{
	minX = std::numeric_limits<double>::max();
	maxX = std::numeric_limits<double>::min();
	minY = std::numeric_limits<double>::max();
	maxY = std::numeric_limits<double>::min();
}

QPointF MapLoader::toMapPoint(const QPointF &point)
{
	QRect rect;
	rect = qobject_cast<MainWindow *>(parent())->getFrameGeometry();
	rect.translate(0, 37);

	double xrobot = rect.x() + rect.width() * (point.x() - minX) / (maxX - minX);
	double yrobot = rect.y() + rect.height() - rect.height() * (point.y() - minY) / (maxY - minY);

	return QPointF(xrobot, yrobot);
}

QPointF MapLoader::toWorldPoint(const QPointF &mapPoint)
{
	QRect rect;
	auto win = qobject_cast<MainWindow *>(parent());

	double mapX = ((mapPoint.x() - win->m_ui->frame->x()) * (maxX - minX) / win->m_ui->frame->width() + minX) / 100 - 0.5;
	double mapY = ((win->m_ui->frame->height() - (mapPoint.y() - win->m_ui->frame->y())) * (maxY - minY) / win->m_ui->frame->height() + minY) / 100 - 0.25;

	return QPointF(mapX, mapY);
}

void MapLoader::loadMap(const char filename[])
{
	FILE *fp = fopen(filename, "r");
	if (fp == NULL) {
		printf("zly file\n");
		return;
	}

	//tu nacitame obvodovu stenu
	char myLine[550];
	fgets(myLine, 550, fp);
	printf("%s\n", myLine);
	char *myCopy = (char *)calloc(strlen(myLine) + 2, sizeof(char));
	memcpy(myCopy, myLine, sizeof(char) * strlen(myLine));
	char *freeMyCopy;
	freeMyCopy = myCopy;
	myCopy = strtok(myCopy, "[]");
	m_mapArea.wall.numofpoints = (atoi(myCopy));
	printf("num of points %i\n", m_mapArea.wall.numofpoints);
	m_mapArea.wall.points.reserve(m_mapArea.wall.numofpoints);
	for (int i = 0; i < m_mapArea.wall.numofpoints; i++) {
		TMapPoint temp;
		myCopy = strtok(NULL, "[,");
		temp.point.x = atof(myCopy);
		myCopy = strtok(NULL, "[,");
		temp.point.y = atof(myCopy);
		m_mapArea.wall.points.push_back(temp);
		//	 m_mapArea.wall.points[i/2].suradnice[i%2]=atof(myCopy);

		minX = temp.point.x < minX ? temp.point.x : minX;
		maxX = temp.point.x > maxX ? temp.point.x : maxX;
		minY = temp.point.y < minY ? temp.point.y : minY;
		maxY = temp.point.y > maxY ? temp.point.y : maxY;
	}
	free(freeMyCopy);

	//tu nacitame jednotlive prekazky
	m_mapArea.numofObjects = 0;
	m_mapArea.obstacle.clear();
	while (fgets(myLine, 550, fp)) {
		printf("%s\n", myLine);
		myCopy = (char *)calloc(strlen(myLine) + 2, sizeof(char));
		memcpy(myCopy, myLine, sizeof(char) * strlen(myLine));

		freeMyCopy = myCopy;
		myCopy = strtok(myCopy, "[]");
		if ((atoi(myCopy)) == 0)
			break;
		TMapObject tempObstacle;
		m_mapArea.numofObjects++;

		tempObstacle.numofpoints = (atoi(myCopy));
		for (int i = 0; i < tempObstacle.numofpoints; i++) {
			TMapPoint temp;
			myCopy = strtok(NULL, "[,");
			temp.point.x = atof(myCopy);
			myCopy = strtok(NULL, "[,");
			temp.point.y = atof(myCopy);
			tempObstacle.points.push_back(temp);
		}
		free(freeMyCopy);
		m_mapArea.obstacle.push_back(tempObstacle);
	}

	fflush(stdout);

	auto objects = walls();

	m_walls = std::move(objects);
}

double MapLoader::distance(QPointF p1, QPointF p2)
{
	return sqrt(pow(p2.x() - p1.x(), 2) + pow(p2.y() - p1.y(), 2));
}

// Function to calculate distance from a point to a line segment
double MapLoader::distanceFromPointToLine(QPointF point, QPointF lineStart, QPointF lineEnd)
{
	double l2 = pow(distance(lineStart, lineEnd), 2); // Length of the line segment squared

	// If both points are the same, return distance between the point and one of the endpoints
	if (l2 == 0) {
		return distance(point, lineStart);
	}

	// Parametric value of projection of point p onto the line segment
	double t = ((point.x() - lineStart.x()) * (lineEnd.x() - lineStart.x()) + (point.y() - lineStart.y()) * (lineEnd.y() - lineStart.y())) / l2;

	// Clamp t to ensure it's within the line segment
	t = std::max(0.0, std::min(1.0, t));

	// Projection point on the line segment
	QPointF projection = { lineStart.x() + t * (lineEnd.x() - lineStart.x()), lineStart.y() + t * (lineEnd.y() - lineStart.y()) };

	// Return distance between point p and its projection on the line segment
	return distance(point, projection);
}

bool MapLoader::isLineInCollision(const QPointF &start, const QPointF &end)
{
	QLineF trajectory(start, end);
	for (auto &wall : m_walls) {
		QPointF intersectionPoint = { DOUBLE_MAX, DOUBLE_MAX };
		QLineF line(wall.start, wall.end);

		line.intersect(trajectory, &intersectionPoint);
		if (intersectionPoint == QPointF(DOUBLE_MAX, DOUBLE_MAX)) {
			continue;
		}

		if (distanceFromPointToLine(end, wall.start, wall.end) < 29) {
			return true;
		}
		if (distanceFromPointToLine(wall.start, start, end) < 29) {
			return true;
		}
		if (distanceFromPointToLine(wall.end, start, end) < 29) {
			return true;
		}
	}
	return false;
}

std::ostream &operator<<(std::ostream &os, const WallObject &obj)
{
	os << "Start: " << obj.start.x() << ", " << obj.start.y() << std::endl;
	os << "End: " << obj.end.x() << ", " << obj.end.y() << std::endl;
	return os;
}

QVector<WallObject> MapLoader::walls()
{
	QVector<WallObject> objects;

	for (int i = 0; i < m_mapArea.wall.points.size(); i++) {
		double xmin = m_mapArea.wall.points[i].point.x;
		double xmax = m_mapArea.wall.points[(i + 1) % m_mapArea.wall.points.size()].point.x;
		double ymin = m_mapArea.wall.points[i].point.y;
		double ymax = m_mapArea.wall.points[(i + 1) % m_mapArea.wall.points.size()].point.y;

		auto min = toMapPoint({ xmin, ymin });
		auto max = toMapPoint({ xmax, ymax });

		objects.push_back({ min, max });
		// painter.drawLine(min, max);
	}

	// qDebug() << "Current position is " << m_mapLoader->toMapPoint({getX(), getY()});

	for (int i = 0; i < m_mapArea.obstacle.size(); i++) {
		for (int j = 0; j < m_mapArea.obstacle[i].points.size(); j++) {
			double xmin = m_mapArea.obstacle[i].points[j].point.x;
			double xmax = m_mapArea.obstacle[i].points[(j + 1) % m_mapArea.obstacle[i].points.size()].point.x;
			double ymin = m_mapArea.obstacle[i].points[j].point.y;
			double ymax = m_mapArea.obstacle[i].points[(j + 1) % m_mapArea.obstacle[i].points.size()].point.y;

			auto min = toMapPoint({ xmin, ymin });
			auto max = toMapPoint({ xmax, ymax });

			// qDebug() << "Drawing line from " << min << " to " << max;
			objects.push_back({ min, max });
			// painter.drawLine(min, max);
		}
	}

	m_walls = std::move(objects);
	return m_walls;
}
