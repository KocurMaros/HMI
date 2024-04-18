#ifndef MAP_LOADER_H
#define MAP_LOADER_H
#include <qobject.h>
#include <vector>

#include <QPointF>
#include <QVector>

#define DEAD_POINT QPointF(11,481.019)

typedef struct
{
	double x;
	double y;

} Point;

typedef union {
	Point point;
	double suradnice[2];
} TMapPoint;

typedef struct
{
	int numofpoints;
	std::vector<TMapPoint> points;
} TMapObject;

typedef struct
{
	TMapObject wall;
	int numofObjects;
	std::vector<TMapObject> obstacle;
} TMapArea;

struct WallObject
{
	QPointF start;
	QPointF end;
};

std::ostream &operator<<(std::ostream &os, const WallObject &obj);

class MapLoader
	: public QObject
{
public:
	explicit MapLoader(QObject *parrent, double width, double height);
	QPointF toMapPoint(const QPointF &point);
	QPointF toWorldPoint(const QPointF &point);
	void loadMap(const char filename[]);
	double distance(QPointF p1, QPointF p2);
	double distanceFromPointToLine(QPointF point, QPointF lineStart, QPointF lineEnd);
	bool isLineInCollision(const QPointF &start, const QPointF &end);
	QVector<WallObject> walls();
	TMapArea mapArea() const { return m_mapArea; }

	double minX;
	double maxX;
	double minY;
	double maxY;

	double m_height;
	double m_width;
	QVector<WallObject> m_walls;
	TMapArea m_mapArea;
};

#endif // MAP_LOADER_H
