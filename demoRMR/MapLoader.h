#ifndef MAP_LOADER_H
#define MAP_LOADER_H
#include <vector>

#include <QPointF>

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
	QPointF line;
};

class MapLoader
{
public:
	explicit MapLoader(double width, double height);
	double minX;
	double maxX;
	double minY;
	double maxY;

	double m_height;
	double m_width;
	QVector<WallObject> loadMap(const char filename[], TMapArea &mapss);
};

#endif // MAP_LOADER_H
