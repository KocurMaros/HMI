#ifndef MAP_LOADER_H
#define MAP_LOADER_H
#include <qobject.h>
#include <vector>

#include <QPointF>
#include <QVector>
#include <QFileDialog>

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
	bool isLoaded() const { return !m_walls.isEmpty(); }
	QPointF toMapPoint(const QPointF &point);
	QPointF toWorldPoint(const QPointF &point);
	void loadMap(const char filename[]);
	void loadMap(const QString filename) { loadMap(filename.toStdString().c_str()); }
	double distance(QPointF p1, QPointF p2);
	double distanceFromPointToLine(QPointF point, QPointF lineStart, QPointF lineEnd);
	bool isLineInCollision(const QPointF &start, const QPointF &end);
	QVector<WallObject> walls();
	TMapArea mapArea() const { return m_mapArea; }

// slots
	void on_loadMapButton_clicked(bool clicked);

	double minX;
	double maxX;
	double minY;
	double maxY;

	double m_height;
	double m_width;
	QVector<WallObject> m_walls;
	TMapArea m_mapArea;
	QFileDialog *m_fileDialog;
};

#endif // MAP_LOADER_H
