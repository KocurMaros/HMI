#include "MapLoader.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include <limits>
#include <qpoint.h>
#include <qvector.h>

MapLoader::MapLoader(double width, double height)
	: m_width(width)
	, m_height(height)
{
	minX = std::numeric_limits<double>::max();
	maxX = std::numeric_limits<double>::min();
	minY = std::numeric_limits<double>::max();
	maxY = std::numeric_limits<double>::min();
}

QVector<WallObject> MapLoader::loadMap(const char filename[], TMapArea &mapss)
{
	FILE *fp = fopen(filename, "r");
	if (fp == NULL) {
		printf("zly file\n");
		return {};
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
	mapss.wall.numofpoints = (atoi(myCopy));
	printf("num of points %i\n", mapss.wall.numofpoints);
	mapss.wall.points.reserve(mapss.wall.numofpoints);
	for (int i = 0; i < mapss.wall.numofpoints; i++) {
		TMapPoint temp;
		myCopy = strtok(NULL, "[,");
		temp.point.x = atof(myCopy);
		myCopy = strtok(NULL, "[,");
		temp.point.y = atof(myCopy);
		mapss.wall.points.push_back(temp);
		//   mapss.wall.points[i/2].suradnice[i%2]=atof(myCopy);

		minX = temp.point.x < minX ? temp.point.x : minX;
		maxX = temp.point.x > maxX ? temp.point.x : maxX;
		minY = temp.point.y < minY ? temp.point.y : minY;
		maxY = temp.point.y > maxY ? temp.point.y : maxY;
	}
	free(freeMyCopy);

	//tu nacitame jednotlive prekazky
	mapss.numofObjects = 0;
	mapss.obstacle.clear();
	while (fgets(myLine, 550, fp)) {
		printf("%s\n", myLine);
		myCopy = (char *)calloc(strlen(myLine) + 2, sizeof(char));
		memcpy(myCopy, myLine, sizeof(char) * strlen(myLine));

		freeMyCopy = myCopy;
		myCopy = strtok(myCopy, "[]");
		if ((atoi(myCopy)) == 0)
			break;
		TMapObject tempObstacle;
		mapss.numofObjects++;

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
		mapss.obstacle.push_back(tempObstacle);
	}

	fflush(stdout);

	QVector<WallObject> objects;

	for (int i = 0; i < mapss.wall.points.size(); i++) {
		int xmin = m_width * (mapss.wall.points[i].point.x - minX) / (maxX - minX);
		int xmax = m_width * (mapss.wall.points[(i + 1) % mapss.wall.points.size()].point.x - minX)
			/ (maxX - minX);
		int ymin = m_height - m_height * (mapss.wall.points[i].point.y - minY) / (maxY - minY);
		int ymax = m_height
			- m_height * (mapss.wall.points[(i + 1) % mapss.wall.points.size()].point.y - minY) / (maxY - minY);

		objects.push_back({ QPointF(xmin, ymin), QPointF(xmax, ymax), QPointF(xmax - xmin, ymax - ymin) });
		// painter.drawLine(rect.x() + xmin, rect.y() + ymin, rect.x() + xmax, rect.y() + ymax);
	}

	for (int i = 0; i < mapss.obstacle.size(); i++) {
		for (int j = 0; j < mapss.obstacle[i].points.size(); j++) {
			int xmin = m_width * (mapss.obstacle[i].points[j].point.x - minX) / (maxX - minX);
			int xmax = m_width * (mapss.obstacle[i].points[(j + 1) % mapss.obstacle[i].points.size()].point.x - minX)
				/ (maxX - minX);
			int ymin = m_height - m_height * (mapss.obstacle[i].points[j].point.y - minY) / (maxY - minY);
			int ymax = m_height
				- m_height * (mapss.obstacle[i].points[(j + 1) % mapss.obstacle[i].points.size()].point.y - minY)
					/ (maxY - minY);
			objects.push_back({ QPointF(xmin, ymin), QPointF(xmax, ymax), QPointF(xmax - xmin, ymax - ymin) });
		}
	}
	return objects;
}
