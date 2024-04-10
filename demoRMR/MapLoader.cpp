#include "MapLoader.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include <limits>

MapLoader::MapLoader()
{
	minX = std::numeric_limits<double>::max();
	maxX = std::numeric_limits<double>::min();
	minY = std::numeric_limits<double>::max();
	maxY = std::numeric_limits<double>::min();
}

void MapLoader::loadMap(const char filename[], TMapArea &mapss)
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
		;
		;
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
}

