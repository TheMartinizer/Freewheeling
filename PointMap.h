#ifndef POINTMAP_H
#define POINTMAP_H

#include <vector>
#include <map>

#include "DataStructures.h"

// The PointMap is used to quickly look up points by location. Given minimum and maximum values for the three coordinates,
// and a number of bins N, the PointMap divides the volume inside the given coordinates into N*N*N cubes. This should make
// it easier to look up points during the mapping process where the connections between each point is found.
class PointMap {
private:
	double minimumX, maximumX, minimumY, maximumY, minimumZ, maximumZ, pointThreshold;
	int numberOfBins;
	std::map<int, std::vector<RoadPoint*>*> pointMap;
	std::vector<RoadPoint*> roadPoints;

public:
	PointMap(
			double minimumX,
			double maximumX,
			double minimumY,
			double maximumY,
			double minimumZ,
			double maximumZ,
			int numberOfBins,
			double pointThreshold
	);
	~PointMap();

	RoadPoint* getPoint(double x, double y, double z);
	std::vector<RoadPoint*> getAllPoints();
};

#endif // POINTMAP_H
