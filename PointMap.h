#ifndef POINTMAP_H
#define POINTMAP_H

#include <vector>
#include <map>
#include "ogrsf_frmts.h"

#include "DataStructures.h"
#include "Constants.h"

// The PointMap is used to quickly look up points by location. Given minimum and maximum values for the three coordinates,
// and a number of bins N, the PointMap divides the volume inside the given coordinates into N*N*N cubes. This should make
// it easier to look up points during the mapping process where the connections between each point is found.
class PointMap {
private:
	double minimumX, maximumX, minimumY, maximumY, minimumZ, maximumZ;
	int numberOfBins = NUMBER_OF_BINS;
	double pointThreshold = POINT_THRESHOLD;
	std::map<int, std::shared_ptr<std::vector<std::shared_ptr<RoadPoint>>>> pointMap;
	std::vector<std::shared_ptr<RoadPoint>> roadPoints;

public:
	PointMap(GDALDataset *dataset);
	PointMap(GDALDataset *dataset, bool printProgress);
	std::shared_ptr<RoadPoint> getPoint(double x, double y, double z);
	std::vector<std::shared_ptr<RoadPoint>> getAllPoints();
};

#endif // POINTMAP_H
