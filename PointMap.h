#ifndef POINTMAP_H
#define POINTMAP_H

#include <functional>
#include <vector>
#include <map>
#include "ogrsf_frmts.h"

#include "DataStructures.h"
#include "Constants.h"

// We want to update the frontend with our progress on how far we have come doing various tasks
// Return this in a progress structure
struct Progress {
	std::string message;
	int percent;
	std::vector<std::vector<std::shared_ptr<RoadPoint>>> loadedRoads;
};

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
	int srcEpsg;
	OGRCoordinateTransformation *transformation;
	std::vector<std::vector<std::shared_ptr<RoadPoint>>> loadedRoads;
	
	void rebuildPointMap();

public:
	PointMap();
	
	void loadDataset(GDALDataset *dataset);
	void loadDataset(GDALDataset *dataset, std::function<void(Progress)> progressCallback);
	std::shared_ptr<RoadPoint> getPoint(double x, double y, double z, OGRSpatialReference *ref);
	std::vector<std::shared_ptr<RoadPoint>> getAllPoints();
	std::vector<std::vector<std::shared_ptr<RoadPoint>>> getLoadedRoads();
};

#endif // POINTMAP_H
