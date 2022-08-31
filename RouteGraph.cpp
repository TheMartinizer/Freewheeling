#include "RouteGraph.h"

#include <unordered_map>
#include <cmath>

#include "Constants.h"

RouteGraph::RouteGraph(std::shared_ptr<RoadPoint> startingPoint) : startingPoint(startingPoint) {
	// Find all points that appear to be reachable from the starting point
	std::unordered_map<std::shared_ptr<RoadPoint>, std::pair<double, double>> possibleNext;
	
	double startingSpeed = MINIMUM_SPEED_KMH / 3.6;
	double initialEnergy = startingPoint->z * GRAVITY_CONSTANT + 0.5*pow(startingSpeed, 2);
	possibleNext[startingPoint] = std::pair<double, double>(initialEnergy, startingSpeed);
	
	while (possibleNext.size() > 0) {
		double highestEnergy = -1e12;
		double speed = 0;
		std::shared_ptr<RoadPoint> bestNext;
		
		for (auto const& keyValuePair : possibleNext) {
			std::shared_ptr<RoadPoint> point = keyValuePair.first;
			double energy = keyValuePair.second.first;
			if (energy > highestEnergy) {
				highestEnergy = energy;
				bestNext = point;
				speed = keyValuePair.second.second;
			}
		}
		
		// Find the distance
		double distance = sqrt(pow(bestNext->x - startingPoint->x,2) + pow(bestNext->y - startingPoint->y,2));
		if (distance > this->farthestDistance) {
			this->farthestDistance = distance;
			this->farthestEndpoint = bestNext;
		}
		
		// We found the best next point to search from
		possibleNext.erase(bestNext);
		
		if (bestNext != startingPoint) {
			reachablePoints.insert(bestNext);
		}
		
		// Find the points that are reachable from this point
		bool endPoint = true;
		for (std::shared_ptr<Connection> connection : bestNext->connections) {
			std::shared_ptr<RoadPoint> nextPoint = connection->connectedPoint;
			
			// Check if we have already been to this point first
			if (reachablePoints.count(nextPoint) == 1) {
				continue;
			}
			
			// Next, check if it is possible to go there from here
			double outputSpeed = connection->getOutputSpeed(speed);
			if (outputSpeed < 0) {
				continue;
			}
			
			endPoint = false;
			
			// Finally, add this point to the list of potential points, if the energy by going this route is higher
			// than the energy we have previously stored
			double nextEnergy = nextPoint->z * GRAVITY_CONSTANT + 0.5*pow(outputSpeed, 2);
			if (possibleNext.count(nextPoint) == 0 || nextEnergy > possibleNext[nextPoint].first) {
				possibleNext[nextPoint] = std::pair<double,double>(nextEnergy, outputSpeed);
			}
		}
		
		if (endPoint && distance > MINIMUM_ROUTE_DISTANCE) {
			endPoints.insert(bestNext);
		}
	}
}


