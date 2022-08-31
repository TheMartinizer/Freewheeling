#ifndef MARBLEWORKER_H
#define MARBLEWORKER_H

#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <marble/GeoDataDocument.h>

#include "PointMap.h"
#include "RouteGraph.h"

class MarbleWorker : public QThread {
	Q_OBJECT
	
public:
	MarbleWorker(QObject *parent);
	~MarbleWorker();
	
	void loadData(QString filename);
	void findStartingPoints();
	void receiveClick(float lat, float lon);
	
signals:
	void mapUpdated(Marble::GeoDataDocument *document, bool appendDocument, QString progressMessage, int progressPercent);

protected:
	void run() override;

private:
	QMutex mutex;
	QWaitCondition condition;
	
	bool stop = false;
	
	PointMap *pointMap = nullptr;
	std::vector<std::shared_ptr<RouteGraph>> routeGraphs;
	
	bool shouldLoadFile = false;
	QString filename = "";
	
	bool shouldFindStartingPoints = false;
	
	bool receivedClick = false;
	float clickedLat = 0;
	float clickedLon = 0;
	
	static Marble::GeoDataDocument *createDocument(Progress progress, int roads);
	static void addRouteGraphsToDocument(std::vector<std::shared_ptr<RouteGraph>> routeGraphs, Marble::GeoDataDocument *document, int number = -1);
	static void addRouteGraphToDocument(std::shared_ptr<RouteGraph>, Marble::GeoDataDocument *document);
};

#endif /* MARBLEWORKER_H */

