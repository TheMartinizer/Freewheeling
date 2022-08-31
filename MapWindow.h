#ifndef MAPWINDOW_H
#define MAPWINDOW_H

#include <QApplication>
#include <QWidget>
#include <QObject>
#include <QProgressBar>
#include <marble/MarbleWidget.h>
#include <marble/GeoPainter.h>
#include <marble/GeoDataCoordinates.h>

#include "MarbleWorker.h"

class MapWindow : public QWidget {
	Q_OBJECT
	
public:
	explicit MapWindow(QWidget *parent = nullptr);
	
public Q_SLOTS:
	void loadGml();
	void findStartingPoints();
	void exit();
	
private slots:
	void receiveMapUpdated(Marble::GeoDataDocument *document, bool appendDocument, QString message, int percent);
	void receiveMouseClicked(float longitude, float latitude, Marble::GeoDataCoordinates::Unit unit);

private:
	MarbleWorker worker;
	
	Marble::MarbleWidget *mapWidget;
	PointMap *pointMap;
	std::vector<Marble::GeoDataDocument*> documents;
	QProgressBar progressBar;
};

#endif /* MAPWINDOW_H */

