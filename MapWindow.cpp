#include "MapWindow.h"

#include <QVBoxLayout>
#include <QMenuBar>
#include <QFileDialog>
#include <marble/GeoDataMultiGeometry.h>
#include <marble/GeoDataLineString.h>
#include <marble/GeoDataDocument.h>
#include <marble/MarbleModel.h>
#include <marble/GeoDataTreeModel.h>
#include <marble/GeoDataPlacemark.h>
#include <marble/GeoDataLineStyle.h>
#include <marble/GeoDataLinearRing.h>
#include <marble/MarbleWidgetInputHandler.h>

using namespace Marble;

MapWindow::MapWindow(QWidget *parent) : QWidget(parent), mapWidget(new Marble::MarbleWidget()), pointMap(nullptr), worker(MarbleWorker(this)), progressBar(QProgressBar(this)) {
	
	this->connect(&worker, &MarbleWorker::mapUpdated, this, &MapWindow::receiveMapUpdated);
	
	mapWidget->setProjection(Mercator);
	mapWidget->setMapThemeId("earth/openstreetmap/openstreetmap.dgml");
	mapWidget->setShowCrosshairs(false);
	mapWidget->setShowGrid(false);
	mapWidget->setShowCompass(false);
	mapWidget->setShowOverviewMap(false);
	mapWidget->inputHandler()->setMouseButtonPopupEnabled(Qt::LeftButton, false);
	
	// Get menu bar going
	QVBoxLayout *layout = new QVBoxLayout(this);
	this->layout()->addWidget(mapWidget);
	
	QMenuBar *menuBar = new QMenuBar();
	QMenu *fileMenu = new QMenu("File");
	
	fileMenu->addAction("Load GML", this, SLOT(loadGml()));
	fileMenu->addAction("Find starting points", this, SLOT(findStartingPoints()));
	fileMenu->addAction("Quit");
	
	this->connect(mapWidget, &MarbleWidget::mouseClickGeoPosition, this, &MapWindow::receiveMouseClicked);
	
	menuBar->addMenu(fileMenu);
	this->layout()->setMenuBar(menuBar);
	
	this->layout()->addWidget(&progressBar);
}

void MapWindow::loadGml() {
	QString fileName = QFileDialog::getOpenFileName(this, tr("Load GML file"), "", tr("GML Files (*.gml);;All Files (*)"));
	worker.loadData(fileName);
}

void MapWindow::findStartingPoints() {
	worker.findStartingPoints();
}

void MapWindow::receiveMouseClicked(float lon, float lat, GeoDataCoordinates::Unit unit) {
	if (unit == GeoDataCoordinates::Radian) {
		lon *= 180 / M_PI;
		lat *= 180 / M_PI;
	}
	
	worker.receiveClick(lat,lon);
}

void MapWindow::exit() {
	
}

void MapWindow::receiveMapUpdated(GeoDataDocument *document, bool appendDocument, QString message, int percent) {
	if (!appendDocument) {
		for (GeoDataDocument *document : documents) {
			mapWidget->model()->treeModel()->removeDocument(document);
			delete document;
		}
		
		documents.clear();
	}
	
	if (document != nullptr) {
		mapWidget->model()->treeModel()->addDocument(document);
		documents.push_back(document);
	}
	
	progressBar.setMinimum(0);
	progressBar.setMaximum(100);
	progressBar.setValue(percent);
	progressBar.setFormat(message);
}