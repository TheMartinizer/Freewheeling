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

MapWindow::MapWindow(QWidget *parent) : QWidget(parent), mapWidget(new Marble::MarbleWidget()), pointMap(nullptr), worker(MarbleWorker(this)), progressBar(QProgressBar(this)), amountOfPointsSlider(QSlider(Qt::Horizontal, this)), amountOfPointsLabel(QString("Showing 10 starting points"), this) {
	
	this->connect(&worker, &MarbleWorker::mapUpdated, this, &MapWindow::receiveMapUpdated);
	this->connect(mapWidget, &MarbleWidget::zoomChanged, this, &MapWindow::handleZoomChanged);
	
	mapWidget->setProjection(Mercator);
	mapWidget->setMapThemeId("earth/openstreetmap/openstreetmap.dgml");
	mapWidget->setShowCrosshairs(false);
	mapWidget->setShowGrid(false);
	mapWidget->setShowCompass(false);
	mapWidget->setShowOverviewMap(false);
	mapWidget->inputHandler()->setMouseButtonPopupEnabled(Qt::LeftButton, false);
	//mapWidget->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
	
	// Get menu bar going
	QVBoxLayout *layout = new QVBoxLayout(this);
	layout->addWidget(mapWidget);
	
	QMenuBar *menuBar = new QMenuBar();
	QMenu *fileMenu = new QMenu("File");
	
	fileMenu->addAction("Load GML", this, SLOT(loadGml()));
	fileMenu->addAction("Find starting points", this, SLOT(findStartingPoints()));
	fileMenu->addAction("Quit");
	
	this->connect(mapWidget, &MarbleWidget::mouseClickGeoPosition, this, &MapWindow::receiveMouseClicked);
	this->connect(&amountOfPointsSlider, &QSlider::valueChanged, this, &MapWindow::receiveSliderMoved);
	this->connect(&amountOfPointsSlider, &QSlider::sliderReleased, this, &MapWindow::receiveSliderReleased);
	
	menuBar->addMenu(fileMenu);
	layout->setMenuBar(menuBar);
	
	amountOfPointsLabel.setAlignment(Qt::AlignCenter);
	layout->addWidget(&amountOfPointsLabel);
	amountOfPointsSlider.setTickInterval(1);
	amountOfPointsSlider.setRange(1, 100);
	amountOfPointsSlider.setValue(amountOfPoints);
	layout->addWidget(&amountOfPointsSlider);
	layout->addWidget(&progressBar);
	
	layout->setStretchFactor(mapWidget, 1);
	layout->setStretchFactor(&amountOfPointsLabel, 0);
}

void MapWindow::loadGml() {
	QStringList fileNames = QFileDialog::getOpenFileNames(this, tr("Load GML file"), "", tr("GML Files (*.gml);;All Files (*)"));
	worker.loadData(fileNames);
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

void MapWindow::receiveSliderMoved(int newValue) {
	QString amountOfPointsString("Showing %1 starting points");
	if (newValue == 100) {
		amountOfPointsString = amountOfPointsString.arg("all");
	} else {
		amountOfPointsString = amountOfPointsString.arg(newValue);
	}
	
	amountOfPointsLabel.setText(amountOfPointsString);
	amountOfPoints = newValue;
	
	if (!amountOfPointsSlider.isSliderDown()) {
		receiveSliderReleased();
	}
}

void MapWindow::receiveSliderReleased() {
	worker.receiveRedrawStartingPoints(amountOfPoints);
}

void MapWindow::handleZoomChanged(int zoomLevel) {
	
}