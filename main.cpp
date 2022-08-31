#include <iostream>
#include <float.h>
#include <algorithm>
#include <stdio.h>
#include <cmath>
#include <chrono>
#include <unordered_map>
#include <unordered_set>

#include "ogrsf_frmts.h"
#include <QApplication>

#include "DataStructures.h"
#include "PointMap.h"
#include "Constants.h"
#include "MapWindow.h"

using namespace std;

int main(int argc, char** argv)
{
	// Load GDAL
	GDALAllRegister();
	
	QApplication app(argc, argv);
	MapWindow *mapWidget = new MapWindow();
	mapWidget->show();
	return app.exec();
}
