/*
 * File:		mainwindow.h
 * Author:		Alex Liao
 * Desc:		Main GUI window for the Pillar GCS
 * Etc:			Requires Qt library
 */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtCore/QtGlobal>
#include <QMainWindow>
#include <QDialog>
#include <QLabel>

#include "UAVTalk.h"

//namespace Ui {
//	class MainWindow;
//}

class MainWindow : public QDialog
{
	Q_OBJECT

	public:
		MainWindow(QWidget *parent = 0);
		~MainWindow();

		void updateAttitudeState(const float r, 
				const float pitch, const float yaw);

		void updateAccelState(const float x,
				const float y, const float z);

		void updateGPSState(const int lat, const int lon,
				const int satellites, const int gpsheading,
				const int alt, const int speed);

	private:
		void initActionsConnections();

	private:
		QLabel* rollLabel;
		QLabel* pitchLabel;
		QLabel* yawLabel;

		QLabel* accelXLabel;
		QLabel* accelYLabel;
		QLabel* accelZLabel;

		QLabel* gyroXLabel;
		QLabel* gyroYLabel;
		QLabel* gyroZLabel;

		QLabel* latLabel;
		QLabel* lonLabel;
		QLabel* satellitesLabel;
		QLabel* gpsheadingLabel;
		QLabel* altLabel;
		QLabel* groundspeedLabel;

		//Ui::MainWindow *ui;

		UAVTalk* uavtalk;
};

#endif
