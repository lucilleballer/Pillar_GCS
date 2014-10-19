#include "mainwindow.h"
#include <QVBoxLayout>
//#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QDialog(parent)
    
{
    /*ui->setupUi(this);

    ui->actionConnect->setEnabled(true);
    ui->actionDisconnect->setEnabled(false);
    ui->actionQuit->setEnabled(true);
    ui->actionConfigure->setEnabled(true);*/

    //initActionsConnections();
	rollLabel = new QLabel("Roll: 0.00");
	pitchLabel = new QLabel("Pitch: 0.00");
	yawLabel = new QLabel("Yaw: 0.00");
	accelXLabel = new QLabel("Accel X: 0.00");
	accelYLabel = new QLabel("Accel Y: 0.00");
	accelZLabel = new QLabel("Accel Z: 0.00");
	latLabel = new QLabel("Lat: 0.00");
	lonLabel = new QLabel("Lon: 0.00");
	satellitesLabel = new QLabel("Satellites: 0.00");
	gpsheadingLabel = new QLabel("GPS Heading: 0.00");
	altLabel = new QLabel("Altitude: 0.00");
	groundspeedLabel = new QLabel("Ground Speed: 0.00");
	QVBoxLayout *mainLayout = new QVBoxLayout;
	mainLayout->addWidget(rollLabel);
	mainLayout->addWidget(pitchLabel);
	mainLayout->addWidget(yawLabel);
	mainLayout->addWidget(accelXLabel);
	mainLayout->addWidget(accelYLabel);
	mainLayout->addWidget(accelZLabel);
	mainLayout->addWidget(latLabel);
	mainLayout->addWidget(lonLabel);
	mainLayout->addWidget(satellitesLabel);
	mainLayout->addWidget(gpsheadingLabel);
	mainLayout->addWidget(altLabel);
	mainLayout->addWidget(groundspeedLabel);
	setLayout(mainLayout);
	//setCentralWidget(rollLabel);
	resize(300,300);

	uavtalk = new UAVTalk(this);
	uavtalk->openSerialPort();
}

MainWindow::~MainWindow()
{
    //delete ui;
	delete uavtalk;
}

void MainWindow::updateAttitudeState(const float roll, 
		const float pitch, const float yaw) 
{
	rollLabel->setText(QString("Roll: %1").arg(roll));
	pitchLabel->setText(QString("Pitch: %1").arg(pitch));
	yawLabel->setText(QString("Yaw: %1").arg(yaw));
}

void MainWindow::updateAccelState(const float x, 
		const float y, const float z) 
{
	accelXLabel->setText(QString("Accel X: %1").arg(x));
	accelYLabel->setText(QString("Accel Y: %1").arg(y));
	accelZLabel->setText(QString("Accel Z: %1").arg(z));
}

void MainWindow::updateGPSState(const int lat, const int lon,
		const int satellites, const int gpsheading,
		const int alt, const int speed)
{
	latLabel->setText(QString("Lat: %1").arg(lat));
	lonLabel->setText(QString("Lon: %1").arg(lon));
	satellitesLabel->setText(QString("Satellites: %1").arg(satellites));
	gpsheadingLabel->setText(QString("GPS Heading: %1").arg(gpsheading));
	altLabel->setText(QString("Altitude: %1").arg(alt));
	groundspeedLabel->setText(QString("Ground Speed: %1").arg(speed));
}
