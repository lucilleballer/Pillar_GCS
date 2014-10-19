#include <iostream>
#include <QCoreApplication>
#include <QApplication>
#include <QPushButton>
#include <QTimer>

#include "UAVTalk.h"

#include "mainwindow.h"

using namespace std;

int main(int argc, char *argv[]) {
	QApplication app(argc, argv);
	
	MainWindow w;
	w.show();

	//UAVTalk uavtalk;
	//uavtalk_message_t msg;

	//cout << "Pillar GCS v0.10\n";

	/*for (int i = 0; i < 100000; ++i) {
		// Read in a message and update the data
		while(!uavtalk.read(msg));
		//cout << "Sync: " << (int) msg.Sync << endl;
		//cout << "Type: " << (int) msg.MsgType << endl;
		//cout << "Length: " << (int) msg.Length << endl;
		//cout << "ObjID: " << (unsigned int) msg.ObjID << endl;
		//cout << "Data Length: " << msg.Length << endl;
		//cout << "InstID: " << (int) msg.InstID << endl;
		if (msg.ObjID == 3621837156) {
			for (int i = 2; i < msg.Length-8; ++i) {
				if ((i-2) % 8 == 0) cout << endl;
				cout << (int) msg.Data[i] << '\t';
			}
		}
		cout << endl;
		cout << endl;
		//cout << "\nCrc: " << (int) msg.Crc << endl;

		cout << "RSSI: " << (int) uavtalk.uav_rssi << endl;
		cout << "Link: " << (int) uavtalk.uav_linkquality << endl;
		cout << "Roll: " << uavtalk.uav_roll << endl;
		cout << "Pitch: " << uavtalk.uav_pitch << endl;
		cout << "Heading: " << uavtalk.uav_heading << endl;
		cout << "Accel X: " << uavtalk.uav_accel_x << endl;
		cout << "Accel Y: " << uavtalk.uav_accel_y << endl;
		cout << "Accel Z: " << uavtalk.uav_accel_z << endl;
		cout << "Gyro X: " << uavtalk.uav_gyro_x << endl;
		cout << "Gyro Y: " << uavtalk.uav_gyro_y << endl;
		cout << "Gyro Z: " << uavtalk.uav_gyro_z << endl;
		cout << "Batt: " << uavtalk.uav_bat << endl;
		cout << "Alt: " << uavtalk.uav_alt << endl;
		cout << "Arm: " << (int) uavtalk.uav_arm << endl;
		cout << "Flight mode: " << (int) uavtalk.uav_flightmode << endl;
		cout << "Satellites visible: " << (int) uavtalk.uav_satellites_visible << endl;
		cout << "GPS Heading: " << (int) uavtalk.uav_gpsheading << endl;
		cout << "Ground speed: " << (int) uavtalk.uav_groundspeed << endl;
		cout << "Current: " << (int) uavtalk.uav_current << endl;
		cout << "Amp: " << (int) uavtalk.uav_amp << endl;
		cout << endl;
	}*/

	return app.exec();
}
