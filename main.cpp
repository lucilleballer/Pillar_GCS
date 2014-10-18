#include <iostream>
#include <QCoreApplication>
#include <QTimer>

#include "UAVTalk.h"

#include <unistd.h>

using namespace std;

int main(int argc, char *argv[]) {
	QCoreApplication app(argc, argv);
	cout << "Pillar GCS v0.01\n";
	UAVTalk uavtalk;

	//while(1) cerr << uavtalk.readByte();

	uavtalk_message_t msg;
	
	for (int i = 0; i < 100000; ++i) {
	while(!uavtalk.read(msg));
	//cout << "Sync: " << (int) msg.Sync << endl;
	//cout << "Type: " << (int) msg.MsgType << endl;
	//cout << "Length: " << (int) msg.Length << endl;
	//cout << "ObjID: " << (unsigned int) msg.ObjID << endl;
	cout << "Data Length: " << msg.Length << endl;
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
	cout << "Roll: " << (float) uavtalk.uav_roll << endl;
	cout << "Pitch: " << uavtalk.uav_pitch << endl;
	cout << "Heading: " << uavtalk.uav_heading << endl;
	/*cout << "Batt: " << (int) uavtalk.uav_bat << endl;
	cout << "Alt: " << (int) uavtalk.uav_alt << endl;
	cout << "Arm: " << (int) uavtalk.uav_arm << endl;
	cout << "Flight mode: " << (int) uavtalk.uav_flightmode << endl;*/
	//uint32_t uav_lat;
	//uint32_t uav_lon;
	/*cout << "Satellites visible: " << (int) uavtalk.uav_satellites_visible << endl;
	cout << "GPS Heading: " << (int) uavtalk.uav_gpsheading << endl;
	cout << "Ground speed: " << (int) uavtalk.uav_groundspeed << endl;
	cout << "Current: " << (int) uavtalk.uav_current << endl;
	cout << "Amp: " << (int) uavtalk.uav_amp << endl;*/
	}

	/*do { 
		uavtalk.write(0x42);
	} while (uavtalk.readByte() != 0x42);

	cout << "Starting\n";
	//QTimer::singleShot(200, &app, SLOT(quit())); //stop after 5 seconds
	uint8_t temp1;
	uint8_t temp2;
	while (1) {
	for(int j = 0; j < 6; ++j) {
		temp1 = uavtalk.readByte();
		temp2 = uavtalk.readByte();
		uint16_t temp = temp1 << 8 | temp2;
		cout << (short)temp << '\t';
	}
	cout << endl;
	}*/




	return app.exec();
}
