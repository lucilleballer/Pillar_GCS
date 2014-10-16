#include <iostream>
#include <QCoreApplication>
#include <QTimer>

#include "UAVTalk.h"

#include <unistd.h>

using namespace std;

int main(int argc, char *argv[]) {
	QCoreApplication app(argc, argv);
	UAVTalk uavtalk;

	cout << "Pillar GCS v0.01\n";
	do { 
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
	}


	return app.exec();
}
