// Make sure that things are okay


// If gps failed
bool GPS_OK(){

	if(uav_satellites_visible >= 7) {

		return true;

	}
	
	else {

		return false;

	}


}


bool battery_OK(uav_bat){

	if (uav_bat <0.1) {
		// begin land function
		return false;
	} 

	else {

		return true;
	}

}


