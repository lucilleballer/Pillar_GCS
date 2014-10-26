// Calcuations before autopilot moves
 // current location
// need to define 

#include <stdio.h>      /* printf */
#include <math.h>       /* atan */

struct location{
  float lat;
  float lon;
  float alt;
  float x;
  float y;
  float z;
} ;

location current_location;
location destination;
float test;
float delta_lon, delta_lat,a, c, d;



static float latlong_to_cm(float pos1, float pos2){
	delta_lon = pos2.lon - pos1.lon;
	delta_lat = pos2.lat - pos1.lat;
	a = sin(delta_lat/2)*sin(delta_lat/2) + 
		cos(pos1.lat) * cos(pos2.lat) * sin(delta_lon/2) * sin(delta_lon/2);
	c = 2 * atan2(sqrt(a), sqrt(1-a));
	d = 6371 * c;
}
/*
int main(){

	
	test = latlong_to_cm(current_location, dest_location);
	printf ("The result is\n", test);


}*/


static void run_nav_updates(void)
{
    // fetch position from inertial navigation
    calc_position();

    // calculate distance and bearing for reporting and autopilot decisions
    calc_distance_and_bearing();

    // run autopilot to make high level decisions about control modes
    run_autopilot();
}

// calc_position - get lat and lon positions from inertial nav library
static void calc_position(){
    if( )//inertial_nav.position_ok() ) { // Currently the ArduCopter control for checking if the position has been initialized and we have GPS data
        
        // pull position from interial nav library
    	current_location.lat = uav_lat;
    	current_location.lon = uav_lon;
    	current_location.alt = uav_alt;

    }
}

// calc_distance_and_bearing - calculate distance and bearing to next waypoint and home
static void calc_distance_and_bearing()
{
    calc_wp_distance();
    calc_wp_bearing();
    calc_home_distance_and_bearing();
}

// calc_wp_distance - calculate distance to next waypoint for reporting and autopilot decisions
static void calc_wp_distance()
{
    if(autopilot_state= AUTO){

		latlong_to_cm(float current_location, float destination);
       
    }
 
 	else{
        wp_distance = 0;
    }
}

// calc_wp_bearing - calculate bearing to next waypoint for reporting and autopilot decisions
static void calc_wp_bearing()
{
    // get target from loiter or wpinav controller
    
   else if (control_mode == AUTO) {
        wp_bearing = wp_nav.get_wp_bearing_to_destination();
    } else {
        wp_bearing = 0;
    }
}

// calc_home_distance_and_bearing - calculate distance and bearing to home for reporting and autopilot decisions
static void calc_home_distance_and_bearing()
{
    Vector3f curr = inertial_nav.get_position();

    // calculate home distance and bearing
    if (GPS_ok()) {
        home_distance = pythagorous2(curr.x, curr.y);
        home_bearing = pv_get_bearing_cd(curr,Vector3f(0,0,0));

        // update super simple bearing (if required) because it relies on home_bearing
        update_super_simple_bearing(false);
    }
}

// run_autopilot - highest level call to process mission commands
static void run_autopilot()
{
    if (control_mode == AUTO) {
        // update state of mission
        // may call commands_process.pde's start_command and verify_command functions
        mission.update();
    }
}
