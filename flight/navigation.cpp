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



static double latlong_to_dis(float pos1, float pos2){
	delta_lon = pos2.lon - pos1.lon;
	delta_lat = pos2.lat - pos1.lat;
	double a = sin(delta_lat/2)*sin(delta_lat/2) + 
		cos(pos1.lat) * cos(pos2.lat) * sin(delta_lon/2) * sin(delta_lon/2);
	double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return 6371 * c;
}

static double latlong_to_bear(float pos1, float pos2){
    double y = sin(pos2.lon - pos1.lon) * cos(pos2.lat);
    double x = cos(pos1.lat) * sin(pos2.lat) - sin(pos1.lat)*cos(pos1.lat) * cos(pos2.lon - pos1.lon);
    return atan2(y,x); 

}



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

		wp_distance = latlong_to_dis( current_location,  destination);
       
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
        wp_bearing = latlong_to_bear(current_location, destination);
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
